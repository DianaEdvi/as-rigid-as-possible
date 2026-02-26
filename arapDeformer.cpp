#include "arapDeformer.h"

#include <igl/cotmatrix.h>


ArapDeformer::ArapDeformer(const Eigen::MatrixXd& v, const Eigen::MatrixXi& f, std::vector<int>& anchors, std::vector<Eigen::Vector3d>& anchors_positions) :
V(v), F(f), anchor_indices(anchors), anchors_positions(anchors_positions){}

/**
 * Constructs the overdetermined system matrix (A) and pre-factors the solver
 * 1. Computes the Laplace-Beltrami operator (cotangent weights)
 * 2. Computes the original mesh curvature (delta)
 * 3. Appends anchors as extra rows to pin specific vertices
 * 4. Pre-computes the Normal Equations (A^T * A) to ensure symmetry
 * 5. Computes Cholesky decomposition for faster solving
 */
void ArapDeformer::populateAugmentedLaplacian(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, const double& anchorWeight){
    // Generate base cotangent matrix
    igl::cotmatrix(V, F, L_cot);

    delta = L_cot * V;

    // Store new data in triplets. More efficient when we rebuild the sparse matrix 
    std::vector<Eigen::Triplet<double>> triplets;

    // Traverse efficiently through sparse matrix
    for (int c = 0; c < L_cot.outerSize(); ++c){
        // Copy L_cot into triplets 
        for (Eigen::SparseMatrix<double>::InnerIterator it(L_cot, c); it; ++it){
            triplets.push_back(Eigen::Triplet<double>(it.row(), it.col(), it.value()));
        }
    }

    int currentRow = V.rows(); // begin establishing anchor data after Nth row 

    // Traverse through all the vertices we want to anchor
    for (int i = 0; i < anchor_indices.size(); ++i){
        // Add a new row for each anchor. Provide the location and weight
        triplets.push_back(Eigen::Triplet<double>(currentRow, anchor_indices[i], anchorWeight));
        currentRow++;
    }

    L_aug.resize(V.rows() + anchor_indices.size(), V.rows()); // Num rows, num cols. 
    // Num cols represents the number of vertices, not xyz. We are in Laplacian after all

    L_aug.setFromTriplets(triplets.begin(), triplets.end());

    // Normal Equations
    Eigen::SparseMatrix<double> prefactorized_L_aug = L_aug.transpose() * L_aug;

    // Compute Cholesky decomposition 
    solver.compute(prefactorized_L_aug);
    
    if (solver.info() != Eigen::Success) {
        std::cerr << "Decomposition failed!" << std::endl;
        return;
    }
}

/**
 * Constructs the Right-Hand Side (b) for the least-squares system
 * Top N rows: Original differential coordinates (delta)
 * Bottom C rows: Target 3D positions of anchor vertices
 */
void ArapDeformer::populateTargetMatrix(const std::vector<Eigen::Vector3d>& target_positions, double anchorWeight){
    target = Eigen::MatrixXd::Zero(L_aug.rows(), 3); // Num rows = total vertices + anchors, cols = xyz

    // Populate the first N rows with the original curvature (delta)
    //original edge vectors, rotated by the matrices in rotations, and weighted by the cotangent weights.

    for (int i = 0; i < delta.rows(); ++i){
        Eigen::Vector3d weighted_delta = Eigen::Vector3d::Zero();
        for (int j = 0; j < adjacent_indices[i].size(); ++j){
            int neighbor_index = adjacent_indices[i][j];
            double weight = L_cot.coeff(i, neighbor_index);
            Eigen::Vector3d original_edge = (V.row(neighbor_index) - V.row(i)).transpose();
            Eigen::Matrix3d rotation_target = rotations[i];
            Eigen::Matrix3d rotation_neighbor = rotations[neighbor_index];
            weighted_delta += (weight/2) * (rotation_target + rotation_neighbor) * original_edge;
        }
        target.row(i) = weighted_delta.transpose();
    }


    int currentRow = delta.rows();
    // Populate the N + C rows with the anchor vertex positions 
    for (int i = 0; i < anchor_indices.size(); ++i){
        target(currentRow + i, 0) = target_positions[i].x() * anchorWeight;
        target(currentRow + i, 1) = target_positions[i].y() * anchorWeight;
        target(currentRow + i, 2) = target_positions[i].z() * anchorWeight;
    }
}

/**
 * Computes the optimal vertex positions by solving the linear system by using the pre-factored Cholesky decomposition.
 * This effectively finds the vertex positions that minimize the shape deformation while preserving anchor positions.
 */
void ArapDeformer::solveLeastSquares(){
    // Normal Equations 
    Eigen::MatrixXd balanced_target = L_aug.transpose() * target;

    // Solve system 
    V_new = solver.solve(balanced_target);

    if (solver.info() != Eigen::Success) {
        std::cerr << "Solving failed!" << std::endl;
        return;
    }

}

void ArapDeformer::precomputeRotations() {
    
    // Create adjacency list for each vertex
    adjacent_indices.resize(V.rows()); 
    for (int i = 0; i < F.rows(); ++i) {
        int v0 = F(i, 0);
        int v1 = F(i, 1);   
        int v2 = F(i, 2);

        auto it = std::find(adjacent_indices[v0].begin(), adjacent_indices[v0].end(), v1);
        if (it == adjacent_indices[v0].end()) {
            adjacent_indices[v0].push_back(v1);
        }
        it = std::find(adjacent_indices[v0].begin(), adjacent_indices[v0].end(), v2);
        if (it == adjacent_indices[v0].end()) {
            adjacent_indices[v0].push_back(v2);
        }
        it = std::find(adjacent_indices[v1].begin(), adjacent_indices[v1].end(), v0);
        if (it == adjacent_indices[v1].end()) {
            adjacent_indices[v1].push_back(v0);
        }
        it = std::find(adjacent_indices[v1].begin(), adjacent_indices[v1].end(), v2);
        if (it == adjacent_indices[v1].end()) {
            adjacent_indices[v1].push_back(v2);
        }
        it = std::find(adjacent_indices[v2].begin(), adjacent_indices[v2].end(), v0);
        if (it == adjacent_indices[v2].end()) {
            adjacent_indices[v2].push_back(v0);
        }
        it = std::find(adjacent_indices[v2].begin(), adjacent_indices[v2].end(), v1);
        if (it == adjacent_indices[v2].end()) {
            adjacent_indices[v2].push_back(v1);
        }
    }

    rotations.resize(V.rows(), Eigen::Matrix3d::Identity()); // Initialize all rotations to identity 
}

// For each vertex, compute the optimal rotation that best aligns the original and deformed edge vectors to preserve local rigidity.
void ArapDeformer::computeLocalStep(){
    for (int i = 0; i < V.rows(); ++i){
        Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero(); 
        // Loop through all neighbouring vertices
        for (int j = 0; j < adjacent_indices[i].size(); ++j){
            int neighbor_index = adjacent_indices[i][j];
            Eigen::Vector3d original_edge, deformed_edge;

            // Compute the covariance matrix for vertex i by summing over its neighbors
            original_edge = (V.row(i) - V.row(neighbor_index)).transpose();
            deformed_edge = (V_new.row(i) - V_new.row(neighbor_index)).transpose();
            double weight = L_cot.coeff(i, neighbor_index);

            covariance += weight * original_edge * deformed_edge.transpose();
        }
        // SVD Decomposition 
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d rotation = svd.matrixV() * svd.matrixU().transpose();

        // Ensure the rotation is a proper rotation (determinant = 1)
        if (rotation.determinant() < 0) {
            Eigen::Matrix3d matV = svd.matrixV();
            matV.col(2) *= -1; // Flip the sign of the last column
            rotation = matV * svd.matrixU().transpose();
        }
        rotations[i] = rotation;
    }
}


