#include "arapDeformer.h"

#include <igl/cotmatrix.h>
#include <igl/parallel_for.h>
#include <igl/polar_svd.h>

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
    // Store new data in triplets. More efficient when we rebuild the sparse matrix 
    std::vector<Eigen::Triplet<double>> triplets;

    // Reserve space
    triplets.reserve(L_cot.nonZeros() + anchor_indices.size());

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

    L_aug_T = L_aug.transpose();

    // Normal Equations
    Eigen::SparseMatrix<double> prefactorized_L_aug = L_aug_T * L_aug;

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
    target.resize(L_aug.rows(), 3); // Num rows = total vertices + anchors, cols = xyz

    // Populate the first N rows with the original curvature (delta)
    //original edge vectors, rotated by the matrices in rotations, and weighted by the cotangent weights.

    igl::parallel_for(delta.rows(), [&](int i){
        Eigen::Vector3d weighted_delta = Eigen::Vector3d::Zero();
        for (int j = 0; j < precomputed_neighbors[i].size(); ++j){
            int neighbor_index = precomputed_neighbors[i][j].index;
            double weight = precomputed_neighbors[i][j].weight;
            Eigen::Vector3d original_edge = precomputed_neighbors[i][j].original_edge;
            Eigen::Matrix3d rotation_target = rotations[i];
            Eigen::Matrix3d rotation_neighbor = rotations[neighbor_index];
            weighted_delta += (weight/2) * (rotation_target + rotation_neighbor) * original_edge;
        }
        target.row(i) = weighted_delta.transpose();
    });


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
    // Directly evaluate the multiplication into the pre-allocated member variable
    balanced_target.noalias() = L_aug_T * target;

    // Solve system 
    V_new = solver.solve(balanced_target);

    if (solver.info() != Eigen::Success) {
        std::cerr << "Solving failed!" << std::endl;
        return;
    }

}

void ArapDeformer::precomputeStaticData() {
    // Generate base cotangent matrix
    igl::cotmatrix(V, F, L_cot);
    
    delta = L_cot * V;
    
    // Create adjacency list for each vertex
    precomputed_neighbors.resize(V.rows()); 
    rotations.resize(V.rows(), Eigen::Matrix3d::Identity()); // Initialize all rotations to identity 

    // Traverse efficiently through sparse matrix
    for (int c = 0; c < L_cot.outerSize(); ++c){
        for (Eigen::SparseMatrix<double>::InnerIterator it(L_cot, c); it; ++it){
            int row = it.row();
            int col = it.col();
            double weight = it.value();

            if (row != col) { // Ignore diagonal entries
                NeighborData data;
                data.index = col;
                data.weight = weight;
                data.original_edge = (V.row(col) - V.row(row)).transpose();
                precomputed_neighbors[row].push_back(data);
            }
        }
    }
    

}

// For each vertex, compute the optimal rotation that best aligns the original and deformed edge vectors to preserve local rigidity.
void ArapDeformer::computeLocalStep(){
    
    igl::parallel_for(V.rows(), [&](int i){
        Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero(); 
        // Loop through all neighbouring vertices
        for (int j = 0; j < precomputed_neighbors[i].size(); ++j){
            int neighbor_index = precomputed_neighbors[i][j].index;
            Eigen::Vector3d original_edge, deformed_edge;

            // Compute the covariance matrix for vertex i by summing over its neighbors
            original_edge = precomputed_neighbors[i][j].original_edge;
            deformed_edge = (V_new.row(i) - V_new.row(neighbor_index)).transpose();
            double weight = -precomputed_neighbors[i][j].weight;

            covariance += weight * original_edge * deformed_edge.transpose();
        }
        // SVD Decomposition 
        Eigen::Matrix3d rotation, T;
        igl::polar_svd(covariance, rotation, T);
        rotations[i] = rotation;
    });
}


