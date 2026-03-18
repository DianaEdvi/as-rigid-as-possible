#include "arapDeformer.h"

#include <igl/cotmatrix.h>
#include <igl/parallel_for.h>
#include <igl/polar_svd.h>

ArapDeformer::ArapDeformer(const Eigen::MatrixXd& v, const Eigen::MatrixXi& f, std::vector<int>& anchors, std::vector<Eigen::Vector3d>& anchors_positions) :
V(v), F(f), anchor_indices(anchors), anchors_positions(anchors_positions){}

/**
 * Constructs the system matrix for the global step and pre-factors the solver.
 * Initializes the system using the symmetric negative cotangent Laplacian.
 * Adds anchor weights directly to the diagonal of pinned vertices.
 * Pre-computes the Cholesky decomposition of this N x N symmetric positive-definite 
 * matrix for fast back-substitution during the global solve.
 */
void ArapDeformer::populateAugmentedLaplacian(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, const double& anchorWeight){
    Eigen::SparseMatrix<double> L_system = -L_cot;

    // Add anchor weights directly to the diagonal of the existing vertices
    for (int i = 0; i < anchor_indices.size(); ++i) {
        int idx = anchor_indices[i];
        L_system.coeffRef(idx, idx) += anchorWeight;
    }

    // Compute Cholesky decomposition 
    solver.compute(L_system);
    
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
    target.resize(V.rows(), 3); // Num rows = total vertices + anchors, cols = xyz

    // Populate the first N rows with the original curvature (delta)
    //original edge vectors, rotated by the matrices in rotations, and weighted by the cotangent weights.

    igl::parallel_for(delta.rows(), [&](int i){
        Eigen::Vector3d weighted_delta = Eigen::Vector3d::Zero();
        for (int j = 0; j < precomputed_neighbors[i].size(); ++j){
            int neighbor_index = precomputed_neighbors[i][j].index;
            Eigen::Matrix3d rotation_target = rotations[i];
            Eigen::Matrix3d rotation_neighbor = rotations[neighbor_index];
            weighted_delta += (rotation_target + rotation_neighbor) * precomputed_neighbors[i][j].half_weighted_edge;
        }
        target.row(i) = -weighted_delta.transpose();
    });

    // Populate the N + C rows with the anchor vertex positions 
    for (int i = 0; i < anchor_indices.size(); ++i){
        int idx = anchor_indices[i];
        target.row(idx) += target_positions[i].transpose() * anchorWeight; // Apply anchor weight to the target position
    }
}

/**
 * Computes the optimal vertex positions by solving the linear system by using the pre-factored Cholesky decomposition.
 * This effectively finds the vertex positions that minimize the shape deformation while preserving anchor positions.
 */
void ArapDeformer::solveLeastSquares(){
    // Solve system 
    V_new = solver.solve(target);

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
                data.weighted_edge = -data.weight * data.original_edge;
                data.half_weighted_edge = (data.weight / 2.0) * data.original_edge;
                precomputed_neighbors[row].push_back(data);
            }
        }
    }
}

// For each vertex, compute the optimal rotation that best aligns the original and deformed edge vectors to preserve local rigidity.
void ArapDeformer::computeLocalStep(){
    // MULTITHREADING COOLNESS: Splits the loop across available CPU threads
    igl::parallel_for(V.rows(), [&](int i){
        Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero(); 
        
        // Loop through all neighbouring vertices
        for (int j = 0; j < precomputed_neighbors[i].size(); ++j){
            int neighbor_index = precomputed_neighbors[i][j].index;
            Eigen::Vector3d original_edge, deformed_edge;

            // Compute the covariance matrix for vertex i by summing over its neighbors
            deformed_edge = (V_new.row(i) - V_new.row(neighbor_index)).transpose();
            covariance += deformed_edge * precomputed_neighbors[i][j].weighted_edge.transpose();
        }
        
        // SVD Decomposition 
        Eigen::Matrix3d rotation, T;
        igl::polar_svd(covariance, rotation, T);
        rotations[i] = rotation;
    });
}


