#include "arapDeformer.h"

#include <igl/cotmatrix.h>
#include <igl/parallel_for.h>
#include <igl/polar_svd.h>

ArapDeformer::ArapDeformer(const Eigen::MatrixXd& v, const Eigen::MatrixXi& f, std::vector<int>& anchors, std::vector<Eigen::Vector3d>& anchors_positions) :
V(v), F(f), anchor_indices(anchors), anchors_positions(anchors_positions){}

/**
 * Constructs the system matrix for the global step and pre-factors the solver.
 * Adds anchor weights directly to the diagonal of pinned vertices.
 * Pre-computes the Cholesky decomposition for fast back-substitution during the global solve.
 */
void ArapDeformer::populateAugmentedLaplacian(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, const double& anchorWeight){
    Eigen::SparseMatrix<double> L_system = -L_cot; // flip from negative semi definite to positive semi definite

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
 * Constructs the Right-Hand Side (b) for the global linear system.
 * The matrix size is strictly N x 3 (total vertices by xyz).
 * Accumulates the rotation-based target components from the local step using precomputed half-weighted edges.
 * Injects the soft-constraint target positions for anchor vertices by adding their weighted 3D positions directly to their respective rows.
 */
void ArapDeformer::populateTargetMatrix(const std::vector<Eigen::Vector3d>& target_positions, double anchorWeight){
    target.resize(V.rows(), 3); // Num rows = total vertices, cols = xyz

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

    for (int i = 0; i < anchor_indices.size(); ++i){
        int idx = anchor_indices[i];
        target.row(idx) += target_positions[i].transpose() * anchorWeight; // Apply anchor weight to the target position
    }
}

/**
 * Executes the Global Step of the ARAP algorithm.
 * Computes the optimal vertex positions by solving the pre-factored linear system.
 * This finds the coordinates that best match the optimal local rotations while 
 * being pulled towards the user-defined anchor positions.
 */
void ArapDeformer::solveLeastSquares(){
    // Solve system 
    V_new = solver.solve(target);

    if (solver.info() != Eigen::Success) {
        std::cerr << "Solving failed!" << std::endl;
        return;
    }
}

/**
 * Pre-computes static geometry data to optimize the iterative local-global solver.
 * Generates the base cotangent Laplacian matrix.
 * Computes the original differential coordinates (delta).
 * Builds an optimized adjacency list storing original edge vectors and their pre-multiplied cotangent weights to avoid redundant calculations per frame.
 */
void ArapDeformer::precomputeStaticData() {
    // Generate base cotangent matrix
    igl::cotmatrix(V, F, L_cot);
    
    // Calculate differential coordinates
    delta = L_cot * V;
    
    // Create adjacency list for each vertex
    precomputed_neighbors.resize(V.rows()); 
    rotations.resize(V.rows(), Eigen::Matrix3d::Identity()); // Initialize all rotations to identity 

    // Traverse efficiently through sparse matrix
    for (int c = 0; c < L_cot.outerSize(); ++c){
        for (Eigen::SparseMatrix<double>::InnerIterator it(L_cot, c); it; ++it){
            // Access data from cotangent matrix
            int currentVertex = it.row();
            int neighbouringVertex = it.col();
            double weight = it.value();

            if (currentVertex != neighbouringVertex) { // Ignore diagonal entries
                // Populate neighbour data 
                NeighborData data;
                data.index = neighbouringVertex;
                data.weight = weight;
                data.original_edge = (V.row(neighbouringVertex) - V.row(currentVertex)).transpose();
                data.weighted_edge = -data.weight * data.original_edge;
                data.half_weighted_edge = (data.weight / 2.0) * data.original_edge;
                // save the neighbour data for this vertex
                precomputed_neighbors[currentVertex].push_back(data);
            }
        }
    }
}

/**
 * Executes the Local Step of the ARAP algorithm.
 * For each vertex, finds the optimal 3D rotation matrix that best aligns its 
 * 1-ring neighborhood from the original rest shape to the current deformed shape.
 * Solved by constructing a covariance matrix and extracting the rotation via SVD.
 */void ArapDeformer::computeLocalStep(){
    // MULTITHREADING COOLNESS: Splits the loop across available CPU threads
    igl::parallel_for(V.rows(), [&](int i){
        Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero(); 
        
        // Loop through all neighbouring vertices
        for (int j = 0; j < precomputed_neighbors[i].size(); ++j){
            int neighbor_index = precomputed_neighbors[i][j].index;
            Eigen::Vector3d deformed_edge;

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


