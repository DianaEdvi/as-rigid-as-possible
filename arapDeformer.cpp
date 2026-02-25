#include "arapDeformer.h"

#include <igl/cotmatrix.h>


ArapDeformer::ArapDeformer(const Eigen::MatrixXd& v, const Eigen::MatrixXi& f, std::vector<int>& anchors) :
V(v), F(f), anchor_indices(anchors){}

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
    Eigen::SparseMatrix<double> L_cot;
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
    target.topRows(delta.rows()) = delta;

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




