#include "arapDeformer.h"

#include <igl/cotmatrix.h>
void ArapDeformer::populateAugmentedLaplacian(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, const std::vector<int>& anchorVertices, const double& anchorWeight){
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

    int currentRow = V.rows(); // begin establishing anchor data after l_cot

    // Traverse through all the vertices we want to anchor
    for (int i = 0; i < anchorVertices.size(); ++i){
        // Add a new row for each anchor. Provide the location and weight
        triplets.push_back(Eigen::Triplet<double>(currentRow, anchorVertices[i], anchorWeight));
        currentRow++;
    }

    L_aug.resize(V.rows() + anchorVertices.size(), V.rows()); // Num rows, num cols. 
    // Don't forget num cols represents the number of vertices, not xyz. We are in Laplacian after all
    L_aug.setFromTriplets(triplets.begin(), triplets.end());
}


