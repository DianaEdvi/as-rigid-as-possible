#ifndef ARAP_DEFORMER_H
#define ARAP_DEFORMER_H

#include <igl/opengl/glfw/Viewer.h>
#include <iostream>

struct ArapDeformer {
    void populateAugmentedLaplacian(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, const std::vector<int>& anchorVertices, const double& anchorWeight);
    Eigen::MatrixXd delta;
    Eigen::SparseMatrix<double> L_aug;
};

#endif