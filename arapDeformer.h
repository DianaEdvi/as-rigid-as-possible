#ifndef ARAP_DEFORMER_H
#define ARAP_DEFORMER_H

#include <igl/opengl/glfw/Viewer.h>
#include <Eigen/SparseCholesky>
#include <iostream>

struct ArapDeformer {
    void populateAugmentedLaplacian(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, const std::vector<int>& anchor_indices, const double& anchorWeight);
    void populateTargetMatrix(const std::vector<int>& anchor_indices, const std::vector<Eigen::Vector3d>& target_positions, double constraint_weight);
    void solveLeastSquares();
    Eigen::MatrixXd target;
    Eigen::MatrixXd delta;
    Eigen::SparseMatrix<double> L_aug;
    Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> solver;
    Eigen::MatrixXd V_new;
};

#endif