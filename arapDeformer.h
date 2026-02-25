#ifndef ARAP_DEFORMER_H
#define ARAP_DEFORMER_H

#include <igl/opengl/glfw/Viewer.h>
#include <Eigen/SparseCholesky>
#include <iostream>

struct ArapDeformer {
    ArapDeformer(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, std::vector<int>& anchors);
    void populateAugmentedLaplacian(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, const double& anchorWeight);
    void populateTargetMatrix(const std::vector<Eigen::Vector3d>& target_positions, double anchorWeight);
    void solveLeastSquares();
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    Eigen::MatrixXd target;
    Eigen::MatrixXd delta;
    Eigen::SparseMatrix<double> L_aug;
    Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> solver;
    Eigen::MatrixXd V_new;
    std::vector<int> anchor_indices;

};

#endif