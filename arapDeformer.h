#ifndef ARAP_DEFORMER_H
#define ARAP_DEFORMER_H

#include <igl/opengl/glfw/Viewer.h>
#include <Eigen/SparseCholesky>
#include <iostream>

struct ArapDeformer {
    ArapDeformer(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, std::vector<int>& anchors, std::vector<Eigen::Vector3d>& anchors_positions);
    void populateAugmentedLaplacian(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, const double& anchorWeight);
    void populateTargetMatrix(const std::vector<Eigen::Vector3d>& target_positions, double anchorWeight);
    void solveLeastSquares();
    void precomputeStaticData();
    void computeLocalStep();
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    Eigen::MatrixXd target;
    Eigen::MatrixXd delta;
    Eigen::SparseMatrix<double> L_cot;
    Eigen::SparseMatrix<double> L_aug;
    Eigen::SparseMatrix<double> L_aug_T;
    Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> solver;
    Eigen::MatrixXd V_new;
    Eigen::MatrixXd balanced_target;
    std::vector<int>& anchor_indices;
    std::vector<Eigen::Vector3d>& anchors_positions;
    std::vector<Eigen::Matrix3d> rotations;
    struct NeighborData {
        int index;
        double weight;
        Eigen::Vector3d original_edge;
    };
    std::vector<std::vector<NeighborData>> precomputed_neighbors;
};

#endif