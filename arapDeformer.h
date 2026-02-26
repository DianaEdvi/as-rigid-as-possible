#ifndef ARAP_DEFORMER_H
#define ARAP_DEFORMER_H

#include <igl/opengl/glfw/Viewer.h>
#include <Eigen/SparseCholesky>
#include <iostream>

// Define a new Row-Major dynamic matrix type (Cache locality optimization for sparse solvers)
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXdRow;
struct ArapDeformer {
    ArapDeformer(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, std::vector<int>& anchors, std::vector<Eigen::Vector3d>& anchors_positions);
    void populateAugmentedLaplacian(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, const double& anchorWeight);
    void populateTargetMatrix(const std::vector<Eigen::Vector3d>& target_positions, double anchorWeight);
    void solveLeastSquares();
    void precomputeStaticData();
    void computeLocalStep();
    MatrixXdRow V;
    Eigen::MatrixXi F;
    MatrixXdRow target;
    MatrixXdRow delta;
    Eigen::SparseMatrix<double> L_cot;
    Eigen::SparseMatrix<double> L_aug;
    Eigen::SparseMatrix<double> L_aug_T;
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
    MatrixXdRow V_new;
    MatrixXdRow balanced_target;
    std::vector<int>& anchor_indices;
    std::vector<Eigen::Vector3d>& anchors_positions;
    std::vector<Eigen::Matrix3d> rotations;
    struct NeighborData {
        int index;
        double weight;
        Eigen::Vector3d original_edge;
        Eigen::Vector3d weighted_edge;      // For computeLocalStep
        Eigen::Vector3d half_weighted_edge; // For populateTargetMatrix
    };
    std::vector<std::vector<NeighborData>> precomputed_neighbors;
};

#endif