#ifndef ARAP_DEFORMER_H
#define ARAP_DEFORMER_H

#include <igl/opengl/glfw/Viewer.h>
#include <iostream>

class ArapDeformer {
    void buildLaplacian(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F);
};

#endif