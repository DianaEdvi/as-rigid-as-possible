#include "arapDeformer.h"

#include <igl/cotmatrix.h>
void ArapDeformer::buildLaplacian(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F){
    // Generate base cotangent matrix
    Eigen::SparseMatrix<double> L_cot;
    igl::cotmatrix(V, F, L_cot);

}


