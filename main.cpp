#include <igl/opengl/glfw/Viewer.h>
#include <igl/readOBJ.h>
#include <iostream>
#include <Eigen/Core>

#include "arapDeformer.h"
#include "UIManager.h"
int main(int argc, char *argv[])
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    
    // Check if the user provided an argument
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <path_to_obj_file>" << std::endl;
        return -1;
    }
    
    // Pass argv[1]
    if (!igl::readOBJ(argv[1], V, F)) {
        std::cerr << "Failed to load mesh from " << argv[1] << "!" << std::endl;
        return -1;
    }
    
    igl::opengl::glfw::Viewer viewer;
    std::vector<int> anchors;
    bool needs_rebuild = false;

    ArapDeformer deformer(V, F, anchors);
    UIManager uiManager(viewer, V, F, anchors, needs_rebuild);

    uiManager.launch();
}
