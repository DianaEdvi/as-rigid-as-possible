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
    std::vector<Eigen::Vector3d> anchors_positions;
    bool needs_rebuild = false;

    ArapDeformer deformer(V, F, anchors, anchors_positions);
    UIManager uiManager(viewer, V, F, anchors, needs_rebuild, anchors_positions);

    viewer.callback_mouse_down = [&](igl::opengl::glfw::Viewer& v, int button, int mod) -> bool {
        bool handled = uiManager.handle_mouse_down(button, mod);
        deformer.populateAugmentedLaplacian(V, F, 1.0);
        needs_rebuild = false; 
        return handled;
    };

    viewer.callback_mouse_move = [&](igl::opengl::glfw::Viewer& v, int x, int y) -> bool {
        bool handled = uiManager.handle_mouse_move(x, y);
        if (handled){
            deformer.populateTargetMatrix(anchors_positions, 1.0);
            deformer.solveLeastSquares();
            V = deformer.V_new;
            viewer.data().set_vertices(V);
        }
        return handled;
    };

    viewer.callback_mouse_up = [&](igl::opengl::glfw::Viewer& v, int button, int mod) -> bool {
        return uiManager.handle_mouse_up(button, mod);
    };



    // Set default mesh view settings
    viewer.data().set_mesh(V, F);
    viewer.data().set_face_based(true);
    
    // Launch window
    viewer.launch(); 
}
