#include <igl/opengl/glfw/Viewer.h>
#include <igl/readOBJ.h>
#include <iostream>
#include <Eigen/Core>
#include <igl/opengl/glfw/imgui/ImGuiPlugin.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>

#include "arapDeformer.h"
#include "UIManager.h"

#include <thread>

int main(int argc, char *argv[])
{
    unsigned int hardware_threads = std::thread::hardware_concurrency();
    std::cout << "Hardware threads available: " << hardware_threads << std::endl;
    
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

    // Add menu plugin
    igl::opengl::glfw::imgui::ImGuiPlugin plugin;
    viewer.plugins.push_back(&plugin);
    igl::opengl::glfw::imgui::ImGuiMenu menu;
    plugin.widgets.push_back(&menu);

    menu.callback_draw_viewer_window = [&]()
    {
        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(10, 10), ImGuiCond_FirstUseEver);

        ImGui::Text("As-Rigid-As-Possible Mesh Deformation");
        ImGui::Text("Number of faces: %d", F.rows());
        ImGui::Text("Framerate: %.1f FPS", ImGui::GetIO().Framerate);
        ImGui::Separator();
        ImGui::Text("Instructions:");
        ImGui::BulletText("Drag left-click to rotate the view");
        ImGui::BulletText("Scroll to zoom in/out");
        ImGui::BulletText("SHIFT + Left-click: Anchor vertices");
        ImGui::BulletText("CONTROL + drag + Left-click: Move anchors");
        ImGui::BulletText("R: to reset the mesh");
    };

    std::vector<int> anchors;
    std::vector<Eigen::Vector3d> anchors_positions;
    bool needs_rebuild = false;
    bool needs_solve = false;
    int arapIterations = 2;

    ArapDeformer deformer(V, F, anchors, anchors_positions);
    deformer.V_new = V;
    UIManager uiManager(viewer, deformer.V_new, F, anchors, needs_rebuild, anchors_positions);

    deformer.precomputeStaticData();

    viewer.callback_mouse_down = [&](igl::opengl::glfw::Viewer& v, int button, int mod) -> bool {
        bool handled = uiManager.handle_mouse_down(button, mod);
        if (needs_rebuild){
            deformer.populateAugmentedLaplacian(V, F, 10000.0);
            needs_rebuild = false; 
        }
        return handled;
    };

    viewer.callback_mouse_move = [&](igl::opengl::glfw::Viewer& v, int x, int y) -> bool {
        int modifier = 0;
        // Query the GLFW window directly for the Control key state
        if (glfwGetKey(v.window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS || 
            glfwGetKey(v.window, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS) {
            modifier |= GLFW_MOD_CONTROL;
        }
        bool handled = uiManager.handle_mouse_move(x, y, modifier);
        if (handled){
            needs_solve = true; 
        }
        return handled;
    };

    viewer.callback_pre_draw = [&](igl::opengl::glfw::Viewer& v) -> bool {
        // Run the iterations once per visual frame
        if (needs_solve) {
            // ARAP iterations to converge to the optimal solution
            for (int i = 0; i < arapIterations; ++i){
                deformer.computeLocalStep();
                deformer.populateTargetMatrix(anchors_positions, 10000.0);
                deformer.solveLeastSquares();
            }
            viewer.data().set_vertices(deformer.V_new);
            
            needs_solve = false; 
        }
        return false; // Return false so libigl continues with the normal draw cycle
    };

    viewer.callback_mouse_up = [&](igl::opengl::glfw::Viewer& v, int button, int mod) -> bool {
        return uiManager.handle_mouse_up(button, mod);
    };

    viewer.callback_key_pressed = [&](igl::opengl::glfw::Viewer& v, unsigned int key, int modifiers) -> bool {
        // Check if the 'r' or 'R' key was pressed
        if (key == 'r' || key == 'R') {            
            // 1. Reset the vertices
            deformer.V_new = V;
            viewer.data().set_vertices(deformer.V_new);
            
            // 2. Clear out all the anchors so it doesn't snap back!
            anchors.clear();
            anchors_positions.clear();
            viewer.data().clear_points(); // Erases the red/green dots
            needs_rebuild = true;

            uiManager.rebuild_tree();
            
            return true; // Tells libigl we handled this key press
        }
        return false;
    };



    // Set default mesh view settings
    viewer.data().set_mesh(V, F);
    viewer.data().set_face_based(true);
    
    // Launch window
    viewer.launch(); 
}

