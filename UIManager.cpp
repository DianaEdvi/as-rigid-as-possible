#include "UIManager.h"
#include <igl/opengl/glfw/Viewer.h>

#include <algorithm>

UIManager::UIManager(igl::opengl::glfw::Viewer& v, Eigen::MatrixXd& V, Eigen::MatrixXi& F, std::vector<int>& anchors, bool needs_rebuild) :
viewer(v), V(V), F(F), anchor_indices(anchors), needs_rebuild(needs_rebuild) {
    tree.init(V, F);
}

int UIManager::raycast_to_vertex(int mouse_x, int mouse_y){
    Eigen::Vector3d nearPlane; 
    Eigen::Vector3d farPlane; 

    // Get the NDC's of the near and far planes 
    igl::unproject(Eigen::Vector3d(mouse_x, mouse_y, 0.0f), viewer.core().view, viewer.core().proj, viewer.core().viewport, nearPlane);
    igl::unproject(Eigen::Vector3d(mouse_x, mouse_y, 1.0f), viewer.core().view, viewer.core().proj, viewer.core().viewport, farPlane);
    Eigen::Vector3d dir = farPlane - nearPlane; 

    igl::Hit<double> hit; 
    bool is_hit = tree.intersect_ray(V, F, nearPlane, dir,hit); // use AABB tree to find triangle 

    if (is_hit){
        // The id of the triangle in the faces matrix
        int id = hit.id;
        
        Eigen::Vector3d hitPoint = nearPlane + dir * hit.t;
        double min_dist = INFINITY;

        // Loop through every corner of the hit triangle 
        for (int i = 0; i < 3; i++){
            int v_index = F(id, i);
            
            // Calculate the distance from the vertex to the hitpoint 
            double dist = (V.row(v_index).transpose() - hitPoint).norm();
            // Find closest vertex 
            if (dist <= min_dist){
                min_dist = dist;
                selected_vertex = v_index;
            }
        }

        // Check if selected vertex is already in the vector
        auto it = std::find(anchor_indices.begin(), anchor_indices.end(), selected_vertex);
        if (it == anchor_indices.end()){
            anchor_indices.push_back(selected_vertex);
            needs_rebuild = true;
        }
        else {
            anchor_indices.erase(it);
        }
    }
    return selected_vertex;
}

// Color selected vertices red 
void UIManager::colorAnchors(){
    // Clear the points if no anchors are selected
    if (anchor_indices.empty()) {
        viewer.data().clear_points();
        return; 
    }

    Eigen::MatrixXd anchorPositions(anchor_indices.size(), 3);

    // Loop through every vertex and color it to red
    for (int i = 0; i < anchor_indices.size(); ++i){
        anchorPositions.row(i) = V.row(anchor_indices[i]);

        viewer.data().point_size = 10; // Make it big enough to see
        viewer.data().set_points(anchorPositions, Eigen::RowVector3d(1, 0, 0));
    }
}


bool UIManager::handle_mouse_down(int button, int modifier){
    if (modifier & GLFW_MOD_SHIFT) { 
        // Convert to OpenGL coords
        int mouseY = viewer.core().viewport(3) - (float)viewer.current_mouse_y;

        // Find nearest vertes
        raycast_to_vertex(viewer.current_mouse_x, mouseY);
        colorAnchors();
        
        is_dragging = true;
        return true; 
    } 
    return false;
}

bool UIManager::handle_mouse_move(int mouse_x, int mouse_y){
    if (is_dragging && selected_vertex != -1) {
            // TODO: Move the selected vertex to the new mouse position
            std::cout << "Dragging vertex: " << selected_vertex << std::endl;

            // viewer.data().set_vertices(V); // You'll call this to update the visual
            return true;
        }
            return false;
}

bool UIManager::handle_mouse_up(int button, int modifier){
    if (is_dragging) {
        is_dragging = false;
        std::cout << "Mouse released!" << std::endl;
        return true;
    }
        return false;

}

void UIManager::launch() {
    // Tell the viewer who to call when mouse events happen
    viewer.callback_mouse_down = [this](igl::opengl::glfw::Viewer& v, int button, int mod) -> bool {
        return this->handle_mouse_down(button, mod);
    };

    viewer.callback_mouse_move = [this](igl::opengl::glfw::Viewer& v, int x, int y) -> bool {
        return this->handle_mouse_move(x, y);
    };

    viewer.callback_mouse_up = [this](igl::opengl::glfw::Viewer& v, int button, int mod) -> bool {
        return this->handle_mouse_up(button, mod);
    };

    // Set default mesh view settings
    viewer.data().set_mesh(V, F);
    viewer.data().set_face_based(true);
    
    // Launch window
    viewer.launch(); 
}


