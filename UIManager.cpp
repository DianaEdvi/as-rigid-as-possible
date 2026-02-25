#include "UIManager.h"
#include <igl/opengl/glfw/Viewer.h>

#include <algorithm>

UIManager::UIManager(igl::opengl::glfw::Viewer& v, Eigen::MatrixXd& V, Eigen::MatrixXi& F, std::vector<int>& anchors, bool needs_rebuild, std::vector<Eigen::Vector3d>& anchors_positions) :
viewer(v), V(V), F(F), anchor_indices(anchors), needs_rebuild(needs_rebuild), anchors_positions(anchors_positions){
    tree.init(V, F);
}

int UIManager::raycast_to_vertex(int mouse_x, int mouse_y){
    std::cout << "Raycast to vertex" << std::endl;
    Eigen::Vector3d nearPlane; 
    Eigen::Vector3d farPlane; 

    // Get the NDC's of the near and far planes 
    igl::unproject(Eigen::Vector3d(mouse_x, mouse_y, 0.0f), viewer.core().view, viewer.core().proj, viewer.core().viewport, nearPlane);
    igl::unproject(Eigen::Vector3d(mouse_x, mouse_y, 1.0f), viewer.core().view, viewer.core().proj, viewer.core().viewport, farPlane);
    Eigen::Vector3d dir = farPlane - nearPlane; 

    igl::Hit<double> hit; 
    bool is_hit = tree.intersect_ray(V, F, nearPlane, dir, hit); // use AABB tree to find triangle 

    if (is_hit){
        int id = hit.id;
        Eigen::Vector3d hitPoint = nearPlane + dir * hit.t;
        
        double min_dist = INFINITY;
        int closest_vertex = -1; // Keep track of the best one

        // Loop through every corner of the hit triangle 
        for (int i = 0; i < 3; i++){
            int v_index = F(id, i);
            Eigen::Vector3d v_pos(V(v_index, 0), V(v_index, 1), V(v_index, 2));
            double dist = (v_pos - hitPoint).norm();
            
            // If this is the closest one so far, save it!
            if (dist <= min_dist){
                min_dist = dist;
                closest_vertex = v_index;
            }
        }
        
        // Return the closest one 
        return closest_vertex; 
    }
    
    return -1; // no vertex found
}

void UIManager::colorAnchors(){
    std::cout << "Coloring anchors" << std::endl;
    if (anchor_indices.empty()) {
        viewer.data().clear_points();
        return; 
    }

    Eigen::MatrixXd P(anchor_indices.size(), 3); // Positions
    Eigen::MatrixXd C(anchor_indices.size(), 3); // Colors

    for (int i = 0; i < anchor_indices.size(); ++i){
        // Read directly from the target positions array (don't overwrite with V)
        P.row(i) = anchors_positions[i].transpose();

        // Highlight the currently dragged vertex in Green, and the rest in Red
        if (anchor_indices[i] == selected_vertex && is_dragging) {
            C.row(i) = Eigen::RowVector3d(0, 1, 0); // Green
        } else {
            C.row(i) = Eigen::RowVector3d(1, 0, 0); // Red
        }
    }

    viewer.data().point_size = 10;
    viewer.data().set_points(P, C); // Pass both the positions and the colors matrix
}

void UIManager::updateAnchorsVector(){
    std::cout << "Updating anchors vector" << std::endl;
    
    auto it = std::find(anchor_indices.begin(), anchor_indices.end(), selected_vertex);
    
    if (it == anchor_indices.end()){
        std::cout << "Adding new vertex" << std::endl;
        anchor_indices.push_back(selected_vertex);
        // Initialize the anchor's position exactly when it is created
        anchors_positions.push_back(Eigen::Vector3d(V(selected_vertex, 0), V(selected_vertex, 1), V(selected_vertex, 2)));
        needs_rebuild = true;
    }
    else {
        std::cout << "Removing vertex" << std::endl;
        // Find the index to keep the positions array perfectly synced with the indices array
        int idx = std::distance(anchor_indices.begin(), it);
        anchor_indices.erase(it);
        anchors_positions.erase(anchors_positions.begin() + idx); 
    }
}


bool UIManager::handle_mouse_down(int button, int modifier) {
    // Convert to OpenGL coords
    float mouseY = viewer.core().viewport(3) - (float)viewer.current_mouse_y;

    // Shift + Left click: Select/Deselect anchors 
    if (modifier & GLFW_MOD_SHIFT) { 
        int hit_vertex = raycast_to_vertex(viewer.current_mouse_x, mouseY);
        
        // Only do something if we actually hit the mesh
        if (hit_vertex != -1) {
            selected_vertex = hit_vertex;
            updateAnchorsVector(); // Toggle it on/off
            colorAnchors();
        }
        return true; 
    } 

    // Control + Left click: Select vertex to drag 
    if (modifier & GLFW_MOD_CONTROL) {
        int hit_vertex = raycast_to_vertex(viewer.current_mouse_x, mouseY);

        // Only allow dragging if we hit a vertex, AND that vertex is already an anchor!
        if (hit_vertex != -1) {
            auto it = std::find(anchor_indices.begin(), anchor_indices.end(), hit_vertex);
            if (it != anchor_indices.end()) {
                // It's a valid anchor! Let's drag it.
                selected_vertex = hit_vertex;
                is_dragging = true; 
                colorAnchors(); // Turn it green
            }
        }
        // Always return true if Control is held, so libigl doesn't hijack the camera
        return true;
    }
    
    return false;
}
bool UIManager::handle_mouse_move(int mouse_x, int mouse_y, int modifier) {
    float mouseY = viewer.core().viewport(3) - (float)mouse_y;

    // If we aren't holding Control, stop dragging immediately
    if (!(modifier & GLFW_MOD_CONTROL)) {
        if (is_dragging) {
            is_dragging = false;
            selected_vertex = -1;
            tree.init(V, F);
            colorAnchors();
        }
        return false;
    }

    if (is_dragging && selected_vertex != -1) {
        
        std::cout << "Dragging vertex: " << selected_vertex << std::endl;

        // 1. Use RowVector3d (1x3) so libigl knows this is exactly ONE vertex
        Eigen::RowVector3d win_coords; 

        // Project the 3D position
        Eigen::RowVector3d v_pos = V.row(selected_vertex);
        igl::project(v_pos, viewer.core().view, viewer.core().proj, viewer.core().viewport, win_coords);
        
        double z = win_coords(2);  
        
        // 2. Unproject using RowVector3d as well
        Eigen::RowVector3d newPos;
        Eigen::RowVector3d mouse_pos(mouse_x, mouseY, z);
        igl::unproject(mouse_pos, viewer.core().view, viewer.core().proj, viewer.core().viewport, newPos);

        auto it = std::find(anchor_indices.begin(), anchor_indices.end(), selected_vertex);
        if (it != anchor_indices.end()) {
            int anchor_idx = std::distance(anchor_indices.begin(), it);
            // Safely assign the row vector data back into your Vector3d array
            anchors_positions[anchor_idx] = Eigen::Vector3d(newPos(0), newPos(1), newPos(2)); 
        }

        colorAnchors();
        return true;
    }

    return false;
}

bool UIManager::handle_mouse_up(int button, int modifier){
    if (is_dragging) {
        is_dragging = false;
        std::cout << "Mouse released!" << std::endl;
        selected_vertex = -1;
        tree.init(V, F);
        colorAnchors();
        return true;
    }
        return false;

}


