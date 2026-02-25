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

void UIManager::updateAnchorsVector(){
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


bool UIManager::handle_mouse_down(int button, int modifier){
    // Convert to OpenGL coords
    float mouseY = viewer.core().viewport(3) - (float)viewer.current_mouse_y;

    // Shift + Left click: Select anchors 
    if (modifier & GLFW_MOD_SHIFT) { 
        // Find nearest vertex and add it to the anchors 
        raycast_to_vertex(viewer.current_mouse_x, mouseY);
        updateAnchorsVector();
        colorAnchors();

        is_dragging = true;
        return true; 
    } 

    // Control + Left click: Select vertex to drag 
    if (modifier & GLFW_MOD_CONTROL) {
        // Find nearest vertex
        int hit_vertex = raycast_to_vertex(viewer.current_mouse_x, mouseY);

        // Only allow dragging if the vertex is actually in our anchor list!
        auto it = std::find(anchor_indices.begin(), anchor_indices.end(), hit_vertex);
        if (it != anchor_indices.end()) {
            selected_vertex = hit_vertex;
            is_dragging = true; 
            return true;
        }
    }
    return false;
}

bool UIManager::handle_mouse_move(int mouse_x, int mouse_y) {
    float mouseY = viewer.core().viewport(3) - (float)mouse_y;
    if (is_dragging && selected_vertex != -1) {
        
        std::cout << "Dragging vertex: " << selected_vertex << std::endl;

        // 3. (Next step) Unproject the 2D mouse_x and mouse_y into 3D space 
        // to find the new target position for selected_vertex.


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


