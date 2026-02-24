#include <igl/opengl/glfw/Viewer.h>
#include <igl/readOBJ.h>
#include <igl/AABB.h>
#include <igl/Hit.h>
#include <iostream>
#include <Eigen/Core>

#include "arapDeformer.h"

int main(int argc, char *argv[])
{
  std::cout << "Hello" << std::endl;

  Eigen::MatrixXd V;
  Eigen::MatrixXi F;

  // Check if the user provided an argument
  if (argc < 2) {
      std::cerr << "Usage: " << argv[0] << " <path_to_obj_file>" << std::endl;
      return -1;
  }

  // Pass argv[1] (the first argument) instead of the hardcoded string
  if (!igl::readOBJ(argv[1], V, F)) {
      std::cerr << "Failed to load mesh from " << argv[1] << "!" << std::endl;
      return -1;
  }

  // Plot the mesh
  igl::opengl::glfw::Viewer viewer;
  viewer.data().set_mesh(V, F);
  viewer.data().set_face_based(true);
  
  igl::AABB<Eigen::MatrixXd, 3> tree;
  tree.init(V, F);

  // --- State Variables ---
  int selected_vertex = -1; // -1 means nothing is selected
  bool is_dragging = false;

  // --- Callbacks ---
viewer.callback_mouse_down = [&](igl::opengl::glfw::Viewer& viewer, int button, int modifier)->bool {
    if (modifier & GLFW_MOD_SHIFT) { // Example: Hold Shift to select/drag
        // TODO: Figure out which vertex was clicked using raycasting
        int mouseY = viewer.core().viewport(3) - (float)viewer.current_mouse_y;
        
        Eigen::Vector3d nearPlane; 
        Eigen::Vector3d farPlane; 
        igl::unproject(Eigen::Vector3d(viewer.current_mouse_x, mouseY, 0.0f), viewer.core().view, viewer.core().proj, viewer.core().viewport, nearPlane);
        igl::unproject(Eigen::Vector3d(viewer.current_mouse_x, mouseY, 1.0f), viewer.core().view, viewer.core().proj, viewer.core().viewport, farPlane);
        Eigen::Vector3d dir = farPlane - nearPlane; 

        igl::Hit<double> hit; 
        bool is_hit = tree.intersect_ray(V, F, nearPlane, dir,hit);

        if (is_hit){
            std::cout << "Face clicked!" << std::endl;
            int id = hit.id;
            Eigen::Vector3d hitPoint = nearPlane + dir * hit.t;
            double min_dist = INFINITY;

            // Loop through every corner
            for (int i = 0; i < 3; i++){
                int v_index = F(id, i);
                
                double dist = (V.row(v_index).transpose() - hitPoint).norm();
                if (dist <= min_dist){
                    min_dist = dist;
                    selected_vertex = v_index;
                }
            }

            Eigen::MatrixXd selected_pos = V.row(selected_vertex);

            viewer.data().point_size = 10; // Make it big enough to see
            viewer.data().set_points(selected_pos, Eigen::RowVector3d(1, 0, 0)); // color red
        }
        is_dragging = true;
        return true; // Return true to tell the viewer we handled the event
    }
    return false;
    };

  viewer.callback_mouse_move = [&](igl::opengl::glfw::Viewer& viewer, int mouse_x, int mouse_y)->bool {
      if (is_dragging && selected_vertex != -1) {
          // TODO: Move the selected vertex to the new mouse position
          std::cout << "Dragging vertex: " << selected_vertex << std::endl;

          // viewer.data().set_vertices(V); // You'll call this to update the visual
          return true;
      }
      return false;
      };

  viewer.callback_mouse_up = [&](igl::opengl::glfw::Viewer& viewer, int button, int modifier)->bool {
      if (is_dragging) {
          is_dragging = false;
          std::cout << "Mouse released!" << std::endl;
          return true;
      }
      return false;
      };

  viewer.launch();
}
