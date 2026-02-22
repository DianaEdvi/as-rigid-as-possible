#include <igl/opengl/glfw/Viewer.h>
#include <igl/readOBJ.h>
#include <iostream>

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


  // --- State Variables ---
  int selected_vertex = -1; // -1 means nothing is selected
  bool is_dragging = false;

  // --- Callbacks ---
  viewer.callback_mouse_down = [&](igl::opengl::glfw::Viewer& viewer, int button, int modifier)->bool {
      if (modifier & GLFW_MOD_SHIFT) { // Example: Hold Shift to select/drag
          // TODO: Figure out which vertex was clicked using raycasting
          std::cout << "Mouse clicked! Need to find the closest vertex." << std::endl;
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
