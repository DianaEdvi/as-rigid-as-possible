#ifndef UI_MANAGER_H
#define UI_MANAGER_H

#include <igl/opengl/glfw/Viewer.h>
#include <igl/AABB.h>
#include <igl/Hit.h>
#include <Eigen/SparseCholesky>

class UIManager {
    public:
        UIManager(igl::opengl::glfw::Viewer& v, Eigen::MatrixXd& V, Eigen::MatrixXi& F, std::vector<int>& anchors, bool needs_rebuild, std::vector<Eigen::Vector3d>& anchors_positions);
        bool& needs_rebuild;
        bool handle_mouse_down(int button, int modifier);
        bool handle_mouse_move(int mouse_x, int mouse_y, int modifier);
        bool handle_mouse_up(int button, int modifier);
    private:
        int selected_vertex = -1;
        bool is_dragging = false;
        std::vector<int>& anchor_indices;
        std::vector<Eigen::Vector3d>& anchors_positions;
        igl::AABB<Eigen::MatrixXd, 3> tree;

        // References to mesh and viewer
        igl::opengl::glfw::Viewer& viewer;
        Eigen::MatrixXd& V;
        Eigen::MatrixXi& F;

        int raycast_to_vertex(int mouse_x, int mouse_y);    
        void colorAnchors();
        void updateAnchorsVector();
};


#endif
