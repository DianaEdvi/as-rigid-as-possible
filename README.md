# libigl As-Rigid-As-Possible project

A project showcasing ARAP deformation.

## Compile

Compile this project using the standard cmake routine:

    mkdir build
    cd build
    cmake ..
    cmake --build . --config Release

This should find and build the dependencies and create a `arap` binary.

## Run

From within the `build` directory just issue:

    ./arap ../models/"MODELNAME".obj

Replace MODELNAME with one of the four provided models:

`sphere.obj` contains **80** faces.

`lilium.obj` contains **6590** faces.

`circle.obj` contains **10240** faces.

`armadillo.obj` contains **86482** faces (fun bonus)

A glfw app should launch displaying a the selected model.

### Instructions

* **Drag + Left-click:** Rotate the view
* **Scroll:** Zoom in/out
* **SHIFT + Left-click:** Anchor vertices
* **CONTROL + Drag + Left-click:** Move anchors
* **R:** Reset the mesh

## Dependencies

The only dependencies are STL, Eigen, [libigl](http://libigl.github.io/libigl/) and the dependencies
of the `igl::opengl::glfw::Viewer` (OpenGL, glad and GLFW).

The CMake build system will automatically download libigl and its dependencies using
[CMake FetchContent](https://cmake.org/cmake/help/latest/module/FetchContent.html),
thus requiring no setup on your part.