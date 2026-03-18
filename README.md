# As-Rigid-As-Possible

This project is an interactive 3D mesh deformation tool that implements the ARAP (As-Rigid-As-Possible) algorithm. It allows a user to select specific "anchor" vertices and drag them one at a time to deform the mesh. The ARAP algorithm calculates how the rest of the mesh (the non-anchored parts) should move in order to preserve the local geometric features (rigidity) of the original shape as much as possible.

## Mesh requirements

This project currently only supports single, continuous meshes. The expected ARAP behaviour cannot be guaranteed with models that are discontinuous or that contain multiple meshes.

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

<!-- ## Implementation

Check out my [implementation](./docs/IMPLEMENTATION.md) doc for a full breakdown of the project
The CMake build system will automatically download libigl and its dependencies using
[CMake FetchContent](https://cmake.org/cmake/help/latest/module/FetchContent.html),
thus requiring no setup on your part. -->
