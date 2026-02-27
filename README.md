# As-Rigid-As-Possible

This project is an interactive 3D mesh deformation tool that implements the ARAP (As-Rigid-As-Possible) algorithm. It allows a user to select specific "anchor" vertices and drag them one at a time to deform the mesh. The ARAP algorithm calculates how the rest of the mesh (the non-anchored parts) should move in order to preserve the local geometric features (rigidity) of the original shape as much as possible.

## Mesh requirements

This project currently only supports single, continuous meshes. The expected ARAP behaviour cannot be guaranteed with models that are discontinuous or that contain multiple meshes.

## Compile

Compile this project using the standard cmake routine:

    mkdir build
    cd build
    cmake ..
    make

Alternatively:

    mkdir build
    cd build
    cmake ..
    cmake --build .

This should find and build the dependencies and create a `arap` binary.

## Run

From within the `build` directory just run the executable and pass in the path to the model object you would like to display. Some example models have been included in this project for your convenience.

    ./arap ../models/cube.obj

A glfw app should launch displaying the passed model.

## Core technologies

- **libigl**: Used for geometry processing, setting up the 3D viewer, and UI overlay (ImGui).

- **Eigen**: Used heavily for linear algebra, specifically for its sparse matrix representations and Cholesky solvers.

- **GLFW & OpenGL**: Used under the hood by libigl to handle windowing and rendering.

## Implementation

Check out my [implementation](./docs/IMPLEMENTATION.md) doc for a full breakdown of the project