
# ArapDeformer

V is a matrix of vertex positions (N x 3), N vertices with position xyz.
F is the matrix of face indices (M x 3), M faces of 3 indices each. The indices referred to in F are indexed into the V matrix.

## What is the Laplace-Beltrami Operator?

In calculus, the Laplace operator measures how much the value at a specific point differs from the average value of its immediate surroundings. The Laplace-Beltrami operator is simply the generalization of this concept applied to curved 2D surfaces (such as a 3D mesh) instead of flat grids. It is represented as an N x N sparse matrix, where N is the number of vertices.

The diagonal entries contain the sum of the weights of all edges connected to a vertex.

The off-diagonal entries contain the negative weight of the edge connecting vertex $i$ and vertex $j$ (or zero if they are not connected).

![laplace-beltrami](./media/laplace-beltrami.png)

*Note that this image represents a normalized graph Laplacian, in which the cotangent weights are normalized to sum up to 1. They do not depend on the angle between the edges, as our code does*

We need the laplace-beltrami operator to calculate the **differential coordinates** of a mesh, otherwise known as the shape of the mesh. It allows the user to know the position of every vertex in relation to its neighbours, regardless of where the mesh is in the world.

## Precomputing Static Data

Not everything needs to be recalculated several times per frame. This function calculates all the data that remains persistent throughout the duration of the program.

First, we calcualte the Laplace-Beltrami operator using a built in libgl function.

    igl::cotmatrix(V, F, L_cot);

`L_cot` is the laplace-beltrami operator. It represents the original configuration of the mesh. Since the goal of ARAP is to try to maintain the original shape as much as possible, we need to store this data for future calculations. It will be used to calculate the optimal rotations of vertices, and for solving the global step of the ARAP.

Next we need to calculate the differential coordinates of the mesh, known as delta. They represent the shape of the mesh.

    delta = L_cot * V;

`delta` is an N x 3 matrix (N x N * N x 3 = N x 3). Each N represents a vertex.

Next, we simply allocate some memory for our `precomputed_neighbours` matrix and our `rotations` matrix.

    precomputed_neighbors.resize(V.rows());
    rotations.resize(V.rows(),Eigen::Matrix3d::Identity());

`precomputed_neighbours` is a 2D dynamic array. The outer array is of size N, with index `i` representing vertex `i`. The inner arrays depend on how many neighbours that particular vertex has. If vertex 0 is connected to 6 other vertices, `precomputed_neighbors[0]` will have a size of 6. Each element inside of each inner array are composed of `NeighbourData`.

`NeighbourData` contains the following data:

- `index`: An int (the ID of the neighboring vertex. It is used to look up a neighbouring vertex position from `V_new` and its rotation matrix from `rotations`).
- `weight`: A double (the cotangent weight $w_{ij}$). This represents the "stiffness" of an edge connecting two vertices. These weights are derived from the angles of adjacent triangles. This ensures that no matter how the mesh is triangulated (what shape and what density the triangles are), the stiffness of the mesh will be the same throughout.
- `original_edge`: A 3D vector ($3 \times 1$ vector of $v_j - v_i$) that represents the direction of this vector in the original, not deformed mesh.
- `weighted_edge`: The original edge multiplied by its negative cotangent weight. ($-w_{ij} \cdot e_{ij}$). Will be used to calculate the covariance matrix during the local step.
- `half_weighted_edge`: The original edge vector multiplied by half of its cotangent weight. ($\frac{w_{ij}}{2} \cdot e_{ij}$). This is required to build the target Right-Hand Side vector ($b$) during the Global Step

`rotations` is an array of N 3 x 3 matrices.

Next, we iterate through the cotangent matrix:

    for (int c = 0; c < L_cot.outerSize(); ++c)
        for (Eigen::SparseMatrix<double>::InnerIterator it(L_cot, c); it; ++it)
    
This is a standard way to access a sparse matrix with Eigen. In Eigen, sparse matrices are stored in column-major order. So first, we loop through the columns, and then we access the non-zero rows within that column.

So, now we have a way to access every non-zero entry in the cotangent matrix. Now it is time to grab the relevant data. Refer to the Laplace-Beltrami image if you don't understand why the currentVertex is represented by the row and the neighbouring is the col (although the matrix is symmetric so it actually doesn't matter which is which).

    int currentVertex = it.row();
    int neighbouringVertex = it.col();
    double weight = it.value();

We ignore the diagonal entries:

    if (currentVertex != neighbouringVertex)

Finally, we populate a NeighbourData object with the relevant information and push it into our `precomputed_neighbours` vector.

    NeighborData data;
    data.index = neighbouringVertex;
    data.weight = weight;
    data.original_edge = (V.row(neighbouringVertex) - V.row(currentVertex)).transpose();
    data.weighted_edge = -data.weight * data.original_edge;
    data.half_weighted_edge = (data.weight / 2.0) * data.original_edge;
    precomputed_neighbors[currentVertex].push_back(data);

And thats it! We have calculates all precomputations. Now that we have this, we can move on to the rest of ARAP.
