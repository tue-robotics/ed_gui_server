# At the moment, only vertices are used. The triangles are described by 9 elements in the
# vertices array (x1, y1, z1, x2, y2, z2, x3, y3, z3). The triangles array is NOT used.

# (original method (NOT USED): List of vertices: each 3 elements describe (x, y, z) )
float32[] vertices

# (origin method (NOT USED) List of indices: each 3 indices describe one triangle, indexing the vertices given above)
uint32[] triangles
