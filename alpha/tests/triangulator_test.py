import numpy as np
from numba import njit

EPS = 1e-8 

# calculate the closest distance between line ab and point p
@njit
def _point_segment_dist(p, a, b):
    # p, a, b are 1D arrays of length 3
    ab = b - a
    t = ((p - a) @ ab) / (np.dot(ab, ab) + EPS)
    # use np.minimum / np.maximum instead of min/max
    t = np.minimum(np.maximum(t, 0.0), 1.0)
    proj = a + t * ab
    diff = p - proj
    return np.sqrt(diff[0]**2 + diff[1]**2 + diff[2]**2)

TRI_INDICES = np.array([[0,1,2], [0,2,3]], dtype=np.int32)

# transforms rectangles into triangles
@njit
def quads_to_triangles(quads):
    N = quads.shape[0]
    triangles = np.empty((N*2,3,3), dtype=quads.dtype)
    obj_ids = np.empty(N*2, dtype=np.int32)
    
    for i in range(N):
        for t in range(2): 
            tri_idx = i*2 + t
            for v in range(3): 
                for c in range(3): 
                    triangles[tri_idx,v,c] = quads[i, TRI_INDICES[t,v], c]
            obj_ids[tri_idx] = i 
    return triangles, obj_ids

# tis is the example
N = 2500
while True:
    quads = np.random.rand(N, 4, 3)

    triangles, obj_ids = quads_to_triangles(quads)

    P = np.array([0.5, 0.5, 0.5])

    min_dist = 5.0
    for tri in triangles:
        edges = [(0,1),(1,2),(2,0)]
        for a,b in edges:
            dist = _point_segment_dist(P, tri[a], tri[b])
            if dist < min_dist:
                min_dist = dist

    print(min_dist)
