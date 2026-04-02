import numpy as np
from numba import njit

num_obj = 4

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

def main():
    dist = np.zeros(1, dtype=np.float64)
    dist_shm = shared_memory.SharedMemory(name="dist", create=True, size=dist.nbytes)

    shm_corners = shared_memory.SharedMemory(name="obstacle_corners")
    shm_vic = shared_memory.SharedMemory(name="vicon_state")

    corners = np.ndarray((num_obj, 4, 3), dtype=np.float64, buffer=shm_corners.buf)
    vic_state = np.ndarray((num_obj, 6), dtype=np.float64, buffer=shm_vic.buf)
    dist_shared = np.ndarray(dist.shape, dtype=dist.dtype, buffer=dist_shm.buf)

    resource_tracker.unregister(shm_corners._name, "shared_memory")
    resource_tracker.unregister(shm_vic._name, "shared_memory")
    resource_tracker.unregister(dist_shm._name, "shared_memory")


    min_dist = 1.0
    while True:
        triangles, obj_ids = quads_to_triangles(corners)    
        for i in triangles:
            edges = [(0,1), (1,2), (2,0)]
            for a, b in edges:
                dist = _point_segment_dist(P, i[a], i[b])
                if dist < min_dist:
                    min_dist = dist

        dist_shared[:] = min_dist
    

if __name__ == "__main__":
    main()
