import math
import numpy as np

class QuickBestzier:
    def __init__(self, control_points):
        self.control_points = np.array(control_points, dtype=float)
        self.n = len(control_points) - 1
        self.binom = np.array([math.comb(self.n, i) for i in range(self.n + 1)], dtype=float)
        self.t_progress = 0.0

    def bernstein(self, i, t):
        return self.binom[i] * (t ** i) * ((1 - t) ** (self.n - i))

    def bernstein_derivative(self, i, t):
        if self.n == 0:
            return 0.0
        term1 = math.comb(self.n - 1, i - 1) * t ** (i - 1) * (1 - t) ** (self.n - i) if i > 0 else 0.0
        term2 = math.comb(self.n - 1, i) * t ** i * (1 - t) ** (self.n - i - 1) if i < self.n else 0.0
        return self.n * (term1 - term2)

    def position(self, t):
        t = np.asarray(t)
        B = np.array([self.bernstein(i, t) for i in range(self.n + 1)])
        return B.T @ self.control_points

    def velocity(self, t):
        t = np.asarray(t)
        Bd = np.array([self.bernstein_derivative(i, t) for i in range(self.n + 1)])
        return Bd.T @ self.control_points

    def closest_t(self, current_pos, coarse_samples=20, refinement_iters=3, refine_factor=5):
        current_pos = np.asarray(current_pos[:2])  # XY only

        ts = np.linspace(0.0, 1.0, coarse_samples)
        pts = self.position(ts)[:, :2]
        idx = np.argmin(np.linalg.norm(pts - current_pos, axis=1))
        t_best = ts[idx]

        for _ in range(refinement_iters):
            delta = 1.0 / (coarse_samples * refine_factor)
            t_lower = max(t_best - delta, 0.0)
            t_upper = min(t_best + delta, 1.0)
            ts_refine = np.linspace(t_lower, t_upper, coarse_samples)
            pts_refine = self.position(ts_refine)[:, :2]
            idx_refine = np.argmin(np.linalg.norm(pts_refine - current_pos, axis=1))
            t_best = ts_refine[idx_refine]

        print(t_best)

        return t_best

    def get_setpoint(self, current_pos, lookahead=0.02, speed=1.0):
        t_closest = self.closest_t(current_pos)
        t_closest = max(t_closest, self.t_progress) 

        t_target = min(t_closest + lookahead, 1.0)
        self.t_progress = t_target

        pos_sp = self.position(t_target)
        vel_dir = self.velocity(t_target)

        norm = np.linalg.norm(vel_dir)
        vel_sp = (vel_dir / norm) * speed if norm > 1e-6 else np.zeros_like(vel_dir)

        return pos_sp, vel_sp, t_target
