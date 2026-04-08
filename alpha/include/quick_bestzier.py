import math
import numpy as np

class QuickBestzier:
    def __init__(self, control_points):
        #needs at minimum 2 contorl points

        self.control_points = np.array(control_points, dtype=float)
        self.n = len(control_points) - 1
        self.binom = [math.comb(self.n, i) for i in range(self.n + 1)]

    def bernstein(self, i, t):
        return self.binom[i] * (t ** i) * ((1 - t) ** (self.n - i))

    def bernstein_derivative(self, i, t):
        if self.n == 0:
            return 0
        return self.n * (
            (math.comb(self.n - 1, i - 1) * t ** (i - 1) * (1 - t) ** (self.n - i) if i > 0 else 0)
            - (math.comb(self.n - 1, i) * t ** i * (1 - t) ** (self.n - i - 1) if i < self.n else 0)
        )

    def position(self, t):
        p = np.zeros(3)
        for i in range(self.n + 1):
            p += self.bernstein(i, t) * self.control_points[i]
        return p

    def velocity(self, t):
        v = np.zeros(3)
        for i in range(self.n + 1):
            v += self.bernstein_derivative(i, t) * self.control_points[i]
        return v

    def sample_curve(self, num_points=200):
        ts = np.linspace(0, 1, num_points)
        pts = np.array([self.position(t) for t in ts])
        return ts, pts

    def closest_t(self, current_pos, num_samples=200):
        ts, pts = self.sample_curve(num_samples)
        #currently only 2d
        dists = np.linalg.norm(pts[;2] - current_pos[:2], axis=1)
        idx = np.argmin(dists)
        return ts[idx]

    def get_setpoint(self, current_pos, lookahead=0.02, speed=1.0):
        t_closest = self.closest_t(current_pos)
        t_target = min(t_closest + lookahead, 1.0)

        pos_sp = self.position(t_target)
        vel_dir = self.velocity(t_target)

        norm = np.linalg.norm(vel_dir)
        if norm > 1e-6:
            vel_sp = (vel_dir / norm) * speed
        else:
            vel_sp = np.zeros_like(vel_dir)

        return pos_sp, vel_sp, t_target
