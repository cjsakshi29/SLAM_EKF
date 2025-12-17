import numpy as np
import math
from utils import wrap_angle
from config import Q, R

class EKFSLAM:
    def __init__(self):
        self.mu = np.array([1.0, 1.0, 0.5])
        self.Sigma = np.eye(3)

        self.landmark_map = {}
        self.path = []

    def predict(self, u, dt):
        v, w = u
        x, y, th = self.mu[:3]

        self.mu[0] += v * math.cos(th) * dt
        self.mu[1] += v * math.sin(th) * dt
        self.mu[2] += w * dt

        self.Sigma[:3, :3] += Q

        self.path.append((self.mu[0], self.mu[1]))

    def update(self, lm_id, z):
        r, b = z
        lx, ly = self.landmark_map[lm_id]

        dx = lx - self.mu[0]
        dy = ly - self.mu[1]
        q = dx**2 + dy**2

        z_hat = np.array([
            math.sqrt(q),
            wrap_angle(math.atan2(dy, dx) - self.mu[2])
        ])

        H = np.array([
            [-dx / math.sqrt(q), -dy / math.sqrt(q), 0],
            [ dy / q,            -dx / q,           -1]
        ])

        S = H @ self.Sigma[:3, :3] @ H.T + R
        K = self.Sigma[:3, :3] @ H.T @ np.linalg.inv(S)

        # State update
        self.mu[:3] += K @ (z - z_hat)
        self.Sigma[:3, :3] = (np.eye(3) - K @ H) @ self.Sigma[:3, :3]

    def add_landmark(self, lm_id, pos):
        self.landmark_map[lm_id] = pos
