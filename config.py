import math
import numpy as np

# World
ROOM_W, ROOM_H = 10.0, 8.0
SCALE = 60

# Screen
SCREEN_W, SCREEN_H = 1200, 700

# Timing
DT = 0.1

# Goals
GOALS = [(2,2), (8,4), (5,7)]
GOAL_NAMES = ["G1", "G2", "G3"]

# Landmarks (known IDs, unknown positions to EKF)
LANDMARKS = {
    0: (4,3),   # Pillar
    1: (8,5),   # Table corner
    2: (2,7)    # Cabinet edge
}

# Camera
FOV = math.radians(90)
MAX_RANGE = 4.0
SAFE_DIST = 0.8

# EKF noise
Q = np.diag([0.02, 0.02, 0.01])           # process noise
R = np.diag([0.1, math.radians(5)])       # measurement noise
