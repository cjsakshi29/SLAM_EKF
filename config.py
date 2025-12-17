import math
import numpy as np

ROOM_W, ROOM_H = 10.0, 8.0
SCALE = 60

SCREEN_W, SCREEN_H = 1200, 700

DT = 0.1

GOALS = [(2,2), (8,4), (5,7)]
GOAL_NAMES = ["G1", "G2", "G3"]

LANDMARKS = {
    0: (4,3),   # Pillar
    1: (8,5),   # Table corner
    2: (2,7)    # Cabinet edge
}

FOV = math.radians(90)
MAX_RANGE = 4.0
SAFE_DIST = 0.8

Q = np.diag([0.02, 0.02, 0.01])
R = np.diag([0.1, math.radians(5)])
