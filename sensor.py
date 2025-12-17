import math
from config import FOV, MAX_RANGE
from utils import wrap_angle

def sense_landmarks(robot_state, landmarks):
    x, y, theta = robot_state
    measurements = []

    for lm_id, (lx, ly) in landmarks.items():
        dx, dy = lx - x, ly - y
        r = math.hypot(dx, dy)
        bearing = wrap_angle(math.atan2(dy, dx) - theta)

        if r <= MAX_RANGE and abs(bearing) <= FOV / 2:
            measurements.append((lm_id, r, bearing))

    return measurements


def obstacle_ahead(robot_state, landmarks, safe_dist):
    """
    Camera-based obstacle detection with proper geometric clearance.
    Prevents robot from overlapping obstacles (pillar / table / cabinet).
    """
    x, y, theta = robot_state

    # Physical sizes (meters)
    OBSTACLE_RADIUS = 0.15    # pillar approx radius
    ROBOT_RADIUS = 0.20      # robot footprint radius

    for lx, ly in landmarks.values():
        dx = lx - x
        dy = ly - y

        r = math.hypot(dx, dy)
        bearing = wrap_angle(math.atan2(dy, dx) - theta)

        # ✅ FINAL FIX:
        # Obstacle is considered blocking if:
        # 1. Inside camera FOV
        # 2. Distance < safe + obstacle + robot radius
        if abs(bearing) < FOV / 2 and r < (safe_dist + OBSTACLE_RADIUS + ROBOT_RADIUS):
            return True

    return False

