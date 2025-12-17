import math
from utils import wrap_angle

class Bug2:
    def __init__(self):
        self.mode = "GO_TO_GOAL"

    def compute(self, robot_state, goal, obstacle):
        x, y, theta = robot_state
        gx, gy = goal

        angle_to_goal = math.atan2(gy - y, gx - x)
        heading_error = wrap_angle(angle_to_goal - theta)

        if obstacle:
            self.mode = "WALL_FOLLOW"
        elif self.mode == "WALL_FOLLOW":
            self.mode = "GO_TO_GOAL"

        if self.mode == "GO_TO_GOAL":
            v = 0.4
            w = 1.5 * heading_error
        else:
            v = 0.2
            w = 0.8

        return v, w
