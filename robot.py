import math

class Robot:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
        self.path = []

    def step(self, v, w, dt):
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += w * dt
        self.path.append((self.x, self.y))

    def state(self):
        return self.x, self.y, self.theta
