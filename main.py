import pygame
import sys
import math

from config import *
from robot import Robot
from ekf import EKFSLAM
from navigation import Bug2
from sensor import sense_landmarks, obstacle_ahead
from utils import wrap_angle, world_to_screen

pygame.init()
screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
pygame.display.set_caption("EKF-SLAM with Bug-2 Navigation")
clock = pygame.time.Clock()
font = pygame.font.SysFont(None, 20)

robot = Robot(3.0, 4.0, 0.0)

ekf = EKFSLAM()

navigator = Bug2()

goal_index = 0
mission_complete = False
simulation_started = False

GRID_RES = 0.1 
GRID_W = int(ROOM_W / GRID_RES)
GRID_H = int(ROOM_H / GRID_RES)

explored = [[0 for _ in range(GRID_H)] for _ in range(GRID_W)]

def update_explored(robot):
    for i in range(GRID_W):
        for j in range(GRID_H):
            wx = i * GRID_RES
            wy = j * GRID_RES

            dx = wx - robot.x
            dy = wy - robot.y
            r = math.hypot(dx, dy)

            if r > MAX_RANGE:
                continue

            angle = wrap_angle(math.atan2(dy, dx) - robot.theta)
            if abs(angle) <= FOV / 2:
                explored[i][j] = 1

def draw_world_grid(surface, ox, oy):
    for x in range(int(ROOM_W) + 1):
        p1 = world_to_screen(x, 0, ox, oy, SCALE, ROOM_H)
        p2 = world_to_screen(x, ROOM_H, ox, oy, SCALE, ROOM_H)
        pygame.draw.line(surface, (60, 60, 60), p1, p2, 1)

    for y in range(int(ROOM_H) + 1):
        p1 = world_to_screen(0, y, ox, oy, SCALE, ROOM_H)
        p2 = world_to_screen(ROOM_W, y, ox, oy, SCALE, ROOM_H)
        pygame.draw.line(surface, (60, 60, 60), p1, p2, 1)

while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()

    if not mission_complete:
        current_goal = GOALS[goal_index]

        obstacle = obstacle_ahead(robot.state(), LANDMARKS, SAFE_DIST)
        v, w = navigator.compute(robot.state(), current_goal, obstacle)

        v *= 0.8
        w *= 0.8

        robot.step(v, w, DT)
        ekf.predict((v, w), DT)

        gx, gy = current_goal
        if math.hypot(robot.x - gx, robot.y - gy) < 0.3:
            if goal_index < len(GOALS) - 1:
                goal_index += 1
            else:
                mission_complete = True
                v = 0.0
                w = 0.0

    update_explored(robot)

    measurements = sense_landmarks(robot.state(), LANDMARKS)
    for lm_id, r, b in measurements:
        if lm_id not in ekf.landmark_map:
            ekf.add_landmark(lm_id, LANDMARKS[lm_id])
        ekf.update(lm_id, (r, b))

    screen.fill((20, 20, 20))
    ox, oy = 20, 20

    pygame.draw.rect(
        screen, (255, 255, 255),
        (ox, oy, ROOM_W * SCALE, ROOM_H * SCALE), 2
    )
    
    for i in range(GRID_W):
        for j in range(GRID_H):
            wx = i * GRID_RES
            wy = j * GRID_RES
            color = (30, 30, 30) if explored[i][j] else (120, 120, 120)

            px, py = world_to_screen(wx, wy, ox, oy, SCALE, ROOM_H)
            pygame.draw.rect(
                screen, color,
                (px, py, GRID_RES * SCALE + 1, GRID_RES * SCALE + 1)
            )

    draw_world_grid(screen, ox, oy)

    pygame.draw.circle(
        screen, (180, 180, 180),
        world_to_screen(4, 3, ox, oy, SCALE, ROOM_H),
        int(0.15 * SCALE)
    )

    tx, ty = world_to_screen(8, 5, ox, oy, SCALE, ROOM_H)
    pygame.draw.line(screen, (180, 180, 180), (tx, ty), (tx + 30, ty), 3)
    pygame.draw.line(screen, (180, 180, 180), (tx, ty), (tx, ty - 30), 3)

    cx, cy = world_to_screen(2, 7, ox, oy, SCALE, ROOM_H)
    pygame.draw.line(screen, (180, 180, 180), (cx - 20, cy), (cx + 20, cy), 3)


    for i, (gx, gy) in enumerate(GOALS):
        color = (0, 255, 0) if i < goal_index else (200, 200, 200)
        pygame.draw.circle(
            screen, color,
            world_to_screen(gx, gy, ox, oy, SCALE, ROOM_H),
            8, 2
        )

    for p in robot.path:
        pygame.draw.circle(
            screen, (0, 150, 255),
            world_to_screen(p[0], p[1], ox, oy, SCALE, ROOM_H), 2
        )

    for p in ekf.path:
        pygame.draw.circle(
            screen, (255, 0, 0),
            world_to_screen(p[0], p[1], ox, oy, SCALE, ROOM_H), 2
        )

    pygame.draw.circle(
        screen, (0, 255, 0),
        world_to_screen(robot.x, robot.y, ox, oy, SCALE, ROOM_H),
        int(0.20 * SCALE)
    )

    pygame.draw.circle(
        screen, (255, 0, 0),
        world_to_screen(ekf.mu[0], ekf.mu[1], ox, oy, SCALE, ROOM_H),
        int(0.20 * SCALE)
    )
    
    rx, ry = world_to_screen(robot.x, robot.y, ox, oy, SCALE, ROOM_H)

    left = (robot.x + MAX_RANGE * math.cos(robot.theta + FOV / 2),
            robot.y + MAX_RANGE * math.sin(robot.theta + FOV / 2))
    right = (robot.x + MAX_RANGE * math.cos(robot.theta - FOV / 2),
             robot.y + MAX_RANGE * math.sin(robot.theta - FOV / 2))

    pygame.draw.polygon(
        screen, (255, 255, 0),
        [(rx, ry),
         world_to_screen(left[0], left[1], ox, oy, SCALE, ROOM_H),
         world_to_screen(right[0], right[1], ox, oy, SCALE, ROOM_H)], 1
    )

    legend_x = ox
    legend_y = oy + ROOM_H * SCALE + 10
    legend_w = 420
    legend_h = 180

    pygame.draw.rect(
        screen, (255, 255, 255),
        (legend_x, legend_y, legend_w, legend_h), 2
    )


    legend_items = [
        ("True Robot Path", (0, 150, 255), "line"),
        ("EKF Estimated Path", (255, 0, 0), "line"),
        ("True Robot", (0, 255, 0), "circle"),
        ("EKF Robot", (255, 0, 0), "circle"),
        ("Landmarks", (255, 255, 0), "circle"),
        ("Goal (Reached)", (0, 255, 0), "hollow"),
        ("Goal (Not Reached)", (180, 180, 180), "hollow")
    ]

    for i, (text, color, style) in enumerate(legend_items):
        y_offset = legend_y + 15 + i * 22

        if style == "line":
            pygame.draw.line(
                screen, color,
                (legend_x + 10, y_offset + 8),
                (legend_x + 30, y_offset + 8),
                3
            )

        elif style == "circle":
            pygame.draw.circle(
                screen, color,
                (legend_x + 20, y_offset + 8),
                6
            )

        elif style == "hollow":
            pygame.draw.circle(
                screen, color,
                (legend_x + 20, y_offset + 8),
                6,
                2
            )

        screen.blit(
            font.render(text, True, (255, 255, 255)),
            (legend_x + 40, y_offset)
        )

    map_x, map_y = 760, 20
    map_w, map_h = 400, 300

    pygame.draw.rect(
        screen, (255, 255, 255),
        (map_x, map_y, map_w, map_h), 2
    )

    for p in ekf.path:
        pygame.draw.circle(
            screen, (255, 100, 100),
            world_to_screen(
                p[0], p[1],
                map_x + 20, map_y + 20,
                SCALE * 0.5, ROOM_H
            ),
            2
        )
        
    pygame.draw.circle(
        screen, (255, 0, 0),
        world_to_screen(
            ekf.mu[0], ekf.mu[1],
            map_x + 20, map_y + 20,
            SCALE * 0.5, ROOM_H
        ),
        6
    )

    for lx, ly in ekf.landmark_map.values():
        pygame.draw.circle(
            screen, (255, 255, 0),
            world_to_screen(
                lx, ly,
                map_x + 20, map_y + 20,
                SCALE * 0.5, ROOM_H
            ),
            5
        )

    info_x, info_y = 760, 350
    info_w, info_h = 400, 300

    pygame.draw.rect(
        screen, (255, 255, 255),
        (info_x, info_y, info_w, info_h), 2
    )

    info_lines = [
    "NUMERICAL READINGS",
    "",
    f"True x : {robot.x:.2f}  "
    f"True y : {robot.y:.2f}  "
    f"True θ : {robot.theta:.2f}",
    f"v      : {v:.2f}",
    f"ω      : {w:.2f}",
    "",
    f"EKF x  : {ekf.mu[0]:.2f}  "
    f"EKF y  : {ekf.mu[1]:.2f}  "
    f"EKF θ  : {ekf.mu[2]:.2f}",
    "",
    f"Error  : {math.hypot(robot.x - ekf.mu[0], robot.y - ekf.mu[1]):.2f}",
    f"Goal   : {GOAL_NAMES[goal_index] if goal_index < len(GOAL_NAMES) else 'MISSION COMPLETE'}",

    f"Mode   : {navigator.mode}"
]


    for i, txt in enumerate(info_lines):
        screen.blit(
            font.render(txt, True, (255, 255, 255)),
            (info_x + 10, info_y + 10 + i * 25)
        )

    pygame.display.flip()
    clock.tick(30)
    
