import math

def wrap_angle(a):
    return (a + math.pi) % (2 * math.pi) - math.pi

def world_to_screen(x, y, ox, oy, scale, room_h):
    return int(ox + x * scale), int(oy + (room_h - y) * scale)
