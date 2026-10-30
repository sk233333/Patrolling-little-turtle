import math

CANVAS_MIN = 0.0
CANVAS_MAX = 11.088


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, low, high):
    return max(low, min(high, value))


def distance(p1, p2):
    return math.hypot(p2.x - p1.x, p2.y - p1.y)


def remaining_distance(pose, waypoints, index):
    total = math.hypot(waypoints[index].x - pose.x,
                       waypoints[index].y - pose.y)
    for j in range(index, len(waypoints) - 1):
        total += math.hypot(waypoints[j + 1].x - waypoints[j].x,
                            waypoints[j + 1].y - waypoints[j].y)
    return total


def validate_waypoints(waypoints, ys=None):
    if ys is None:
        pts = list(waypoints)
        if not pts:
            raise ValueError('waypoints is empty')
        coords = [(wp.x, wp.y) for wp in pts]
    else:
        xs = list(waypoints)
        ys = list(ys)
        if len(xs) != len(ys):
            raise ValueError(
                f'x/y length mismatch: {len(xs)} vs {len(ys)}')
        if not xs:
            raise ValueError('waypoints is empty')
        coords = list(zip(xs, ys))

    for i, (x, y) in enumerate(coords):
        if not (CANVAS_MIN <= x <= CANVAS_MAX):
            raise ValueError(
                f'waypoint {i} x={x} out of '
                f'[{CANVAS_MIN}, {CANVAS_MAX}]')
        if not (CANVAS_MIN <= y <= CANVAS_MAX):
            raise ValueError(
                f'waypoint {i} y={y} out of '
                f'[{CANVAS_MIN}, {CANVAS_MAX}]')
    return True
