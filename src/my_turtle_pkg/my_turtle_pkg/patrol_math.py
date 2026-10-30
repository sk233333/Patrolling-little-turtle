import math

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
