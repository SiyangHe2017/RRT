import math


def node_collision_circle_free(x1, y1, x2, y2, x0, y0, r):
    k = (y1 - y2) / (x1 - x2)
    d0 = abs(k * x0 - y0 + y1 - k * x1) / math.sqrt(k * k + 1)
    if d0 <= r:
        return False
    else:
        return True


print(node_collision_circle_free(0, 1, 3, -4, 2, 2, 1))

print(node_collision_circle_free(0, 1, 2, 4, 2, 2, 1))