"""Shared terrain height field for editable mesh, lake, and robot ground contact."""
import math

LAKE_CENTER = (3.7, 3.8)
LAKE_RADIUS = 3.2
LAKE_LEVEL = -0.38


def height(x, y):
    rolling = .42 * math.sin(x * .32) * math.cos(y * .27) + .26 * math.sin(y * .61 + x * .14)
    hills = (1.30 * math.exp(-((x - 7.5)**2 + (y + 7.5)**2) / 30)
             + .95 * math.exp(-((x + 8.5)**2 + (y - 4.5)**2) / 26))
    shore = max(0., 1. - math.hypot(x - LAKE_CENTER[0], y - LAKE_CENTER[1]) / LAKE_RADIUS)
    return rolling + hills - 1.62 * shore**1.6
