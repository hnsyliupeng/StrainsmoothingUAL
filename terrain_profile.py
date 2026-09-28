"""Shared analytic terrain height field for land, a broad lake basin, and robot contact.

The lake depression is intentionally broad and shallow enough to read as a real
pond from the overview camera. The water mesh solves this exact height field,
so its shoreline is smooth and the scattering mask can reject submerged ground.
"""
import math

LAKE_CENTER = (3.7, 3.8)
# Keep a broad, clearly visible lake, but leave the moving robot a generous,
# walkable dry bank. The previous 5.2 m depression steepened the ground along
# the robot's chase path and made the URDF-limited ankle chains unreachable.
LAKE_RADIUS = 4.5
LAKE_BASIN_DEPTH = 1.9
LAKE_BASIN_EXPONENT = 0.80
LAKE_LEVEL = -0.38


def height(x, y):
    # Keep broad undulation below the lake waterline outside the basin: this
    # guarantees a closed shore instead of disconnected water pockets.
    rolling = .21 * math.sin(x * .32) * math.cos(y * .27) + .13 * math.sin(y * .61 + x * .14)
    hills = (1.30 * math.exp(-((x - 7.5)**2 + (y + 7.5)**2) / 30)
             + .95 * math.exp(-((x + 8.5)**2 + (y - 4.5)**2) / 26))
    radius = math.hypot(x - LAKE_CENTER[0], y - LAKE_CENTER[1])
    basin = max(0.0, 1.0 - radius / LAKE_RADIUS) ** LAKE_BASIN_EXPONENT
    return rolling + hills - LAKE_BASIN_DEPTH * basin
