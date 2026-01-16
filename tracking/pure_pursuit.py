# tracking/pure_pursuit.py

import numpy as np
from config.parameters import (
    LOOKAHEAD_DISTANCE,
    PP_LINEAR_VEL,
    WHEEL_BASE
)

# ============================================================
# PURE PURSUIT CONTROLLER
# ============================================================

class PurePursuitController:
    """
    Pure Pursuit path tracking controller
    """

    def __init__(self, path):
        """
        path: list of (row, col) dari A*
        dikonversi ke (x, y)
        """
        self.path = [(col, row) for row, col in path]
        self.current_target_idx = 0

    def find_lookahead_point(self, x, y):
        """
        Cari titik path dengan jarak >= lookahead distance
        """
        for i in range(self.current_target_idx, len(self.path)):
            px, py = self.path[i]
            dist = np.hypot(px - x, py - y)

            if dist >= LOOKAHEAD_DISTANCE:
                self.current_target_idx = i
                return px, py

        return self.path[-1]

    def control(self, x, y, theta):
        """
        Hitung kontrol (v, omega)
        """
        target_x, target_y = self.find_lookahead_point(x, y)

        dx = target_x - x
        dy = target_y - y

        alpha = np.arctan2(dy, dx) - theta
        alpha = self.normalize_angle(alpha)

        Ld = np.hypot(dx, dy)
        if Ld < 1e-6:
            return 0.0, 0.0

        curvature = (2.0 * np.sin(alpha)) / Ld

        v = PP_LINEAR_VEL
        omega = curvature * v

        return v, omega

    @staticmethod
    def normalize_angle(angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi
