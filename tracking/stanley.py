# tracking/stanley.py

import numpy as np
from config.parameters import STANLEY_K, STANLEY_K_HEADING, STANLEY_LINEAR_VEL, MAX_ANGULAR_VEL

# ============================================================
# STANLEY CONTROLLER
# ============================================================

class StanleyController:
    """
    Stanley path tracking controller untuk Differential Drive Robot.
    """

    def __init__(self, path):
        """
        path: list of (row, col) dari A* -> dikonversi ke (x, y)
        """
        self.path = [(col, row) for row, col in path]  # (x, y)
        self.last_nearest_idx = 0

    def find_nearest_point(self, x, y):
        """
        Cari titik path terdekat dengan posisi robot dan cross-track error
        """
        min_dist = float('inf')
        nearest_idx = self.last_nearest_idx
        nearest_proj_error = 0.0

        # Mulai pencarian dari last_nearest_idx untuk efisiensi
        for i in range(self.last_nearest_idx, len(self.path) - 1):
            x1, y1 = self.path[i]
            x2, y2 = self.path[i + 1]

            # Vektor segmen
            dx = x2 - x1
            dy = y2 - y1

            if dx == 0 and dy == 0:
                continue

            # Proyeksi robot ke segmen path
            t = ((x - x1) * dx + (y - y1) * dy) / (dx**2 + dy**2)
            t = np.clip(t, 0.0, 1.0)
            x_proj = x1 + t * dx
            y_proj = y1 + t * dy

            dist = np.hypot(x - x_proj, y - y_proj)
            if dist < min_dist:
                min_dist = dist
                nearest_idx = i
                # Signed cross-track error
                cross = (x - x1) * dy - (y - y1) * dx
                nearest_proj_error = np.sign(cross) * dist

        self.last_nearest_idx = nearest_idx
        return nearest_idx, nearest_proj_error

    def control(self, x, y, theta):
        """
        Hitung kontrol (v, omega)
        """
        idx, cte = self.find_nearest_point(x, y)

        # Heading path dari titik terdekat ke titik berikutnya
        if idx < len(self.path) - 1:
            x_next, y_next = self.path[idx + 1]
        else:
            x_next, y_next = self.path[idx]

        x_curr, y_curr = self.path[idx]
        path_heading = np.arctan2(y_next - y_curr, x_next - x_curr)

        # Heading error
        heading_error = self.normalize_angle(path_heading - theta)

        # Stanley control law
        v = STANLEY_LINEAR_VEL
        omega = heading_error + np.arctan2(STANLEY_K * cte, v)
        omega = np.clip(omega, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL)

        return v, omega, cte, heading_error

    @staticmethod
    def normalize_angle(angle):
        """
        Normalize angle ke [-pi, pi]
        """
        return (angle + np.pi) % (2 * np.pi) - np.pi
