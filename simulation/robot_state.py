# simulation/robot_state.py

import numpy as np

class RobotState:
    """
    Menyimpan state robot dan histori simulasi
    """

    def __init__(self, x=0.0, y=0.0, theta=0.0):
        # State saat ini
        self.x = x
        self.y = y
        self.theta = theta

        # History posisi
        self.x_history = [x]
        self.y_history = [y]
        self.theta_history = [theta]

        # History error
        self.cross_track_error = []   # pastikan ini ada
        self.heading_error = []       # pastikan ini ada

        # History waktu
        self.time = [0.0]

    def update(self, x, y, theta, t):
        """
        Update state dan simpan ke history
        """
        self.x = x
        self.y = y
        self.theta = theta

        self.x_history.append(x)
        self.y_history.append(y)
        self.theta_history.append(theta)
        self.time.append(t)

    def log_error(self, cte, heading_err):
        """
        Simpan error lintasan
        """
        self.cross_track_error.append(cte)
        self.heading_error.append(heading_err)
