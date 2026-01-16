# tracking/kinematics.py

import numpy as np
from config.parameters import (
    DT,
    MAX_LINEAR_VEL,
    MAX_ANGULAR_VEL
)

# ============================================================
# DIFFERENTIAL DRIVE KINEMATICS
# ============================================================

class DifferentialDriveRobot:
    """
    Model kinematika robot differential drive
    State: x, y, theta
    Control: v (linear), omega (angular)
    """

    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta

    def step(self, v, omega):
        """
        Update state robot berdasarkan input kontrol

        Parameters:
        - v     : linear velocity (m/s)
        - omega : angular velocity (rad/s)
        """

        # Clamp kecepatan
        v = np.clip(v, -MAX_LINEAR_VEL, MAX_LINEAR_VEL)
        omega = np.clip(omega, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL)

        # Update posisi
        self.x += v * np.cos(self.theta) * DT
        self.y += v * np.sin(self.theta) * DT
        self.theta += omega * DT

        # Normalisasi sudut (-pi, pi)
        self.theta = self.normalize_angle(self.theta)

    @staticmethod
    def normalize_angle(angle):
        """Normalisasi sudut ke rentang [-pi, pi]"""
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def state(self):
        """Return state robot"""
        return self.x, self.y, self.theta
