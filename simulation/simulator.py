# simulation/simulator.py

import numpy as np

from simulation.robot_state import RobotState
from tracking.kinematics import DifferentialDriveRobot
from tracking.pure_pursuit import PurePursuitController
from tracking.stanley import StanleyController
from config.parameters import SIMULATION_PARAMS


# ============================================================
# SIMULATOR
# ============================================================

class Simulator:
    """
    Simulator utama untuk path tracking (Differential Drive Robot)
    """

    def __init__(self, path, controller_type="pure_pursuit"):
        self.path = np.array(path)
        self.controller_type = controller_type

        # ===============================
        # Parameter simulasi
        # ===============================
        self.dt = SIMULATION_PARAMS["dt"]
        self.max_time = SIMULATION_PARAMS["max_time"]
        self.target_speed = SIMULATION_PARAMS["target_speed"]

        # ===============================
        # Inisialisasi robot (start dari path)
        # ===============================
        row0, col0 = self.path[0]

        self.state = RobotState(
            x=col0,
            y=row0,
            theta=0.0
        )

        self.robot = DifferentialDriveRobot(
            x=col0,
            y=row0,
            theta=0.0
        )

        # ===============================
        # Inisialisasi controller (BUAT SEKALI)
        # ===============================
        if self.controller_type == "pure_pursuit":
            self.controller = PurePursuitController(self.path)

        elif self.controller_type == "stanley":
            self.controller = StanleyController(self.path)

        else:
            raise ValueError("Controller tidak dikenali")

    # ============================================================
    # RUN SIMULATION
    # ============================================================
    def run(self):
        """
        Jalankan simulasi path tracking
        """
        t = 0.0

        while t <= self.max_time:

            # ===============================
            # Controller
            # ===============================
            if self.controller_type == "pure_pursuit":
                v, omega = self.controller.control(
                    self.robot.x,
                    self.robot.y,
                    self.robot.theta
                )

                # Cross-track error
                target_idx = self.controller.current_target_idx
                tx, ty = self.path[target_idx]
                cte = np.hypot(tx - self.robot.x, ty - self.robot.y)

                # Heading error
                dx = tx - self.robot.x
                dy = ty - self.robot.y
                path_heading = np.arctan2(dy, dx)
                heading_error = path_heading - self.robot.theta
                heading_error = np.arctan2(
                    np.sin(heading_error),
                    np.cos(heading_error)
                )

            elif self.controller_type == "stanley":
                v, omega, cte, heading_error = self.controller.control(
                    self.robot.x,
                    self.robot.y,
                    self.robot.theta
                )

            else:
                raise ValueError("Controller tidak dikenali")

            # ===============================
            # Update kinematika (Differential Drive)
            # ===============================
            self.robot.step(v, omega)
            x, y, theta = self.robot.state()

            t += self.dt

            # ===============================
            # Logging state & error
            # ===============================
            self.state.update(x, y, theta, t)
            self.state.log_error(cte, heading_error)

            # ===============================
            # Stop jika dekat goal
            # ===============================
            goal_dist = np.hypot(
                self.path[-1][0] - x,
                self.path[-1][1] - y
            )
            if goal_dist < SIMULATION_PARAMS["goal_tolerance"]:
                break

        # ===============================
        # HITUNG PANJANG PATH
        # ===============================
        path_array = np.array(self.path)
        path_diffs = np.diff(path_array, axis=0)
        path_length = np.sum(np.hypot(path_diffs[:, 0], path_diffs[:, 1]))

        # ===============================
        # FINAL & AVERAGE ERROR
        # ===============================
        final_cte = self.state.cross_track_error[-1]
        final_heading_error = self.state.heading_error[-1]

        avg_cte = sum(self.state.cross_track_error) / len(self.state.cross_track_error)
        avg_heading_error = sum(self.state.heading_error) / len(self.state.heading_error)

        # ===============================
        # PERSENTASE ERROR TERHADAP PANJANG PATH
        # ===============================
        final_cte_percent = (final_cte / path_length) * 100
        avg_cte_percent = (avg_cte / path_length) * 100

        # ===============================
        # CETAK HASIL KE TERMINAL
        # ===============================
        print("\n=== SIMULATION RESULT ===")
        print(f"Final position: x={x:.2f}, y={y:.2f}, theta={theta:.2f} rad")
        print(f"Distance to goal: {goal_dist:.3f} m")
        print(f"Final cross-track error: {final_cte:.3f} m ({final_cte_percent:.2f} % of path)")
        print(f"Final heading error: {final_heading_error:.3f} rad")
        print(f"Average cross-track error: {avg_cte:.3f} m ({avg_cte_percent:.2f} % of path)")
        print(f"Average heading error: {avg_heading_error:.3f} rad")
        print(f"Total path length: {path_length:.3f} m")
        print("=========================\n")


        return self.state
