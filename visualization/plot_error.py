# visualization/plot_error.py

import matplotlib.pyplot as plt
import numpy as np


def plot_error(robot_state, save_path=None, show=True):
    """
    Plot error lintasan terhadap waktu
    """

    # ===============================
    # AMBIL DATA DENGAN FALLBACK
    # ===============================
    if hasattr(robot_state, "time"):
        time = robot_state.time
    elif hasattr(robot_state, "time_history"):
        time = robot_state.time_history
    else:
        raise AttributeError("RobotState tidak punya time / time_history")

    if hasattr(robot_state, "cross_track_error"):
        cte = robot_state.cross_track_error
    elif hasattr(robot_state, "cte_history"):
        cte = robot_state.cte_history
    else:
        raise AttributeError("RobotState tidak punya cross_track_error / cte_history")

    if hasattr(robot_state, "heading_error"):
        heading_error = robot_state.heading_error
    elif hasattr(robot_state, "heading_error_history"):
        heading_error = robot_state.heading_error_history
    else:
        raise AttributeError("RobotState tidak punya heading_error / heading_error_history")

    # ===============================
    # KONVERSI & SAMAKAN PANJANG
    # ===============================
    time = np.array(time)
    cte = np.array(cte)
    heading_error = np.array(heading_error)

    min_len = min(len(time), len(cte), len(heading_error))

    time = time[:min_len]
    cte = cte[:min_len]
    heading_error = heading_error[:min_len]

    # ===============================
    # PLOT
    # ===============================
    plt.figure(figsize=(10, 5))

    plt.subplot(1, 2, 1)
    plt.plot(time, cte, linewidth=2)
    plt.xlabel("Time [s]")
    plt.ylabel("Cross-Track Error [m]")
    plt.title("Cross-Track Error vs Time")
    plt.grid(True)

    plt.subplot(1, 2, 2)
    plt.plot(time, heading_error, linewidth=2)
    plt.xlabel("Time [s]")
    plt.ylabel("Heading Error [rad]")
    plt.title("Heading Error vs Time")
    plt.grid(True)

    plt.tight_layout()

    if save_path is not None:
        plt.savefig(save_path, dpi=300, bbox_inches="tight")

    if show:
        plt.show()
    else:
        plt.close()
