# visualization/plot_path.py

import matplotlib.pyplot as plt
import numpy as np


def plot_path(reference_path, robot_state, save_path=None, show=True):
    """
    Plot perbandingan jalur referensi dan lintasan robot
    """

    # ===============================
    # KONVERSI PATH: (row,col) → (x,y)
    # ===============================
    ref = np.array([(c, r) for r, c in reference_path])

    plt.figure(figsize=(8, 8))

    # Reference path
    plt.plot(
        ref[:, 0],
        ref[:, 1],
        "k--",
        linewidth=2,
        label="Reference Path (A*)"
    )

    # Robot trajectory
    plt.plot(
        robot_state.x_history,
        robot_state.y_history,
        "r-",
        linewidth=2,
        label="Robot Trajectory"
    )

    # Start & Goal
    plt.scatter(
        ref[0, 0],
        ref[0, 1],
        c="green",
        s=100,
        marker="o",
        label="Start"
    )

    plt.scatter(
        ref[-1, 0],
        ref[-1, 1],
        c="blue",
        s=100,
        marker="x",
        label="Goal"
    )

    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Path Tracking Result")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()

    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches="tight")

    if show:
        plt.show()
    else:
        plt.close()
