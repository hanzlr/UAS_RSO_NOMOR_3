# visualization/animation.py

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation


def animate_simulation(grid, reference_path, robot_state, interval=50):
    """
    Animasi path tracking robot
    """

    # ===============================
    # KONVERSI PATH: (row,col) → (x,y)
    # ===============================
    ref = np.array([(c, r) for r, c in reference_path])

    xs = np.array(robot_state.x_history)
    ys = np.array(robot_state.y_history)

    fig, ax = plt.subplots(figsize=(8, 8))

    # ===============================
    # Plot MAP 
    # ===============================
    ax.imshow(
        grid,
        cmap="gray_r",
        origin="lower"
    )

    # ===============================
    # Reference path (A*)
    # ===============================
    ax.plot(
        ref[:, 0],
        ref[:, 1],
        "b--",
        linewidth=2,
        label="Reference Path (A*)"
    )

    # ===============================
    # Robot trajectory
    # ===============================
    traj_line, = ax.plot(
        [],
        [],
        "r-",
        linewidth=2,
        label="Robot Trajectory"
    )

    robot_point, = ax.plot(
        [],
        [],
        "ro",
        markersize=6,
        label="Robot"
    )

    # ===============================
    # Axis
    # ===============================
    ax.set_xlim(0, grid.shape[1])
    ax.set_ylim(0, grid.shape[0])
    ax.set_title("Path Tracking Animation")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.legend()
    ax.grid(False)

    def init():
        traj_line.set_data([], [])
        robot_point.set_data([], [])
        return traj_line, robot_point

    def update(frame):
        traj_line.set_data(xs[:frame + 1], ys[:frame + 1])
        robot_point.set_data([xs[frame]], [ys[frame]])
        return traj_line, robot_point

    ani = animation.FuncAnimation(
        fig,
        update,
        frames=len(xs),
        init_func=init,
        interval=interval,
        blit=True,
        repeat=False
    )

    plt.show()
    return ani
