# visualization/export_mp4.py

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation


def export_animation_mp4(
    grid,
    reference_path,
    robot_state,
    save_path,
    fps=20
):
    """
    Export animasi path tracking ke MP4

    Parameters
    ----------
    grid : 2D numpy array
        Grid map
    reference_path : list of (x, y)
        Jalur referensi A*
    robot_state : RobotState
        Hasil simulasi
    save_path : str
        Lokasi file MP4
    fps : int
        Frame per second
    """

    ref = np.array(reference_path)
    xs = np.array(robot_state.x_history)
    ys = np.array(robot_state.y_history)

    fig, ax = plt.subplots(figsize=(8, 8))

    # Plot map
    ax.imshow(
        grid,                
        cmap="gray_r",
        origin="lower"
    )

    # Reference path
    ax.plot(
        ref[:, 1],
        ref[:, 0],
        "b--",
        linewidth=2,
        label="Reference Path (A*)"
    )

    # Robot trajectory
    traj_line, = ax.plot([], [], "r-", linewidth=2, label="Robot Trajectory")
    robot_point, = ax.plot([], [], "ro", markersize=6)

    ax.set_xlim(0, grid.shape[1])
    ax.set_ylim(0, grid.shape[0])
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_title("Path Tracking (A* + Controller)")
    ax.legend()
    ax.grid(False)

    def init():
        traj_line.set_data([], [])
        robot_point.set_data([], [])
        return traj_line, robot_point

    def update(frame):
        traj_line.set_data(xs[:frame + 1], ys[:frame + 1])
        robot_point.set_data([xs[frame]], [ys[frame]])  # <<< FIX
        return traj_line, robot_point

    ani = animation.FuncAnimation(
        fig,
        update,
        frames=len(xs),
        init_func=init,
        blit=True,
        repeat=False
    )

    writer = animation.FFMpegWriter(
        fps=fps,
        metadata={"artist": "Path Tracking Simulation"},
        bitrate=1800
    )

    ani.save(save_path, writer=writer)
    plt.close(fig)
