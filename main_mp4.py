# main_mp4.py

import os

from map.grid_map import grid, START, GOAL
from planner.astar import astar
from simulation.simulator import Simulator

from visualization.export_mp4 import export_animation_mp4
from visualization.plot_path import plot_path
from visualization.plot_error import plot_error

from config.parameters import TRACKING_PARAMS


def main():
    # =========================================================
    # 1. PATH PLANNING (A*)
    # =========================================================
    print("[INFO] Running A* path planning...")
    path = astar(grid, START, GOAL)

    if path is None:
        raise RuntimeError("Path tidak ditemukan oleh A*")

    print(f"[INFO] Path found with {len(path)} points")

    # =========================================================
    # 2. PILIH CONTROLLER
    # =========================================================
    controller_type = TRACKING_PARAMS["controller"]
    print(f"[INFO] Using controller: {controller_type}")

    # =========================================================
    # 3. SIMULASI
    # =========================================================
    sim = Simulator(
        path=path,
        controller_type=controller_type
    )

    robot_state = sim.run()
    print("[INFO] Simulation finished")

    # =========================================================
    # 4. SUB-FOLDER RESULTS
    # =========================================================
    results_folder = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "results"
    )
    os.makedirs(results_folder, exist_ok=True)

    # =========================================================
    # 5. EXPORT MP4
    # =========================================================
    mp4_path = os.path.join(results_folder, "simulation.mp4")
    print("[INFO] Exporting MP4 animation...")
    export_animation_mp4(
        grid=grid,
        reference_path=path,
        robot_state=robot_state,
        save_path=mp4_path,
        fps=20
    )
    print(f"[INFO] MP4 saved to {mp4_path}")

    # =========================================================
    # 6. SIMPAN GAMBAR PENDUKUNG
    # =========================================================
    trajectory_path = os.path.join(results_folder, "trajectory.png")
    error_plot_path = os.path.join(results_folder, "error_plot.png")

    print("[INFO] Saving trajectory plot...")
    plot_path(
        reference_path=path,
        robot_state=robot_state,
        save_path=trajectory_path,
        show=False
    )

    print("[INFO] Saving error plot...")
    plot_error(
        robot_state=robot_state,
        save_path=error_plot_path,
        show=False
    )

    print(f"[INFO] All results saved in '{results_folder}' folder")
    print("[INFO] Done!")


if __name__ == "__main__":
    main()
