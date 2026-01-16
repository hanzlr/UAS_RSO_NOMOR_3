# main.py

from map.grid_map import grid, START, GOAL
from planner.astar import astar
from simulation.simulator import Simulator

from visualization.plot_path import plot_path
from visualization.plot_error import plot_error
from visualization.animation import animate_simulation

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
    controller_type = "pure_pursuit"

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
    # 4. VISUALISASI
    # =========================================================
    print("[INFO] Showing animation...")
    animate_simulation(
        grid=grid,
        reference_path=path,
        robot_state=robot_state
    )

    print("[INFO] Plotting trajectory...")
    plot_path(
        reference_path=path,
        robot_state=robot_state,
        show=True
    )

    print("[INFO] Plotting error...")
    plot_error(
        robot_state=robot_state,
        show=True
    )


if __name__ == "__main__":
    main()
