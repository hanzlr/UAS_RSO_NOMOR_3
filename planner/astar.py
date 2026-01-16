# planner/astar.py

import heapq
import math


# ============================================================
# HEURISTIC FUNCTION (Manhattan Distance)
# ============================================================

def heuristic(a, b):
    """
    Heuristic A* (Manhattan distance)
    a, b: (row, col)
    """
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


# ============================================================
# A* PATH PLANNING
# ============================================================

def astar(grid, start, goal):
    """
    A* path planning on grid map

    Parameters:
    - grid  : numpy array (0 free, 1 obstacle)
    - start : (row, col)
    - goal  : (row, col)

    Returns:
    - path  : list of (row, col) from start to goal
    """

    rows, cols = grid.shape

    # Gerakan 4-arah (grid-based planning)
    motions = [
        (-1, 0),  # up
        (1, 0),   # down
        (0, -1),  # left
        (0, 1)    # right
    ]

    # Priority queue (f, node)
    open_set = []
    heapq.heappush(open_set, (0, start))

    came_from = {}
    g_score = {start: 0.0}

    f_score = {start: heuristic(start, goal)}

    while open_set:
        _, current = heapq.heappop(open_set)

        # Goal reached
        if current == goal:
            return reconstruct_path(came_from, current)

        for move in motions:
            neighbor = (current[0] + move[0], current[1] + move[1])

            # Cek batas grid
            if not (0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols):
                continue

            # Cek obstacle
            if grid[neighbor[0], neighbor[1]] == 1:
                continue

            tentative_g = g_score[current] + 1.0

            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f = tentative_g + heuristic(neighbor, goal)
                f_score[neighbor] = f
                heapq.heappush(open_set, (f, neighbor))

    # Jika tidak ditemukan path
    print("WARNING: Path not found!")
    return []


# ============================================================
# PATH RECONSTRUCTION
# ============================================================

def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


# ============================================================
# DEBUG TEST
# ============================================================

if __name__ == "__main__":
    import numpy as np
    from map.grid_map import grid, START, GOAL

    path = astar(grid, START, GOAL)
    print(f"Path length: {len(path)}")
