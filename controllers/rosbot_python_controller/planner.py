#planner.py - Implements the A* pathfinding algorithm with a clearance penalty and B-Spline smoothing for the Rosbot Python Controller
import heapq
import numpy as np
from scipy.ndimage import distance_transform_edt
from scipy.interpolate import splprep, splev
from constants import (BSPLINE_SMOOTHNESS, MIN_PATH_SMOOTH_LEN, MIN_SMOOTH_SAMPLES,
                       SMOOTH_SAMPLE_DENSITY, PLANNER_SAFE_DISTANCE_PX,
                       PLANNER_PENALTY_STRENGTH, ASTAR_GOAL_RADIUS,
                       UNKNOWN_SPACE_PENALTY)

def heuristic(a, b):
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

def smooth_path_bspline(path, smoothness=BSPLINE_SMOOTHNESS):
    """Takes the jagged A* pixels and bends them into a smooth mathematical curve."""
    if len(path) < MIN_PATH_SMOOTH_LEN:
        return path
    
    path_arr = np.array(path, dtype=float)
    x, y = path_arr[:, 0], path_arr[:, 1]
    
    # splprep crashes if consecutive points are identical. This safely removes duplicates.
    dx, dy = np.diff(x), np.diff(y)
    dist = np.sqrt(dx**2 + dy**2)
    valid_idx = np.insert(dist > 0, 0, True)
    x, y = x[valid_idx], y[valid_idx]
    
    if len(x) < 4:
        return path

    s_val = max(0.0, smoothness * len(x))
    try:
        tck, u = splprep([x, y], s=s_val, k=3)
        samples = max(MIN_SMOOTH_SAMPLES, len(x) * SMOOTH_SAMPLE_DENSITY)
        u_fine = np.linspace(0, 1, samples)
        smooth_coords = splev(u_fine, tck)
        
        # Convert back to pixel coordinates
        smooth_path = [(int(round(px)), int(round(py))) for px, py in zip(smooth_coords[0], smooth_coords[1])]
        
        # Clean up the array
        seen = set()
        unique_smooth_path = []
        for point in smooth_path:
            if point not in seen:
                seen.add(point)
                unique_smooth_path.append(point)
        return unique_smooth_path
    except Exception as e:
        print(f"[Planner] Smoothing failed, using raw path. Error: {e}")
        return path

def a_star_search(grid, start, goal):
    # --- 1. THE COSTMAP (Clearance Penalty) ---
    dist_to_obs = distance_transform_edt(grid > 0)

    penalty_map = np.where(dist_to_obs < PLANNER_SAFE_DISTANCE_PX,
                           PLANNER_PENALTY_STRENGTH * (1.0 - dist_to_obs / PLANNER_SAFE_DISTANCE_PX),
                           0.0).astype(np.float32)

    # --- 2. STANDARD A* SEARCH ---
    neighbors = [(0,1), (0,-1), (1,0), (-1,0), (1,1), (1,-1), (-1,1), (-1,-1)]
    close_set = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: heuristic(start, goal)}
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))
    height, width = grid.shape

    while oheap:
        current = heapq.heappop(oheap)[1]
        
        if current == goal or heuristic(current, goal) < ASTAR_GOAL_RADIUS:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
                
            raw_path = data[::-1]
            # Pass the raw jagged path through the B-Spline smoother!
            return smooth_path_bspline(raw_path, smoothness=BSPLINE_SMOOTHNESS)

        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            if 0 <= neighbor[0] < width and 0 <= neighbor[1] < height:
                if grid[neighbor[1], neighbor[0]] == 0: continue
            else: continue
                
            move_cost = 1.414 if i != 0 and j != 0 else 1.0

            # --- UNKNOWN-SPACE PENALTY ---
            # Grid cells with value 127 are unexplored (gray on map). A floating wall
            # that hasn't been confirmed yet lives in this gray space. Penalising these
            # cells steers A* toward known-free corridors (value 255) whenever possible,
            # so the robot doesn't blindly route through uncharted territory.
            if grid[neighbor[1], neighbor[0]] == 127:
                move_cost += UNKNOWN_SPACE_PENALTY

            # --- WALL CLEARANCE PENALTY ---
            cost_with_penalty = move_cost + penalty_map[neighbor[1], neighbor[0]]
            tentative_g_score = gscore[current] + cost_with_penalty

            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0): continue

            if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [n[1] for n in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))
                
    return []