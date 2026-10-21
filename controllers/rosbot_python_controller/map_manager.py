# map_manager.py
import numpy as np
import cv2
import math
import pygame
from constants import *

def bresenham_line(x0, y0, x1, y1):
    points = []
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx, sy = (1 if x0 < x1 else -1), (1 if y0 < y1 else -1)
    err = dx - dy
    while True:
        points.append((x0, y0))
        if x0 == x1 and y0 == y1: break
        e2 = 2 * err
        if e2 > -dy: err -= dy; x0 += sx
        if e2 < dx: err += dx; y0 += sy
    return points

class MapManager:
    def __init__(self):
        self.log_odds = np.zeros((MAP_SIZE, MAP_SIZE), dtype=np.float32)
        self.grid = np.full((MAP_SIZE, MAP_SIZE), 127, dtype=np.uint8)
        # VISITED mask: every cell the robot's BODY has physically occupied. Such a cell
        # cannot be an obstacle, so we keep the whole driven trajectory permanently free and
        # forbid the green/plank stamper from sealing it. Guarantees a retraceable corridor
        # (fixes the Maze4 junction re-sealing on the return to YELLOW).
        self.visited = np.zeros((MAP_SIZE, MAP_SIZE), dtype=bool)
        # BREADCRUMB TRAIL: the world (x, y) points the robot has actually driven on the way
        # to BLUE. Driving this trail in REVERSE is a guaranteed-drivable route back toward
        # YELLOW — the robot physically drove it — and is immune to the junction map state.
        self.trajectory = []
        # When True, ALL map writes are blocked (the map is frozen). Set after reaching BLUE
        # so the return trip plans on the complete, clean map instead of re-mapping/smearing.
        self.frozen = False
        self.path_to_draw = []
        self.blue_pillar_pos = None
        self.yellow_pillar_pos = None
        
        # --- NEW: Store virtual walls to draw them in color! ---
        self.virtual_obstacles = [] 
        
        pygame.init()
        self.window_size = DISPLAY_WINDOW_SIZE
        self.screen = pygame.display.set_mode((self.window_size, self.window_size))
        pygame.display.set_caption("Live SLAM & Path Planning")

    def world_to_map(self, x, y):
        col = int(x / RESOLUTION) + (MAP_SIZE // 2)
        row = (MAP_SIZE // 2) - int(math.ceil(y / RESOLUTION))
        return col, row


    def get_inflated_grid(self, inflation_radius=4):
        # 1. Get raw walls
        wall_mask = (self.log_odds > LOG_ODDS_WALL_THRESHOLD).astype(np.uint8) * 255

        # --- YOUR RESTORED NOISE FILTER! ---
        # 2. Find all connected blobs of walls
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(wall_mask, connectivity=8)
        
        # 3. Erase any blob that is smaller than MIN_WALL_BLOB_AREA pixels (LiDAR dust / ghost walls)
        for i in range(1, num_labels):
            if stats[i, cv2.CC_STAT_AREA] < MIN_WALL_BLOB_AREA:
                wall_mask[labels == i] = 0
        # -----------------------------------

        # 3b. GAP-CLOSING: seal tiny holes in walls (LiDAR noise / unmapped slivers) that
        # are narrower than the robot can fit through. Without this, A* threads a path
        # through a FALSE opening that is actually solid wall; the robot drives there and
        # wastes time discovering it's blocked. Real (≥ robot-width) corridors stay open.
        if GAP_CLOSE_KERNEL > 1:
            close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (GAP_CLOSE_KERNEL, GAP_CLOSE_KERNEL))
            wall_mask = cv2.morphologyEx(wall_mask, cv2.MORPH_CLOSE, close_kernel)

        # 4. Inflate the cleaned walls
        kernel_size = (inflation_radius * 2) + 1
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        inflated_walls = cv2.dilate(wall_mask, kernel, iterations=1)
        
        # 5. Apply to grid
        safe_grid = self.grid.copy()
        safe_grid[inflated_walls == 255] = 0
        return safe_grid


    def inject_virtual_obstacle_world(self, obs_x, obs_y, block_size_m=0.4, obs_type='green'):
        """
        Stamp a virtual obstacle at an ABSOLUTE world (x, y) with a given side length.
        Used by the perception-based green mapper, which computes the true location and
        size of each green patch from the depth camera (rather than guessing a fixed
        distance ahead).

        Safe to call every frame: the grid stamp is idempotent, and the pygame render
        list is de-duplicated so it never grows unbounded when a patch is re-seen.
        """
        if self.frozen:   # map locked post-BLUE
            return
        center_col, center_row = self.world_to_map(obs_x, obs_y)
        pixel_radius = max(1, int((block_size_m / 2.0) / RESOLUTION))

        # Force into the log-odds + occupancy grid (idempotent — repeat-safe).
        # LOG_ODDS_VIRTUAL_OBS >= LOG_ODDS_FREE_UPDATE_CAP, so these cells are immune
        # to free-space erasure and persist through the robot's escape turn.
        for r in range(center_row - pixel_radius, center_row + pixel_radius + 1):
            for c in range(center_col - pixel_radius, center_col + pixel_radius + 1):
                if 0 <= c < MAP_SIZE and 0 <= r < MAP_SIZE:
                    self.log_odds[r, c] = LOG_ODDS_VIRTUAL_OBS
                    self.grid[r, c] = 0

        # De-duplicate the render list (entries are (col, row, half_w, half_h, type)).
        for idx, (ec, er, ehw, ehh, etype) in enumerate(self.virtual_obstacles):
            if etype == obs_type and math.hypot(ec - center_col, er - center_row) < VIRTUAL_OBS_MERGE_DIST:
                self.virtual_obstacles[idx] = (ec, er, max(ehw, pixel_radius), max(ehh, pixel_radius), etype)
                return
        self.virtual_obstacles.append((center_col, center_row, pixel_radius, pixel_radius, obs_type))

    def stamp_solid_cluster(self, world_points):
        """
        MULTI-POINT solid-obstacle stamping. Given many world (x, y) points sampled across
        one coloured region (green paint OR a wood floating wall), stamp a small permanent
        obstacle at each so the region is captured at its true shape/extent in the grid.

        Cells become grid==0, log_odds==LOG_ODDS_VIRTUAL_OBS — sticky (immune to free-space
        erasure, so a floating wall isn't wiped by LiDAR rays passing under it) and the
        single source of truth A* avoids. Stamping is idempotent, so re-seeing a region
        each frame doesn't accumulate.
        """
        if self.frozen:   # map locked post-BLUE → don't let green/plank re-stamp the return path
            return
        if not world_points:
            return

        pad = GREEN_POINT_RADIUS_PX
        for (wx, wy) in world_points:
            c, r = self.world_to_map(wx, wy)
            if not (0 <= c < MAP_SIZE and 0 <= r < MAP_SIZE):
                continue
            for rr in range(r - pad, r + pad + 1):
                for cc in range(c - pad, c + pad + 1):
                    if 0 <= cc < MAP_SIZE and 0 <= rr < MAP_SIZE:
                        # NEVER stamp an obstacle on a cell the robot's body has driven
                        # through — it is provably free. This stops green's slant-distance
                        # over-projection from sealing the driven junction corridor.
                        if self.visited[rr, cc]:
                            continue
                        self.log_odds[rr, cc] = LOG_ODDS_VIRTUAL_OBS
                        self.grid[rr, cc] = 0

    def inject_virtual_obstacle(self, robot_x, robot_y, robot_theta, dist_ahead=0.35, block_size_m=0.4, obs_type='green'):
        """Legacy helper: place a block a fixed distance straight ahead of the robot.
        Kept for compatibility; green mapping now uses stamp_solid_cluster()."""
        obs_x = robot_x + (dist_ahead * math.cos(robot_theta))
        obs_y = robot_y + (dist_ahead * math.sin(robot_theta))
        self.inject_virtual_obstacle_world(obs_x, obs_y, block_size_m, obs_type)

    def update_map(self, robot_x, robot_y, robot_theta, local_lidar_points):
        if self.frozen:   # map complete & locked (post-BLUE) → no more LiDAR/depth/carve writes
            return
        robot_col, robot_row = self.world_to_map(robot_x, robot_y)
        if not (0 <= robot_col < MAP_SIZE and 0 <= robot_row < MAP_SIZE): return

        cos_t, sin_t = math.cos(robot_theta), math.sin(robot_theta)

        for point in local_lidar_points[::LIDAR_UPDATE_STRIDE]:
            global_x = robot_x + (point[0] * cos_t - point[1] * sin_t)
            global_y = robot_y + (point[0] * sin_t + point[1] * cos_t)
            hit_col, hit_row = self.world_to_map(global_x, global_y)
            
            if 0 <= hit_col < MAP_SIZE and 0 <= hit_row < MAP_SIZE:
                ray_cells = bresenham_line(robot_col, robot_row, hit_col, hit_row)
                for cx, cy in ray_cells[:-1]:
                    if self.log_odds[cy, cx] < LOG_ODDS_FREE_UPDATE_CAP:
                        self.log_odds[cy, cx] -= LOG_ODDS_FREE_DEC
                hx, hy = ray_cells[-1]
                self.log_odds[hy, hx] += LOG_ODDS_OCC_INC

        np.clip(self.log_odds, LOG_ODDS_MIN, LOG_ODDS_MAX, out=self.log_odds)
        self.grid[self.log_odds > LOG_ODDS_WALL_THRESHOLD] = 0
        self.grid[self.log_odds < LOG_ODDS_FREE_THRESHOLD] = 255
        self.grid[(self.log_odds >= LOG_ODDS_FREE_THRESHOLD) & (self.log_odds <= LOG_ODDS_WALL_THRESHOLD)] = 127

        # Carve the robot's footprint into the permanent VISITED free corridor (see
        # carve_visited) so the driven path can't be re-sealed by green/LiDAR.
        self.carve_visited(robot_x, robot_y)

    def record_trajectory(self, x, y, min_dist=0.15):
        """Append the robot's world position to the breadcrumb trail if it has moved at least
        min_dist from the last recorded point (keeps the trail compact). Used for the
        breadcrumb RETRACE back toward YELLOW."""
        if not self.trajectory:
            self.trajectory.append((x, y))
            return
        lx, ly = self.trajectory[-1]
        if math.hypot(x - lx, y - ly) >= min_dist:
            self.trajectory.append((x, y))

    def carve_visited(self, robot_x, robot_y):
        """
        Force the cells under the robot's BODY free and record them in the permanent VISITED
        trajectory. The robot cannot be inside a wall, so this is always valid; accumulated
        over the path it guarantees a retraceable corridor (keeps the junction open).
        MUST be callable every step — including during turns/pivots — so the trajectory has
        no gaps at bends. Idempotent, so calling it from both update_map and render_display
        is safe.
        """
        if ROBOT_FREE_CARVE_PX <= 0:
            return
        robot_col, robot_row = self.world_to_map(robot_x, robot_y)
        if not (0 <= robot_col < MAP_SIZE and 0 <= robot_row < MAP_SIZE):
            return
        r = ROBOT_FREE_CARVE_PX
        r0, r1 = max(0, robot_row - r), min(MAP_SIZE, robot_row + r + 1)
        c0, c1 = max(0, robot_col - r), min(MAP_SIZE, robot_col + r + 1)
        self.visited[r0:r1, c0:c1] = True
        self.log_odds[self.visited] = ROBOT_FREE_CARVE_VALUE
        self.grid[self.visited] = 255


    def get_frontier_waypoint(self, robot_x, robot_y):
        """
        Finds the borders between Free Space (white) and Unknown Space (gray) and returns
        the (X, Y) world coords of the BEST unexplored frontier to head for.

        Selection favours LARGE frontiers (big unexplored boundaries = high information
        gain) while still preferring nearer ones, via score = area / (distance + 1). This
        pushes the robot to expand into substantial unknown regions instead of nibbling
        tiny pockets or backtracking into already-explored area.
        """
        # 1. Isolate the Free Space (255) and the Unknown Space (127)
        free_mask = (self.grid == 255).astype(np.uint8) * 255
        unknown_mask = (self.grid == 127).astype(np.uint8) * 255

        # 2. Expand the Free Space slightly so it "bleeds" into the Unknown Space
        kernel = np.ones((3, 3), np.uint8)
        dilated_free = cv2.dilate(free_mask, kernel, iterations=FRONTIER_DILATE_ITER)

        # 3. Find the exact overlap! (Pixels that are Unknown BUT touching Free space)
        frontier_mask = cv2.bitwise_and(dilated_free, unknown_mask)

        # 4. Group these frontier pixels into clusters (blobs)
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(frontier_mask, connectivity=8)

        best_frontier = None
        best_score = -1.0
        robot_col, robot_row = self.world_to_map(robot_x, robot_y)

        # 5. Score every valid frontier by size-vs-distance and keep the best
        for i in range(1, num_labels): # Skip 0 (the background)
            area = stats[i, cv2.CC_STAT_AREA]
            if area > MIN_FRONTIER_AREA:
                fc, fr = centroids[i]
                dist = math.hypot(fc - robot_col, fr - robot_row)
                score = area / (dist + 1.0)   # big-and-near scores highest
                if score > best_score:
                    best_score = score
                    best_frontier = (fc, fr)

        # 6. If we found a frontier, convert it to World (X, Y) and return it
        if best_frontier:
            fc, fr = best_frontier
            world_x = (fc - (MAP_SIZE // 2)) * RESOLUTION
            world_y = ((MAP_SIZE // 2) - fr) * RESOLUTION
            return world_x, world_y
            
        return None # No frontiers found! (Maze is 100% explored)


    def render_display(self, robot_x, robot_y):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return False

        rgb_grid = cv2.cvtColor(self.grid, cv2.COLOR_GRAY2RGB)
        
        for (c, r, half_w, half_h, obs_type) in self.virtual_obstacles:
            color = (0, 255, 0) if obs_type == 'green' else (128, 0, 128)
            cv2.rectangle(rgb_grid, (c - half_w, r - half_h), (c + half_w, r + half_h), color, thickness=-1)
        
        # Draw A* Path (Cyan)
        if len(self.path_to_draw) > 1:
            for i in range(len(self.path_to_draw) - 1):
                cv2.line(rgb_grid, self.path_to_draw[i], self.path_to_draw[i+1], (255, 0, 0), 1)
                
            look_idx = min(PURE_PURSUIT_LOOKAHEAD, len(self.path_to_draw) - 1)
            dot_col, dot_row = self.path_to_draw[look_idx]
            cv2.circle(rgb_grid, (dot_col, dot_row), radius=2, color=(255, 120, 120), thickness=-1)
                
        if self.blue_pillar_pos:
            b_col, b_row = self.world_to_map(self.blue_pillar_pos[0], self.blue_pillar_pos[1])
            cv2.circle(rgb_grid, (b_col, b_row), radius=3, color=(0, 0, 255), thickness=-1)
            
        if self.yellow_pillar_pos:
            y_col, y_row = self.world_to_map(self.yellow_pillar_pos[0], self.yellow_pillar_pos[1])
            cv2.circle(rgb_grid, (y_col, y_row), radius=3, color=(255, 255, 0), thickness=-1)

        r_col, r_row = self.world_to_map(robot_x, robot_y)
        if 0 <= r_col < MAP_SIZE and 0 <= r_row < MAP_SIZE:
            cv2.circle(rgb_grid, (r_col, r_row), radius=3, color=(255, 128, 0), thickness=-1) 

        rgb_grid = np.transpose(rgb_grid, (1, 0, 2))
        surf = pygame.surfarray.make_surface(rgb_grid)
        surf = pygame.transform.scale(surf, (self.window_size, self.window_size))
        self.screen.blit(surf, (0, 0))
        pygame.display.flip()
        
        return True