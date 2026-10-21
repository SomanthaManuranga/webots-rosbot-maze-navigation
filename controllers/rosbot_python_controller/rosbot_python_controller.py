# main.py
import random
import math
import numpy as np
from robot_controller import RosBot
from vision import VisionProcessor
from map_manager import MapManager
from planner import a_star_search
from constants import *
from pid import PIDController

# APPROACH-VIA knobs may be absent in some per-maze constants files → default to OFF here so
# `from constants import *` above doesn't leave them undefined (mazes without the geometry
# problem simply don't set them and get straight-at-pillar behaviour).
try:
    BLUE_APPROACH_VIA
except NameError:
    BLUE_APPROACH_VIA = (0.0, 0.0)
try:
    YELLOW_APPROACH_VIA
except NameError:
    YELLOW_APPROACH_VIA = (0.0, 0.0)
try:
    VIA_REACHED_RADIUS
except NameError:
    VIA_REACHED_RADIUS = 0.30

# Initialize this globally or in your main() setup
steering_pid = PIDController(Kp=PID_KP, Ki=PID_KI, Kd=PID_KD)


def approach_via_offset(state):
    """The (dx, dy) approach-via offset for the pillar the robot is driving to, or (0,0)."""
    if state == "GOING_TO_BLUE":
        return BLUE_APPROACH_VIA
    if state == "GOING_TO_YELLOW":
        return YELLOW_APPROACH_VIA
    return (0.0, 0.0)


def _map_update_common_check(robot, step_count):
    """Shared pre-flight check: skip if it's not an update step or if robot is idle."""
    if step_count % MAP_UPDATE_INTERVAL != 0:
        return False
    left, right = robot.last_left_speed, robot.last_right_speed
    if abs(left) < WHEEL_IDLE_THRESHOLD and abs(right) < WHEEL_IDLE_THRESHOLD:
        return False  # Wheels stopped → no new info, freeze the map
    return True

def safe_update_lidar_map(robot, map_manager, step_count):
    """
    Update the map with LiDAR-only points when NOT turning sharply.
    LiDAR is 360° — even a small rotation smears endpoints across wide arcs.
    Threshold: MAP_SKIP_TURN_SPEED_DIFF (tight, prevents smearing on moderate turns).
    """
    if not _map_update_common_check(robot, step_count):
        return
    if abs(robot.last_left_speed - robot.last_right_speed) > MAP_SKIP_TURN_SPEED_DIFF:
        return
    lidar_pts = robot.get_lidar_points_2d()
    if len(lidar_pts) > 0:
        map_manager.update_map(robot.x, robot.y, robot.theta, lidar_pts)

def _depth_points_lidar_missed(depth_local, lidar_local, match_cells):
    """Keep only the depth points the LiDAR did NOT also see → the FLOATING walls.

    A normal wall is hit by both sensors, so its depth points land on a LiDAR cell and get
    dropped (the LiDAR already maps it 360°; re-stamping it is what fattens the map over
    time). A floating plank the LiDAR beam passes under leaves no LiDAR return there, so its
    depth points survive and are still mapped geometrically. Comparison is done in the
    robot-LOCAL frame so no odometry drift enters the decision; match_cells absorbs the small
    forward offset between the LiDAR and depth-camera mounts.
    """
    if len(lidar_local) == 0:
        return depth_local          # no LiDAR this frame → can't call anything 'normal'; keep all
    inv = 1.0 / RESOLUTION
    lidar_cells = set()
    for lx, ly in lidar_local:
        lidar_cells.add((int(round(lx * inv)), int(round(ly * inv))))
    kept = []
    for dx, dy in depth_local:
        cx, cy = int(round(dx * inv)), int(round(dy * inv))
        seen_by_lidar = any((cx + ox, cy + oy) in lidar_cells
                            for ox in range(-match_cells, match_cells + 1)
                            for oy in range(-match_cells, match_cells + 1))
        if not seen_by_lidar:
            kept.append((dx, dy))
    return np.asarray(kept, dtype=np.float32) if kept else np.zeros((0, 2), dtype=np.float32)


def safe_update_depth_map(robot, map_manager, step_count):
    """
    Update the map with depth-camera-only points — even during PID approach curves.
    The depth camera has a narrow forward FOV, so rotational smearing during gentle
    curves is only ~1-3 pixels: negligible vs the 3-hit wall confirmation needed.
    Only skip during actual escape spins (diff > DEPTH_MAP_SKIP_TURN_SPEED_DIFF = 7.5).

    Gate comparison (ESCAPE_TURN_SPEED = 4.0 rad/s per side):
      Pure escape spin  → diff ≈ 8.0  > 7.5 → SKIPPED  ✓ (prevents sweep noise)
      Centering spin    → diff ≈ 6.0  < 7.5 → UPDATES   ✓ (may detect nearby wall)
      PID curve 0.6 rad → diff ≈ 5.6  < 7.5 → UPDATES   ✓ (floating wall confirmed)
    """
    if not _map_update_common_check(robot, step_count):
        return
    if abs(robot.last_left_speed - robot.last_right_speed) > DEPTH_MAP_SKIP_TURN_SPEED_DIFF:
        return
    depth_pts = robot.get_depth_camera_obstacle_points()
    if len(depth_pts) == 0:
        return
    # NORMAL-WALL SUPPRESSION: only stamp obstacles the LiDAR MISSED (floating walls). We
    # trust LiDAR to discriminate only when it isn't smearing (same turn gate as the LiDAR
    # map update); while turning we keep all depth points rather than risk a smeared LiDAR
    # arc suppressing a real floating plank.
    if (DEPTH_MAP_ONLY_LIDAR_MISSES
            and abs(robot.last_left_speed - robot.last_right_speed) <= MAP_SKIP_TURN_SPEED_DIFF):
        depth_pts = _depth_points_lidar_missed(depth_pts, robot.get_lidar_points_2d(),
                                               DEPTH_LIDAR_MATCH_CELLS)
        if len(depth_pts) == 0:
            return
    map_manager.update_map(robot.x, robot.y, robot.theta, depth_pts)

def safe_update_map(robot, map_manager, step_count):
    """
    Convenience wrapper — calls both LiDAR and depth-camera update functions.
    Existing call-sites (green_escape, escape_maneuver, main loop) keep working
    unchanged; each sub-function applies its own independent turn-speed gate.
    """
    safe_update_lidar_map(robot, map_manager, step_count)
    safe_update_depth_map(robot, map_manager, step_count)

def green_escape(robot, map_manager):
    """Do a perfect 180-degree turn to run away from the green floor!"""
    # Drop the stale path so the display doesn't show a forward line/dot while we recover.
    map_manager.path_to_draw = []
    robot.set_velocity(GREEN_ESCAPE_SPEED, -GREEN_ESCAPE_SPEED)
    for i in range(GREEN_ESCAPE_STEPS):
        robot.step(robot.time_step)
        robot.update_odometry()
        #map_manager.update_map(robot.x, robot.y, robot.theta, robot.get_lidar_points_2d())
        safe_update_map(robot, map_manager, i)
        if i % DISPLAY_UPDATE_INTERVAL == 0:   # keep the SLAM view live during recovery
            map_manager.render_display(robot.x, robot.y)
# THE FIX: Backup steps increased to 25. 10 steps was too short and caused panic loops!

def escape_maneuver(robot, map_manager, backup_steps=ESCAPE_BACKUP_STEPS, turn_steps=ESCAPE_TURN_STEPS):
    print("🛑 COLLISION/TRAP AVOIDANCE! Recovering safely...")
    # Invalidate the stale path immediately: the plan that led into this wall is no longer
    # valid, and we don't want the display showing a forward line/dot while we back out.
    map_manager.path_to_draw = []
    robot.set_velocity(-ESCAPE_BACKUP_SPEED, -ESCAPE_BACKUP_SPEED)

    for i in range(backup_steps):
        if min(robot.dist_sensors['rl'].getValue(), robot.dist_sensors['rr'].getValue()) < SAFE_DISTANCE:
            break
        robot.step(robot.time_step)
        robot.update_odometry()
        safe_update_map(robot, map_manager, 2)
        if i % DISPLAY_UPDATE_INTERVAL == 0:
            map_manager.render_display(robot.x, robot.y)

    if random.random() > 0.5: robot.set_velocity(ESCAPE_TURN_SPEED, -ESCAPE_TURN_SPEED)
    else: robot.set_velocity(-ESCAPE_TURN_SPEED, ESCAPE_TURN_SPEED)

    for i in range(turn_steps):
        robot.step(robot.time_step)
        robot.update_odometry()
        # DO NOT MAP WHILE TURNING (Prevents Smearing)
        if i % DISPLAY_UPDATE_INTERVAL == 0:
            map_manager.render_display(robot.x, robot.y)

def follow_path_steering(robot, path, pid_controller, dist_to_target=float('inf'), path_inflation=INFLATION_RADIUS):
    if not path or len(path) < 3:
        return -SEARCH_TURN_SPEED, -SEARCH_TURN_SPEED

    # Shorten the lookahead on tight (reduced-inflation) paths so the robot hugs the
    # centreline through bends instead of cutting the inside corner into the wall.
    lookahead = TIGHT_PATH_LOOKAHEAD if path_inflation < INFLATION_RADIUS else PURE_PURSUIT_LOOKAHEAD
    lookahead_index = min(lookahead, len(path) - 1)

    target_col, target_row = path[lookahead_index]
    target_x = (target_col - (MAP_SIZE // 2)) * RESOLUTION
    target_y = ((MAP_SIZE // 2) - target_row) * RESOLUTION

    # Calculate angular error
    angle_diff = math.atan2(target_y - robot.y, target_x - robot.x) - robot.theta
    angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

    # Dynamic Velocity Scaling (Slows down for sharp turns to prevent skid-steer slip)
    speed_factor = max(MIN_SPEED_FACTOR, 1.0 - (abs(angle_diff) / (math.pi / 2)))

    # --- APPROACH SLOWDOWN ---
    # When close to the goal, reduce speed so the map has time to accumulate enough
    # log-odds hits on any floating wall between the robot and the pillar.
    # This prevents the "wall confirmed too late → robot hits it" failure mode.
    if dist_to_target < APPROACH_SLOW_RADIUS:
        speed_factor *= APPROACH_SPEED_FACTOR

    # --- TIGHT-PATH SLOWDOWN ---
    # If A* could only route at reduced inflation, the path hugs walls (e.g. a gap by a
    # floating wall). Crawl so the PID tracks the centreline precisely and doesn't clip.
    if path_inflation < INFLATION_RADIUS:
        speed_factor *= TIGHT_PATH_SPEED_FACTOR

    current_base_speed = BASE_NAV_SPEED * speed_factor

    # PID Computation
    dt = robot.time_step / 1000.0  # Convert 32ms to seconds
    turn_speed = pid_controller.compute(angle_diff, dt)

    # NOTE: turn_speed is intentionally NOT clamped against the base speed. In tight
    # zig-zag corridors the robot must be able to pivot (inside wheel reversing) to make
    # sharp turns; clamping it caused the robot to curve into walls and wedge.
    return current_base_speed - turn_speed, current_base_speed + turn_speed



def map_floating_walls(robot, map_manager):
    """
    Map FLOATING walls by their full COLOUR area (same idea as the green-paint mapper),
    not the sparse per-column depth line that A* can thread.

    Colour-segment the wood planks, multi-point sample each blob's whole silhouette, depth-
    project every sample, keep only points inside the robot's BLOCKING height envelope
    (so the floor and high pass-under planks are excluded — same height test as the depth
    obstacle mapper), and stamp them SOLID + immediately. Colour confirms it's a real wall,
    which makes the instant dense stamp safe (depth noise that isn't wood is ignored).
    """
    blobs = VisionProcessor.get_wood_blobs(robot.camera)
    if not blobs:
        return
    depth_array = robot.get_depth_array()
    if depth_array is None:
        return

    # NORMAL-WALL SUPPRESSION for the COLOUR stamper too: every wall is wood, so colour
    # alone can't tell a plank from a wall — reuse the same "LiDAR saw it → normal wall →
    # skip" test as the geometric depth channel. LiDAR is only trustworthy when it isn't
    # smearing, so during turns we skip the whole stamp rather than re-fatten normal walls
    # (or suppress a real plank from a smeared LiDAR arc); the plank is re-captured once
    # the robot straightens out.
    filter_normal = DEPTH_MAP_ONLY_LIDAR_MISSES
    if filter_normal and abs(robot.last_left_speed - robot.last_right_speed) > MAP_SKIP_TURN_SPEED_DIFF:
        return

    dh, dw = depth_array.shape
    rgb_w, rgb_h = robot.camera.getWidth(), robot.camera.getHeight()
    sx, sy = dw / rgb_w, dh / rgb_h          # RGB → depth index scale (1.0 when aligned)
    cos_t, sin_t = math.cos(robot.theta), math.sin(robot.theta)

    local_points = []
    for blob in blobs:
        for (u, v) in blob['samples']:
            du = min(dw - 1, int(u * sx))
            dv = min(dh - 1, int(v * sy))
            depth = depth_array[dv, du]
            if np.isinf(depth) or np.isnan(depth) or not (SAFE_DISTANCE < depth < FLOATING_WALL_MAX_RANGE):
                continue
            # Real-world height of this point; keep only the robot's blocking envelope.
            elev = math.atan2(CAMERA_CY - dv, CAMERA_FX)
            height = CAMERA_MOUNT_HEIGHT + depth * math.sin(elev)
            if not (OBSTACLE_MIN_HEIGHT <= height <= ROBOT_PASS_UNDER_HEIGHT):
                continue
            local_x = depth
            local_y = -(du - CAMERA_CX) * depth / CAMERA_FX
            local_points.append((local_x, local_y))

    if not local_points:
        return

    # Drop wood points the LiDAR ALSO sees (normal walls, already LiDAR-mapped); keep only
    # LiDAR-missed points → genuine floating planks.
    if filter_normal:
        local_points = _depth_points_lidar_missed(np.asarray(local_points, dtype=np.float32),
                                                  robot.get_lidar_points_2d(), DEPTH_LIDAR_MATCH_CELLS)
        if len(local_points) == 0:
            return

    world_points = [(robot.x + (lx * cos_t - ly * sin_t),
                     robot.y + (lx * sin_t + ly * cos_t)) for (lx, ly) in local_points]
    map_manager.stamp_solid_cluster(world_points)


def map_green_patches(robot, map_manager):
    """
    PERCEPTION-BASED GREEN MAPPING with MULTI-POINT SAMPLING.

    For every green floor patch the camera sees, we sample MANY points across the blob
    (not just its centroid), project each through the depth camera into world space, and
    stamp a small permanent obstacle at each. This captures the patch's true shape and
    extent — long/irregular patches in Maze3/Maze5 are mapped accurately instead of
    being approximated by one square.

    Returns the depth (m) of the closest green sample that is roughly straight ahead,
    so the caller can decide whether a 180° escape is needed. inf if none ahead.
    """
    blobs = VisionProcessor.get_green_ground_blobs(robot.camera)
    if not blobs:
        return float('inf')

    depth_array = robot.get_depth_array()
    if depth_array is None:
        return float('inf')

    dh, dw = depth_array.shape
    rgb_w, rgb_h = robot.camera.getWidth(), robot.camera.getHeight()
    # RGB blob pixels → depth-array indices (scale = 1 when the cameras are aligned).
    sx, sy = dw / rgb_w, dh / rgb_h

    cos_t, sin_t = math.cos(robot.theta), math.sin(robot.theta)
    ahead_min_u = rgb_w * (0.5 - GREEN_AHEAD_COL_FRAC)
    ahead_max_u = rgb_w * (0.5 + GREEN_AHEAD_COL_FRAC)
    closest_ahead = float('inf')

    for blob in blobs:
        world_points = []
        for (u, v) in blob['samples']:
            du = min(dw - 1, int(u * sx))
            dv = min(dh - 1, int(v * sy))
            depth = depth_array[dv, du]
            if np.isinf(depth) or np.isnan(depth) or not (SAFE_DISTANCE < depth < GREEN_MAP_MAX_RANGE):
                continue

            # Pinhole projection: camera-local → world (same convention as map/pillars)
            local_x = depth
            local_y = -(du - CAMERA_CX) * depth / CAMERA_FX
            gx = robot.x + (local_x * cos_t - local_y * sin_t)
            gy = robot.y + (local_x * sin_t + local_y * cos_t)
            world_points.append((gx, gy))

            # Closest green sample roughly straight ahead → escape trigger
            if ahead_min_u < u < ahead_max_u and depth < closest_ahead:
                closest_ahead = depth

        # Stamp this blob's true footprint into the occupancy grid (renders black, like any obstacle)
        map_manager.stamp_solid_cluster(world_points)

    return closest_ahead


def get_smarter_waypoint(robot, map_manager):
    """Tries to find an unexplored frontier. Falls back to random if maze is fully mapped."""
    frontier_wp = map_manager.get_frontier_waypoint(robot.x, robot.y)

    if frontier_wp:
        print(f"🎯 Frontier found! Exploring unknown area at {frontier_wp[0]:.2f}, {frontier_wp[1]:.2f}")
        return frontier_wp
    else:
        print("🎲 No frontiers left! Picking random spot...")
        return random.uniform(-RANDOM_WP_RANGE, RANDOM_WP_RANGE), random.uniform(-RANDOM_WP_RANGE, RANDOM_WP_RANGE)


def snap_to_free_cell(grid, col, row):
    """
    If (col, row) sits inside an inflated wall on the grid, spiral outward until
    the nearest free cell (grid != 0) is found and return its (col, row).
    This prevents A* from failing because the robot's own position is blocked.
    """
    h, w = grid.shape
    if 0 <= row < h and 0 <= col < w and grid[row, col] != 0:
        return col, row  # Already in free space

    for radius in range(1, SNAP_TO_FREE_SEARCH_RADIUS + 1):
        for dr in range(-radius, radius + 1):
            for dc in range(-radius, radius + 1):
                # Only walk the outer ring of the square, not the interior
                if abs(dr) != radius and abs(dc) != radius:
                    continue
                nr, nc = row + dr, col + dc
                if 0 <= nr < h and 0 <= nc < w and grid[nr, nc] != 0:
                    return nc, nr  # Return as (col, row)

    return col, row  # No free cell found within search radius — return original
    


def mission_pillar_pos(state, map_manager):
    """The world position of the pillar the robot is actively driving to (or None)."""
    if state == "GOING_TO_BLUE":
        return map_manager.blue_pillar_pos
    if state == "GOING_TO_YELLOW":
        return map_manager.yellow_pillar_pos
    return None


def drive_toward_point(robot, tx, ty, pid_controller, base_speed=DIRECT_APPROACH_SPEED):
    """
    Direct heading control toward a world point. Used for the FINAL approach to a pillar
    (A* can't route into the pillar obstacle) and for BREADCRUMB RETRACE (base_speed passed
    in). The caller's reach check decides when to stop / advance.
    """
    angle = math.atan2(ty - robot.y, tx - robot.x) - robot.theta
    angle = (angle + math.pi) % (2 * math.pi) - math.pi
    dt = robot.time_step / 1000.0
    turn = pid_controller.compute(angle, dt)
    return base_speed - turn, base_speed + turn


def ema_pos(new_x, new_y, old_pos):
    """Exponential moving average for a pillar's world position. Smooths the dot so it
    doesn't jump across the map each frame as viewing angle/depth change, and converges
    as the robot approaches. Returns the raw reading the first time (old_pos is None)."""
    if old_pos is None:
        return new_x, new_y
    ox, oy = old_pos
    return (PILLAR_EMA_ALPHA * new_x + (1 - PILLAR_EMA_ALPHA) * ox,
            PILLAR_EMA_ALPHA * new_y + (1 - PILLAR_EMA_ALPHA) * oy)


def initial_pillar_scan(robot, map_manager):
    """
    Spin in place ~360° to (a) pre-locate every pillar visible from here — including one
    behind the robot the forward camera would otherwise miss — and (b) BUILD THE LOCAL MAP.

    The map-building matters for elevated/floating walls: the forward depth camera only
    sees the slice of such a wall inside its 60° FOV on approach (and the LiDAR passes
    under it), so a long horizontal plank is only partially mapped. Spinning here sweeps
    the depth camera across the wall's FULL extent so A* can route around it.

    During the spin the LiDAR map gate skips (turn too fast → no 360° smearing) but the
    depth-camera gate still updates, so it maps exactly the forward obstacles we want.
    Pure in-place rotation: no translation, safe near walls/green.
    """
    print("🔄 360° scan (pillars + local map)...")
    robot.set_velocity(SCAN_SPIN_SPEED, -SCAN_SPIN_SPEED)

    accumulated = 0.0
    prev_theta = robot.theta
    steps = 0

    while accumulated < 2 * math.pi and steps < SCAN_MAX_STEPS:
        if robot.step(robot.time_step) == -1:
            break
        robot.update_odometry()
        # FIX A (Maze1): map-building DISABLED during the spin. A full 360° rotation is the
        # worst dwell/smear case — depth frames slip the 7.5 gate and re-paint each wall
        # offset-by-offset into a fat wedge. The pillar-localisation below still runs; we just
        # don't write the map here. (Safe for Maze1: no elevated planks needing the sweep.)
        # safe_update_map(robot, map_manager, steps)   # build the local map during the sweep
        steps += 1

        # Accumulate absolute heading change (compass) until a full turn is covered.
        dtheta = (robot.theta - prev_theta + math.pi) % (2 * math.pi) - math.pi
        accumulated += abs(dtheta)
        prev_theta = robot.theta

        cos_t, sin_t = math.cos(robot.theta), math.sin(robot.theta)
        for color in ('blue', 'yellow'):
            mask = VisionProcessor.get_pillar_mask(robot.camera, color)
            if mask is None:
                continue
            centroid = VisionProcessor.centroid_of_mask(mask)
            if centroid is None:
                continue
            u, v = centroid
            depth = robot.get_pillar_depth(mask)
            if not (SAFE_DISTANCE < depth < MAX_PILLAR_DEPTH):
                continue
            local_x = depth
            local_y = -(u - CAMERA_CX) * depth / CAMERA_FX
            px = robot.x + (local_x * cos_t - local_y * sin_t)
            py = robot.y + (local_x * sin_t + local_y * cos_t)
            if color == 'blue':
                map_manager.blue_pillar_pos = ema_pos(px, py, map_manager.blue_pillar_pos)
            else:
                map_manager.yellow_pillar_pos = ema_pos(px, py, map_manager.yellow_pillar_pos)

        if steps % DISPLAY_UPDATE_INTERVAL == 0:
            map_manager.render_display(robot.x, robot.y)

    robot.stop()
    found = [c for c, p in (('BLUE', map_manager.blue_pillar_pos),
                            ('YELLOW', map_manager.yellow_pillar_pos)) if p]
    print(f"🔄 Scan complete. Pre-located: {found if found else 'none'}")


def main():
    robot = RosBot()
    map_manager = MapManager()
    
    # 1. Initialize the PID Controller
    steering_pid = PIDController(Kp=PID_KP, Ki=PID_KI, Kd=PID_KD)
    
    print("[Main] Robot ready. Starting exploration...")

    for _ in range(WARMUP_STEPS): robot.step(robot.time_step)
    robot.calibrate_odometry()

    # Pre-locate pillars visible from the start (incl. one behind the robot) so the
    # mission is known upfront — no slow post-BLUE search for YELLOW.
    initial_pillar_scan(robot, map_manager)

    STATE = "SEARCHING_BLUE"
    target_world_x, target_world_y = get_smarter_waypoint(robot, map_manager)
    step_count = 0
    path = []
    path_inflation = INFLATION_RADIUS   # inflation radius the current path was planned at
    force_replan = True   # set True after any escape → replan from the new position next step

    # --- PERCEPTION CACHE (updated every PERCEPTION_INTERVAL steps, reused between) ---
    green_ground = False
    closest_green = float('inf')
    seen_colors = set()

    # --- STUCK / WEDGE DETECTION state ---
    stuck_check_x, stuck_check_y = 0.0, 0.0
    stuck_counter = 0
    wedge_check_x, wedge_check_y = 0.0, 0.0

    # --- BREADCRUMB RETRACE state ---
    retracing = False       # True while driving the recorded trail back toward YELLOW
    retrace_idx = -1        # index into map_manager.trajectory we're currently heading to

    # --- APPROACH-VIA state ---
    via_reached = False     # True once the current mission's approach-via waypoint is reached

    # --- DETOUR RETRY state ---
    detour_start_step = 0   # step_count when the current DETOUR_* began (for the timed retry)

    while robot.step(robot.time_step) != -1:
        robot.update_odometry()
        safe_update_map(robot, map_manager, step_count)

        # Drop breadcrumbs while heading to BLUE so we can retrace the trail back to YELLOW.
        if "BLUE" in STATE:
            map_manager.record_trajectory(robot.x, robot.y, RETRACE_RECORD_DIST)

        # --- HEAVY PERCEPTION (throttled to every PERCEPTION_INTERVAL steps) ---
        # The 4 full 640×480 HSV conversions + green-blob extraction + per-pillar depth
        # reads are the dominant per-step cost. Running them EVERY step made the sim
        # compute-bound (Webots RTF ~0.00x), which is the real cause of the "stop & think
        # each step" stutter — not the robot deciding to pause. Pillars/green/floor don't
        # change in 0.1 s, so we refresh them every few steps and reuse the cached results
        # (green_ground, closest_green, seen_colors) in between. CONTROL — the state
        # machine, A*, path following and collision avoidance below — still runs EVERY
        # step, so the robot stays responsive.
        is_centering = False
        if step_count % PERCEPTION_INTERVAL == 0:
            mission_color = 'blue' if "BLUE" in STATE else 'yellow'
            green_ground = VisionProcessor.is_green_ground_ahead(robot.camera)
            closest_green = map_green_patches(robot, map_manager)
            map_floating_walls(robot, map_manager)   # map wood planks by full colour area
            seen_colors = set()   # which pillars are visible this perception cycle

            cam_width = robot.camera.getWidth()
            cos_t, sin_t = math.cos(robot.theta), math.sin(robot.theta)

            # --- OPPORTUNISTIC DUAL-PILLAR DETECTION (both colours, any angle) ---
            for color in ('blue', 'yellow'):
                mask = VisionProcessor.get_pillar_mask(robot.camera, color)
                if mask is None:
                    continue
                centroid = VisionProcessor.centroid_of_mask(mask)
                if centroid is None:
                    continue
                u, v = centroid
                offset = u - (cam_width // 2)
                seen_colors.add(color)

                # Spin-centre ONLY on the mission pillar and ONLY while SEARCHING.
                if color == mission_color and abs(offset) > CENTERING_PIXEL_THRESHOLD and "SEARCHING" in STATE:
                    turn_spd = max(min(offset * CENTERING_GAIN, CENTERING_MAX_SPEED), -CENTERING_MAX_SPEED)
                    robot.set_velocity(turn_spd, -turn_spd)
                    is_centering = True

                # Depth from ONLY the pillar's pixels (no far-background bias), then
                # project to world coordinates (pinhole) and EMA-smooth.
                depth = robot.get_pillar_depth(mask)
                if not (SAFE_DISTANCE < depth < MAX_PILLAR_DEPTH):
                    continue
                local_x = depth
                local_y = -(u - CAMERA_CX) * depth / CAMERA_FX
                px = robot.x + (local_x * cos_t - local_y * sin_t)
                py = robot.y + (local_x * sin_t + local_y * cos_t)

                # Current stored position + whether this colour is the active GOING_TO target.
                if color == 'blue':
                    stored = map_manager.blue_pillar_pos
                    going = (STATE == "GOING_TO_BLUE")
                else:
                    stored = map_manager.yellow_pillar_pos
                    going = (STATE == "GOING_TO_YELLOW")

                # LOCK the dot once we're close: at short range (esp. mid-turn) the pillar
                # sits at the frame edge and its localisation jitters; updating then would
                # move the target and make the robot re-route to a pillar it's already at.
                locked = (going and stored is not None and
                          math.hypot(robot.x - stored[0], robot.y - stored[1]) < PILLAR_LOCK_RADIUS)

                if not locked:
                    new_pos = ema_pos(px, py, stored)
                    if color == 'blue':
                        map_manager.blue_pillar_pos = new_pos
                    else:
                        map_manager.yellow_pillar_pos = new_pos

                if going:   # keep the target on the (possibly frozen) stored position
                    if color == 'blue':
                        target_world_x, target_world_y = map_manager.blue_pillar_pos
                    else:
                        target_world_x, target_world_y = map_manager.yellow_pillar_pos

        # --- STATE MACHINE (runs every frame; lightweight, uses cached seen_colors) ---
        if STATE == "SEARCHING_BLUE":
            if map_manager.blue_pillar_pos:
                print("🚀 Routing to BLUE...")
                STATE = "GOING_TO_BLUE"
                target_world_x, target_world_y = map_manager.blue_pillar_pos
                via_reached = False   # fresh mission → re-arm the approach-via waypoint
                path = []
                steering_pid.reset()
            elif math.hypot(target_world_x - robot.x, target_world_y - robot.y) < WAYPOINT_REACHED_RADIUS:
                target_world_x, target_world_y = get_smarter_waypoint(robot, map_manager)
                path = []
                steering_pid.reset()

        elif STATE == "GOING_TO_BLUE":
            # Measure distance to the actual PILLAR (not target_world, which A*-blocked may
            # have hijacked to a frontier) so we recognise arrival even sitting on top of it.
            bp = map_manager.blue_pillar_pos
            if bp is not None and math.hypot(bp[0] - robot.x, bp[1] - robot.y) < PILLAR_REACHED_RADIUS:
                # FIX: being within reach of the LOCKED pillar position counts as reached,
                # even if 'blue' dropped out of seen_colors this frame. At close range the
                # pillar fills/leaves the frame and the blob detection flickers; the OLD code
                # deleted blue_pillar_pos and reverted to SEARCHING here, throwing away a
                # pillar the robot had clearly arrived at (→ frontier-wander). The locked
                # position within PILLAR_REACHED_RADIUS is stronger evidence than a 1-frame blob.
                print("✅ Reached BLUE! Pausing 2 seconds...")
                robot.stop()
                for _ in range(BLUE_PAUSE_STEPS):
                    robot.step(robot.time_step)
                    robot.update_odometry()
                    if _ % 3 == 0: map_manager.render_display(robot.x, robot.y)
                # NOTE: the post-BLUE 360° scan is intentionally DISABLED for Maze4.
                # Yellow is already located by the startup scan and there are no floating
                # walls here, so the scan gives no benefit — and spinning at blue re-stamps
                # the junction (green patch + plank, green is sticky) into the map, SEALING
                # the return route to YELLOW. Skipping it keeps the path the robot already
                # drove through open, and saves time. (Re-enable for mazes with floating
                # walls near blue by uncommenting the line below.)
                # initial_pillar_scan(robot, map_manager)
                print("Moving on to YELLOW...")
                # FREEZE the map (optional): stop further map writes so the return plans
                # on the clean complete map.
                if FREEZE_MAP_AFTER_BLUE:
                    map_manager.frozen = True
                    print("❄️  Map FROZEN — return uses the complete scanned map (no more updates).")
                # BREADCRUMB RETRACE: drive the recorded BLUE trail in REVERSE. The robot
                # physically drove this route (incl. the junction), so it is guaranteed
                # drivable and immune to the A*-through-the-junction flicker.
                if ENABLE_BREADCRUMB_RETRACE and len(map_manager.trajectory) > 1:
                    retracing = True
                    retrace_idx = len(map_manager.trajectory) - 1
                    print(f"🍞 Retracing {len(map_manager.trajectory)} breadcrumbs back toward YELLOW...")
                STATE = "SEARCHING_YELLOW"
                # If yellow was already spotted en route, SEARCHING_YELLOW will route
                # straight there next frame; otherwise fall back to frontier search.
                target_world_x, target_world_y = get_smarter_waypoint(robot, map_manager)
                path = []
                force_replan = True
                steering_pid.reset()

        elif STATE == "DETOUR_BLUE":
            # KEEP the BLUE dot (deleting it made the dot vanish and forced a slow
            # face-to-face re-sighting). Exit the detour on EITHER (a) reaching the
            # relocation frontier, or (b) the retry TIMER: after DETOUR_RETRY_STEPS of
            # exploring, re-plan straight back to the kept dot — the approach angle and
            # the known map have changed, so the route that failed often exists now.
            # If the retry fails, the A*-fail branch re-enters DETOUR toward a fresh
            # frontier and the timer restarts (explore → retry → explore).
            arrived = math.hypot(target_world_x - robot.x, target_world_y - robot.y) < WAYPOINT_REACHED_RADIUS
            retry_due = (DETOUR_RETRY_STEPS > 0 and map_manager.blue_pillar_pos is not None
                         and step_count - detour_start_step > DETOUR_RETRY_STEPS)
            if arrived or retry_due:
                if map_manager.blue_pillar_pos is not None:
                    print(f"🔁 Detour {'timer' if retry_due and not arrived else 'reached'} — retrying route to BLUE (dot kept).")
                    STATE = "GOING_TO_BLUE"
                    target_world_x, target_world_y = map_manager.blue_pillar_pos
                    via_reached = False   # re-approach via the safe side from the new vantage
                else:
                    print("✅ Detour reached — BLUE unknown, re-searching.")
                    STATE = "SEARCHING_BLUE"
                    target_world_x, target_world_y = get_smarter_waypoint(robot, map_manager)
                path = []
                force_replan = True   # replan immediately, not in ≤15 steps
                steering_pid.reset()

        elif STATE == "SEARCHING_YELLOW":
            if map_manager.yellow_pillar_pos:
                print("🚀 Routing to YELLOW...")
                STATE = "GOING_TO_YELLOW"
                target_world_x, target_world_y = map_manager.yellow_pillar_pos
                via_reached = False   # fresh mission → re-arm the approach-via waypoint
                path = []
                steering_pid.reset()
            elif math.hypot(target_world_x - robot.x, target_world_y - robot.y) < WAYPOINT_REACHED_RADIUS:
                target_world_x, target_world_y = get_smarter_waypoint(robot, map_manager)
                path = []
                steering_pid.reset()

        elif STATE == "GOING_TO_YELLOW":
            yp = map_manager.yellow_pillar_pos
            if yp is not None and math.hypot(yp[0] - robot.x, yp[1] - robot.y) < PILLAR_REACHED_RADIUS:
                # FIX (same as BLUE): within reach of the locked YELLOW position = reached,
                # even if the blob dropped this frame. Prevents discarding the goal at the
                # finish line and bailing to frontier search one step before completion.
                print("🏁 YELLOW Reached! MAZE COMPLETED!")
                robot.stop()
                break

        elif STATE == "DETOUR_YELLOW":
            # KEEP the YELLOW dot + timed retry — same mechanism as DETOUR_BLUE above.
            arrived = math.hypot(target_world_x - robot.x, target_world_y - robot.y) < WAYPOINT_REACHED_RADIUS
            retry_due = (DETOUR_RETRY_STEPS > 0 and map_manager.yellow_pillar_pos is not None
                         and step_count - detour_start_step > DETOUR_RETRY_STEPS)
            if arrived or retry_due:
                if map_manager.yellow_pillar_pos is not None:
                    print(f"🔁 Detour {'timer' if retry_due and not arrived else 'reached'} — retrying route to YELLOW (dot kept).")
                    STATE = "GOING_TO_YELLOW"
                    target_world_x, target_world_y = map_manager.yellow_pillar_pos
                    via_reached = False   # re-approach via the safe side from the new vantage
                else:
                    print("✅ Detour reached — YELLOW unknown, re-searching.")
                    STATE = "SEARCHING_YELLOW"
                    target_world_x, target_world_y = get_smarter_waypoint(robot, map_manager)
                path = []
                force_replan = True   # replan immediately, not in ≤15 steps
                steering_pid.reset()
        # --- STATE MACHINE END ---

        # --- APPROACH-VIA: force a chosen approach side for a pillar ------------------
        # When the mission pillar has a configured (dx,dy) offset, redirect the target to an
        # intermediate "via" waypoint at pillar+(dx,dy) until the robot reaches it, THEN let
        # the normal approach close on the pillar itself — so it arrives from that side (e.g.
        # from above when a tilted plank blocks the low side of BLUE in Maze3). The via point
        # sits in free space, so A* can route to it (unlike the pillar obstacle itself).
        via = approach_via_offset(STATE)
        pillar_now = mission_pillar_pos(STATE, map_manager)
        via_active = (pillar_now is not None and (via[0] != 0.0 or via[1] != 0.0)
                      and not via_reached)
        if via_active:
            via_x = pillar_now[0] + via[0]
            via_y = pillar_now[1] + via[1]
            if math.hypot(via_x - robot.x, via_y - robot.y) < VIA_REACHED_RADIUS:
                via_reached = True
                via_active = False
                print("↪️  Reached approach-via waypoint — closing on pillar from the chosen side.")
            else:
                target_world_x, target_world_y = via_x, via_y   # route to the via point first

        dist_to_target = math.hypot(target_world_x - robot.x, target_world_y - robot.y)

        # Replan on the normal interval, OR immediately after an escape (force_replan) so
        # the robot never resumes a stale pre-collision path that points back into the wall.
        # SKIP A* entirely while retracing breadcrumbs — we follow the recorded trail, not A*.
        if not retracing and (step_count % ASTAR_REPLAN_INTERVAL == 0 or force_replan):
            force_replan = False
            raw_start = map_manager.world_to_map(robot.x, robot.y)
            goal_node  = map_manager.world_to_map(target_world_x, target_world_y)

            # --- FALLBACK A* LOOP ---
            # Try each inflation radius in order. If the robot's cell is inside an
            # inflated wall (common after escapes), snap it to the nearest free cell
            # first so A* doesn't fail on the very first node.
            new_path = []
            for radius in ASTAR_INFLATION_FALLBACK:
                safe_grid  = map_manager.get_inflated_grid(inflation_radius=radius)
                start_node = snap_to_free_cell(safe_grid, *raw_start)
                new_path   = a_star_search(safe_grid, start_node, goal_node)
                if new_path:
                    path_inflation = radius   # remember how tight this path is (for slowdown)
                    if radius < INFLATION_RADIUS:
                        print(f"⚠️  A* succeeded only with reduced inflation ({radius}px) — crawling through.")
                    break

            if new_path:
                path = new_path
                map_manager.path_to_draw = path
            else:
                # A* failed.
                path = []
                map_manager.path_to_draw = []
                mp = mission_pillar_pos(STATE, map_manager)
                # While steering to the via point, don't let an A* hiccup snap the target onto
                # the pillar (that would bee-line into the very side we're avoiding).
                near_pillar = (mp is not None and not via_active
                               and math.hypot(mp[0] - robot.x, mp[1] - robot.y) < DIRECT_APPROACH_RADIUS)
                if near_pillar:
                    # A* can't path INTO the pillar — direct-drive handles the final stretch.
                    target_world_x, target_world_y = mp
                elif STATE == "GOING_TO_BLUE":
                    # DETOUR: commit to relocating, then re-acquire BLUE from a new vantage.
                    print("🚧 Blocked en route to BLUE — DETOUR (relocate + re-acquire).")
                    STATE = "DETOUR_BLUE"
                    detour_start_step = step_count   # arm the timed retry-to-dot
                    target_world_x, target_world_y = get_smarter_waypoint(robot, map_manager)
                    steering_pid.reset()
                    force_replan = True
                elif STATE == "GOING_TO_YELLOW":
                    print("🚧 Blocked en route to YELLOW — DETOUR (relocate + re-acquire).")
                    STATE = "DETOUR_YELLOW"
                    detour_start_step = step_count   # arm the timed retry-to-dot
                    target_world_x, target_world_y = get_smarter_waypoint(robot, map_manager)
                    steering_pid.reset()
                    force_replan = True
                else:
                    # SEARCHING or already DETOUR → quietly pick another frontier (no spam loop).
                    target_world_x, target_world_y = get_smarter_waypoint(robot, map_manager)
                    steering_pid.reset()
                    force_replan = True

        step_count += 1
        if step_count % DISPLAY_UPDATE_INTERVAL == 0:
            if not map_manager.render_display(robot.x, robot.y): break

        # --- STUCK DETECTION ---
        # Checks every STUCK_CHECK_INTERVAL steps. If the robot has barely moved,
        # increments the counter. After STUCK_TRIGGER_COUNT consecutive stuck periods,
        # forces a deeper escape (more backup + more turn) to exit tight corners.
        if step_count % STUCK_CHECK_INTERVAL == 0 and step_count > 0:
            dist_moved = math.hypot(robot.x - stuck_check_x, robot.y - stuck_check_y)
            if dist_moved < STUCK_DISTANCE_THRESHOLD:
                stuck_counter += 1
                print(f"⚠️ Low progress: {dist_moved:.3f}m in {STUCK_CHECK_INTERVAL} steps (stuck count: {stuck_counter})")
                if stuck_counter >= STUCK_TRIGGER_COUNT:
                    print("🔄 GENUINELY STUCK! Forcing deep escape...")
                    escape_maneuver(robot, map_manager,
                                    backup_steps=ESCAPE_BACKUP_STEPS_DEEP,
                                    turn_steps=ESCAPE_TURN_STEPS_DEEP)
                    steering_pid.reset()
                    path = []
                    force_replan = True
                    target_world_x, target_world_y = get_smarter_waypoint(robot, map_manager)
                    stuck_counter = 0
            else:
                stuck_counter = 0  # making progress — reset the counter
            stuck_check_x, stuck_check_y = robot.x, robot.y

        # --- FAST WEDGE DETECTION ---
        # Compare commanded FORWARD motion to actual displacement. Using (left+right)/2 for
        # the forward command means in-place pivots (forward cmd ≈ 0) never false-trigger.
        # If we were told to drive forward but barely moved, the robot is jammed: mark the
        # blockage just ahead (so A* learns the obstacle the sensors under-mapped, e.g. a
        # floating wall), back out, and replan around it. Generalises to any maze/trap.
        if step_count % WEDGE_CHECK_INTERVAL == 0 and step_count > 0 and not is_centering:
            moved = math.hypot(robot.x - wedge_check_x, robot.y - wedge_check_y)
            forward_cmd = abs(robot.last_left_speed + robot.last_right_speed) / 2.0 * WHEEL_RADIUS
            expected = forward_cmd * (WEDGE_CHECK_INTERVAL * TIME_STEP / 1000.0)
            if expected > WEDGE_MIN_EXPECTED and moved < WEDGE_PROGRESS_FRAC * expected:
                print(f"🧱 WEDGED! commanded ~{expected:.3f}m, moved {moved:.3f}m. Marking blockage + backing out.")
                # Mark where the robot physically can't pass, so the next A* routes wider.
                map_manager.inject_virtual_obstacle(robot.x, robot.y, robot.theta,
                                                    dist_ahead=WEDGE_MARK_DIST,
                                                    block_size_m=WEDGE_MARK_SIZE, obs_type='wedge')
                escape_maneuver(robot, map_manager)
                steering_pid.reset()
                path = []
                force_replan = True
                stuck_counter = 0
            wedge_check_x, wedge_check_y = robot.x, robot.y

        # --- OBSTACLE LOGIC ---
        # (Green patches are mapped + closest_green computed in the throttled perception
        # block above; we just react to the cached value here, every step.)
        depth_obs, body_obs, lidar_obs = robot.check_obstacles()

        # Final pillar approach: A* can't route INTO the pillar (it's a mapped obstacle), and
        # the pillar itself trips the collision sensors. So when we're near the mission pillar
        # AND can see it, (a) DON'T flee it as an obstacle, and (b) drive straight at it. The
        # reach check (0.30 m > SAFE_DISTANCE 0.16 m) stops us before contact.
        mp = mission_pillar_pos(STATE, map_manager)
        dist_to_pillar = math.hypot(mp[0] - robot.x, mp[1] - robot.y) if mp is not None else float('inf')
        mcolor = 'blue' if STATE == "GOING_TO_BLUE" else ('yellow' if STATE == "GOING_TO_YELLOW" else None)
        # `not via_active`: while routing to the approach-via waypoint the robot is NOT yet at
        # the pillar, so keep normal obstacle avoidance on and don't direct-drive at the pillar.
        at_pillar = (mp is not None and mcolor in seen_colors
                     and dist_to_pillar < DIRECT_APPROACH_RADIUS and not via_active)

        if (green_ground or closest_green < GREEN_IMMINENT_RANGE) and not retracing:
            # Green-escape is SUPPRESSED during breadcrumb retrace. The trail goes through the
            # green-FREE gap the robot already drove (it reached blue without touching green),
            # so firing a 180° turn just because green is NEARBY traps the robot in a loop at
            # the junction. Following the known-safe trail crosses the gap without driving on
            # green. (Forward trip is unaffected — green escape still fully active there.)
            print(f"🟩 GREEN GROUND! Mapped patches; closest ahead {closest_green:.2f}m. Doing 180° turn...")
            green_escape(robot, map_manager)
            steering_pid.reset()
            stuck_counter = 0          # green escape counts as movement
            path = []                  # the pre-escape plan is invalid now
            force_replan = True        # replan from the new position next step
            green_ground = False           # clear the cached trigger so it doesn't re-fire
            closest_green = float('inf')   # before the next perception refresh

        elif (body_obs or lidar_obs or depth_obs) and not at_pillar and not retracing:
            # NOTE: collision-escape is SUPPRESSED during breadcrumb retrace. The trail is
            # known-drivable (the robot drove it to reach blue), so the sensors merely tripping
            # near a rung tip shouldn't trigger the disruptive back-up + random turn that made
            # the robot bounce off the rung and waste time. It pushes along the trail instead;
            # genuine wedges are still caught by the slower STUCK detection.
            escape_maneuver(robot, map_manager)
            steering_pid.reset()
            stuck_counter = 0          # escape counts as intentional movement
            path = []                  # don't drive back along the path into the same wall
            force_replan = True        # replan from the new position next step

        elif retracing:
            # BREADCRUMB RETRACE: drive the recorded BLUE trail in REVERSE, point by point.
            # Bypasses A* and the junction map entirely — guaranteed drivable, and retraces
            # the clean descent line. Green/collision escapes above still run for safety.
            if mp is not None and mcolor in seen_colors and dist_to_pillar < RETRACE_YELLOW_EXIT_RANGE and not via_active:
                # YELLOW visible & near → abandon the trail and drive STRAIGHT at the pillar.
                # (Suppressed while an approach-via waypoint is still pending, so we don't dash
                # in from the wrong side; the trail is followed until via handling takes over.)
                # Stays in 'retracing' mode (so escapes remain suppressed) and lets the state
                # machine's reached-check finish the mission — no A* handoff gap. Skips the slow
                # one-rung-at-a-time crawl up the last of the zig-zag once the goal is in sight.
                left_spd, right_spd = drive_toward_point(robot, mp[0], mp[1], steering_pid)
                robot.set_velocity(left_spd, right_spd)
            elif retrace_idx < 0:
                retracing = False          # trail exhausted → back near start/YELLOW
                force_replan = True        # hand off to normal A*/approach to YELLOW
                robot.stop()
            else:
                tx, ty = map_manager.trajectory[retrace_idx]
                if math.hypot(tx - robot.x, ty - robot.y) < RETRACE_REACH:
                    retrace_idx -= 1       # advance to the next breadcrumb (toward start)
                aim = map_manager.trajectory[max(0, retrace_idx)]
                # Slow down on sharp bends — at constant speed the skid-steer robot's corner
                # swings into the rung tip while turning. Scale speed down with heading error
                # (same dynamic slowdown the PID follower uses) so it rounds the rungs gently.
                aim_ang = math.atan2(aim[1] - robot.y, aim[0] - robot.x) - robot.theta
                aim_ang = (aim_ang + math.pi) % (2 * math.pi) - math.pi
                retrace_spd = RETRACE_SPEED * max(MIN_SPEED_FACTOR, 1.0 - abs(aim_ang) / (math.pi / 2))
                left_spd, right_spd = drive_toward_point(robot, aim[0], aim[1], steering_pid, retrace_spd)
                robot.set_velocity(left_spd, right_spd)
                # Draw the remaining trail on the SLAM view.
                map_manager.path_to_draw = [map_manager.world_to_map(px, py)
                                            for (px, py) in map_manager.trajectory[:retrace_idx + 1]]

        elif not is_centering:
            if not path and mp is not None and dist_to_pillar < DIRECT_APPROACH_RADIUS and not via_active:
                # Direct final approach — A* couldn't route into the pillar obstacle.
                left_spd, right_spd = drive_toward_point(robot, mp[0], mp[1], steering_pid)
            else:
                left_spd, right_spd = follow_path_steering(robot, path, steering_pid, dist_to_target, path_inflation)
            robot.set_velocity(left_spd, right_spd)
        
    
# --- THIS GOES AT THE VERY BOTTOM, OUTSIDE THE DEF MAIN() FUNCTION ---
if __name__ == "__main__":
    main()
    