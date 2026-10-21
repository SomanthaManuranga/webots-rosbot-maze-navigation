# robot_controller.py
from controller import Robot
import numpy as np
import math
from constants import *
import warnings 

class RosBot(Robot):
    def __init__(self):
        super().__init__()
        self.time_step = TIME_STEP
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.prev_left_enc, self.prev_right_enc = 0.0, 0.0

        self._setup_motors()
        self._setup_sensors()

        # --- NEW: Track last commanded speeds ---
        self.last_left_speed = 0.0
        self.last_right_speed = 0.0

    def _setup_motors(self):
        self.motors = {
            'fl': self.getDevice("fl_wheel_joint"), 'fr': self.getDevice("fr_wheel_joint"),
            'rl': self.getDevice("rl_wheel_joint"), 'rr': self.getDevice("rr_wheel_joint"),
        }
        for motor in self.motors.values():
            motor.setPosition(float('inf'))
            motor.setVelocity(0.0)

    def _setup_sensors(self):
        self.lidar = self.getDevice("laser")
        self.lidar.enable(self.time_step)
        self.lidar.enablePointCloud()
        
        self.encoders = {
            'fl': self.getDevice("front left wheel motor sensor"),
            'fr': self.getDevice("front right wheel motor sensor")
        }
        for enc in self.encoders.values(): enc.enable(self.time_step)
            
        self.camera = self.getDevice("camera rgb")
        self.camera.enable(self.time_step)

        self.depth_camera = self.getDevice("camera depth")
        self.depth_camera.enable(self.time_step)

        self.dist_sensors = {
            'fl': self.getDevice("fl_range"), 'fr': self.getDevice("fr_range"),
            'rl': self.getDevice("rl_range"), 'rr': self.getDevice("rr_range")
        }
        for sns in self.dist_sensors.values(): sns.enable(self.time_step)

        self.compass = self.getDevice("imu compass")
        self.compass.enable(self.time_step)

    def calibrate_odometry(self):
        self.prev_left_enc = self.encoders['fl'].getValue()
        self.prev_right_enc = self.encoders['fr'].getValue()
        self.x, self.y = 0.0, 0.0
        comp = self.compass.getValues()
        self.theta = math.atan2(comp[0], comp[1])
        print("[System] Odometry & Compass Calibrated.")

    def update_odometry(self):
        left_enc = self.encoders['fl'].getValue()
        right_enc = self.encoders['fr'].getValue()

        dist_left = (left_enc - self.prev_left_enc) * WHEEL_RADIUS
        dist_right = (right_enc - self.prev_right_enc) * WHEEL_RADIUS
        delta_distance = (dist_left + dist_right) / 2.0

        comp = self.compass.getValues()
        self.theta = math.atan2(comp[0], comp[1])

        self.x += delta_distance * math.cos(self.theta)
        self.y += delta_distance * math.sin(self.theta)

        self.prev_left_enc = left_enc
        self.prev_right_enc = right_enc

    def get_lidar_points_2d(self):
        cloud = self.lidar.getPointCloud()
        if not cloud: return np.array([])
        points = np.array([[p.x, p.y] for p in cloud])
        return points[~np.isinf(points).any(axis=1)]
    
    def get_depth_camera_obstacle_points(self):
        """
        Returns depth-camera obstacle points (no LiDAR), HEIGHT-FILTERED to the robot's
        vertical envelope.

        For every sampled pixel we compute the point's real height above the ground:
            height = CAMERA_MOUNT_HEIGHT + depth * sin( atan( (CAMERA_CY - row) / CAMERA_FX ) )
        and keep it only if OBSTACLE_MIN_HEIGHT <= height <= ROBOT_PASS_UNDER_HEIGHT.
        This (a) drops FLOOR points (height ≈ 0) so open ground isn't mapped as a wall, and
        (b) drops HIGH floating planks the short robot can drive UNDER (Maze3). A normal
        wall still maps via its lower body, which sits inside the envelope.

        Per column we keep the CLOSEST in-envelope point, so a near object doesn't hide a
        farther wall in another column and out-of-envelope noise is discarded.
        """
        depth_data = self.depth_camera.getRangeImage()
        if not depth_data:
            return np.zeros((0, 2), dtype=np.float32)

        width, height = self.depth_camera.getWidth(), self.depth_camera.getHeight()
        depth_array = np.array(depth_data).reshape((height, width))

        rows = np.arange(DEPTH_HEIGHT_ROW_START, min(DEPTH_HEIGHT_ROW_END, height), DEPTH_HEIGHT_ROW_STRIDE)
        cols = np.arange(0, width, DEPTH_SAMPLE_STRIDE)
        if len(rows) == 0 or len(cols) == 0:
            return np.zeros((0, 2), dtype=np.float32)

        sub = depth_array[np.ix_(rows, cols)]                       # (R, C)

        with warnings.catch_warnings():
            warnings.simplefilter("ignore", category=RuntimeWarning)
            # Elevation angle of each scanned row (positive = ray points upward).
            elev = np.arctan((CAMERA_CY - rows) / CAMERA_FX)        # (R,)
            sin_elev = np.sin(elev)[:, None]                        # (R, 1)
            # Real-world height above the ground for each sampled point.
            heights = CAMERA_MOUNT_HEIGHT + sub * sin_elev          # (R, C)

            valid = (~np.isinf(sub) & ~np.isnan(sub)
                     & (sub < FLOATING_WALL_MAX_RANGE)
                     & (heights >= OBSTACLE_MIN_HEIGHT)
                     & (heights <= ROBOT_PASS_UNDER_HEIGHT))

            # Closest valid (in-envelope) depth per column.
            masked = np.where(valid, sub, np.inf)                   # (R, C)
            col_min = np.min(masked, axis=0)                        # (C,)

        good = np.isfinite(col_min)
        if not np.any(good):
            return np.zeros((0, 2), dtype=np.float32)

        u = cols[good]
        d = col_min[good]
        local_x = d
        local_y = -(u - CAMERA_CX) * d / CAMERA_FX
        return np.column_stack((local_x, local_y)).astype(np.float32)

    def get_fused_lidar_and_depth_points(self):
        """Fuses LiDAR + depth camera into one 2D obstacle array.
        Still used by safe_update_map() (which calls both sub-functions).
        Refactored to reuse get_depth_camera_obstacle_points() — no code duplication."""
        lidar_points = self.get_lidar_points_2d()
        depth_points = self.get_depth_camera_obstacle_points()

        if len(depth_points) > 0:
            if len(lidar_points) > 0:
                return np.vstack((lidar_points, depth_points))
            return depth_points
        return lidar_points
    
    def get_depth_array(self):
        """Returns the full depth image as a (height, width) numpy array, or None.
        Lets callers read many pixels' depths from one device fetch (e.g. multi-point
        green sampling) instead of reshaping per pixel."""
        depth_data = self.depth_camera.getRangeImage()
        if not depth_data:
            return None
        width, height = self.depth_camera.getWidth(), self.depth_camera.getHeight()
        return np.array(depth_data).reshape((height, width))

    def get_pillar_depth(self, mask):
        """
        Distance to a pillar using ONLY its colour-mask pixels (not a geometric patch that
        would include far background around a thin pillar). Uses a low percentile so the
        estimate sits on the near front surface and rejects the few farther edge/occluded
        pixels — placing the pillar dot ON the pillar instead of beyond it.
        """
        depth_data = self.depth_camera.getRangeImage()
        if not depth_data:
            return float('inf')
        width, height = self.depth_camera.getWidth(), self.depth_camera.getHeight()
        depth_array = np.array(depth_data).reshape((height, width))
        if mask.shape != (height, width):
            return float('inf')   # RGB/depth resolution mismatch (shouldn't happen — both 640×480)

        vals = depth_array[mask > 0]
        vals = vals[~np.isinf(vals) & ~np.isnan(vals)]
        if len(vals) == 0:
            return float('inf')
        return float(np.percentile(vals, PILLAR_DEPTH_PERCENTILE))

    def get_depth_at_pixel(self, u, v, patch_size=DEPTH_PATCH_SIZE):
        """Extracts the median depth specifically at pixel (u, v) to ignore nearby floating walls."""
        depth_data = self.depth_camera.getRangeImage()
        if not depth_data: return float('inf')
            
        width, height = self.depth_camera.getWidth(), self.depth_camera.getHeight()
        depth_array = np.array(depth_data).reshape((height, width))
        
        # Create a safe 8x8 pixel box around our target
        u_min = max(0, u - patch_size)
        u_max = min(width, u + patch_size)
        v_min = max(0, v - patch_size)
        v_max = min(height, v + patch_size)
        
        patch = depth_array[v_min:v_max, u_min:u_max]
        
        # Filter out NaN/Inf holes
        valid_depths = patch[~np.isinf(patch) & ~np.isnan(patch)]
        
        # Return median to filter out noisy sensor edges
        return float(np.median(valid_depths)) if len(valid_depths) > 0 else float('inf')
    
    
    def get_min_depth_ahead(self):
        depth_data = self.depth_camera.getRangeImage()
        if not depth_data: return float('inf')

        width, height = self.depth_camera.getWidth(), self.depth_camera.getHeight()
        depth_array = np.array(depth_data).reshape((height, width))

        center_patch = depth_array[int(height * DEPTH_CENTER_ROW_MIN_FRAC) : int(height * DEPTH_CENTER_ROW_MAX_FRAC),
                                   int(width * DEPTH_CENTER_COL_MIN_FRAC) : int(width * DEPTH_CENTER_COL_MAX_FRAC)]
        valid_depths = center_patch[~np.isinf(center_patch) & ~np.isnan(center_patch)]
        return float(np.min(valid_depths)) if len(valid_depths) > 0 else float('inf')

    def get_min_depth_front_wide(self):
        """
        Wider obstacle check: upper 67% of image, central 80% of width.
        Catches floating walls (elevated planks) that sit OUTSIDE the narrow
        centre patch used by get_min_depth_ahead — especially at close range
        when the plank occupies upper image rows the narrow patch misses.
        """
        depth_data = self.depth_camera.getRangeImage()
        if not depth_data: return float('inf')

        width, height = self.depth_camera.getWidth(), self.depth_camera.getHeight()
        depth_array = np.array(depth_data).reshape((height, width))

        row_end   = int(height * DEPTH_FRONT_WIDE_ROW_FRAC)
        col_start = int(width  * DEPTH_FRONT_WIDE_COL_MIN)
        col_end   = int(width  * DEPTH_FRONT_WIDE_COL_MAX)

        wide_patch   = depth_array[:row_end, col_start:col_end]
        valid_depths = wide_patch[~np.isinf(wide_patch) & ~np.isnan(wide_patch)]
        return float(np.min(valid_depths)) if len(valid_depths) > 0 else float('inf')

    def check_obstacles(self):
        depth_obs = self.get_min_depth_ahead() < SAFE_DISTANCE
        # Wide check specifically catches elevated floating walls off-centre or in upper rows
        float_obs = self.get_min_depth_front_wide() < FLOATING_WALL_CLOSE_RANGE
        body_obs  = min(self.dist_sensors['fl'].getValue(), self.dist_sensors['fr'].getValue()) < SAFE_DISTANCE

        points = self.get_lidar_points_2d()
        lidar_obs = False
        if len(points) > 0:
            distances = np.linalg.norm(points, axis=1)
            angles = np.arctan2(points[:, 1], points[:, 0])
            front_distances = distances[(angles > -LIDAR_FRONT_HALF_ANGLE) & (angles < LIDAR_FRONT_HALF_ANGLE)]
            if len(front_distances) > 0 and np.min(front_distances) < SAFE_DISTANCE:
                lidar_obs = True
                
        return depth_obs or float_obs, body_obs, lidar_obs


    def is_rear_blocked(self):
        """True if the 360° LiDAR sees an obstacle within SAFE_DISTANCE BEHIND the robot.
        Used to stop a backup before reversing into a wall (the rear range sensors alone
        miss walls between them; the LiDAR covers the whole rear arc)."""
        points = self.get_lidar_points_2d()
        if len(points) == 0:
            return False
        distances = np.linalg.norm(points, axis=1)
        angles = np.arctan2(points[:, 1], points[:, 0])
        # Rear cone = directions near ±pi (straight behind).
        rear_mask = np.abs(angles) > (math.pi - LIDAR_FRONT_HALF_ANGLE)
        rear_d = distances[rear_mask]
        return len(rear_d) > 0 and np.min(rear_d) < SAFE_DISTANCE

    def set_velocity(self, left_speed, right_speed):
        left_speed = max(min(left_speed, MAX_WHEEL_SPEED), -MAX_WHEEL_SPEED)
        right_speed = max(min(right_speed, MAX_WHEEL_SPEED), -MAX_WHEEL_SPEED)
        self.motors['fl'].setVelocity(left_speed)
        self.motors['rl'].setVelocity(left_speed)
        self.motors['fr'].setVelocity(right_speed)
        self.motors['rr'].setVelocity(right_speed)

        # --- NEW: Save speeds for turn detection ---
        self.last_left_speed = left_speed
        self.last_right_speed = right_speed
        
    def stop(self):
        self.set_velocity(0.0, 0.0)