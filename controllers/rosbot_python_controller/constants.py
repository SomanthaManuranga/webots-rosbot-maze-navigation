# constants.py

# --- ROBOT HARDWARE ---
TIME_STEP = 32
MAX_WHEEL_SPEED = 10.0
WHEEL_RADIUS = 0.043
AXLE_LENGTH = 0.19
# Collision-avoidance trigger distance (m). Robot is ~0.19 m wide, so it needs
# ~0.19 + 2×SAFE_DISTANCE of corridor width to pass without a false "blocked".
# Raising this makes the robot react to walls earlier (fewer hits) but risks false
# escapes in the narrow zig-zag passages. 0.18 is a conservative bump from 0.15;
# if narrow corridors start triggering constant escapes, drop back to 0.15.
SAFE_DISTANCE = 0.16


# --- VISION: COLOR BOUNDS (HSV) ---
RED_LOWER1 = [0, 120, 70]
RED_UPPER1 = [10, 255, 255]
RED_LOWER2 = [170, 120, 70]
RED_UPPER2 = [180, 255, 255]

BLUE_LOWER = [100, 150, 50]
BLUE_UPPER = [140, 255, 255]

YELLOW_LOWER = [20, 100, 100]
YELLOW_UPPER = [35, 255, 255]

# Wooden planks / walls (tan). Used to map FLOATING walls by their full colour area (like
# the green-paint mapper) instead of a sparse per-column depth line. The gray floor has
# near-zero saturation so the S floor (>= 40) separates wood from floor. Tune per world if
# the planks aren't picked up (widen H/S) or the floor leaks in (raise the S lower bound).
WOOD_LOWER = [5, 40, 55]
WOOD_UPPER = [30, 230, 255]


GREEN_LOWER = [36, 100, 100]
GREEN_UPPER = [86, 255, 255]


# --- VISION: PIXEL THRESHOLDS ---
RED_PIXEL_THRESHOLD = 0.15      # fraction of image that triggers red-wall detection
GREEN_PIXEL_THRESHOLD = 0.25    # fraction of floor that triggers green-ground escape
MIN_PILLAR_PIXELS = 50          # minimum blob size to consider a pillar real.   if pillar does not detected change to 30

# --- VISION: PILLAR TARGETING ---
CAMERA_FX = 558.89              # focal length x (pixels) from camera intrinsics
CAMERA_CX = 320.0               # principal point x (pixels)
# Orbbec Astra depth camera hardware range is 0.6 m – 8.0 m. We were capping pillar
# localization at 4.0 m, discarding the whole 4–8 m band and forcing the robot to drive
# blind toward far pillars (e.g. the distant BLUE in Maze2). Raised to 7.5 m (just under
# the 8.0 m hardware limit, to avoid the maxRange "no-hit" edge) so a far pillar is
# localized the moment it is in view. EMA + continuous refinement on approach correct the
# larger position error inherent to long-range readings.



#pillar

MAX_PILLAR_DEPTH = 5.0        # ignore pillar readings beyond this distance (m); was 4.0
CENTERING_PIXEL_THRESHOLD = 10  # pixel offset below which robot is considered centred on pillar
CENTERING_GAIN = 0.05           # P-gain for pixel-centering turn controller
CENTERING_MAX_SPEED = 3.0       # clamp for centering turn speed (rad/s equivalent)




# --- APPROACH-VIA (per-pillar forced approach side) ---------------------------------
# (dx, dy) metres added to a pillar's position to make an intermediate "via" waypoint the
# robot must reach BEFORE closing on the pillar. Forces approaching the pillar from a chosen
# side (e.g. from above when a tilted plank blocks the low side). +y = up on the SLAM view.
# (0.0, 0.0) = disabled → drive straight at the pillar like normal. Opt-in PER MAZE.
BLUE_APPROACH_VIA   = (0.0, 0.0)   # constants_maze3.py — approach BLUE from above
YELLOW_APPROACH_VIA = (0.0, 0.0)   # symmetric knob for YELLOW; (0,0)=off. Set per maze if needed.
VIA_REACHED_RADIUS  = 0.30         # m — how close counts as having reached the via waypoint






# Pillar position EMA: blends each new camera estimate with the stored position.
# Prevents the blue/yellow dot jumping across the map as angle/depth change.
# α=1.0 → always use raw reading; α→0 → nearly frozen. 0.45 converges fast enough that the
# dot doesn't lag FORWARD of the pillar during approach (the cause of the overshoot), while
# the mask-based depth below keeps each reading stable enough that 0.45 doesn't jitter.
PILLAR_EMA_ALPHA = 0.15         # exponential moving average weight for pillar position updates
# Pillar depth is taken from ONLY the pillar's coloured pixels (not a geometric patch that
# leaks onto far background). A low percentile picks the near front surface, rejecting the
# few farther edge/occluded pixels — so the dot lands ON the pillar, not beyond it.
PILLAR_DEPTH_PERCENTILE = 25    # percentile of masked pillar depths used as the distance
# Once the robot is within this distance of the mission pillar (while GOING_TO it), FREEZE
# the pillar position. Up close — especially mid-turn — the pillar is at the frame edge and
# its localisation jitters; without a lock that jitter moves the target and the robot keeps
# "re-deciding" to go to a pillar it's already at. The ~1 m estimate is accurate enough to
# finish the approach to.
PILLAR_LOCK_RADIUS = 2.0        # m — stop updating the mission pillar's position within this range
# A pillar is both the GOAL and a mapped OBSTACLE, so A* can't path INTO it — it fails once
# the robot is near. For the final stretch we drive STRAIGHT at the pillar instead of using
# A*. This radius is where that direct approach takes over; PILLAR_REACHED_RADIUS (0.30 m,
# wider than SAFE_DISTANCE 0.16 m) then stops the robot before collision-avoidance fights it.
DIRECT_APPROACH_RADIUS = 0.7    # m — within this range of the mission pillar, drive straight at it
DIRECT_APPROACH_SPEED  = 3.0    # wheel speed for the direct final approach to a pillar

# --- DETOUR: KEEP THE PILLAR DOT + TIMED RETRY ---
# On DETOUR the pillar dot is now KEPT (not deleted): perception still refines it via EMA
# whenever the pillar is visible, and the robot no longer needs a face-to-face re-sighting
# to resume the mission. The detour ends when EITHER (a) the relocation frontier is reached,
# or (b) this TIMER fires: after this many main-loop steps of detour exploring, re-plan
# straight back to the kept dot — the approach angle and the known map have both changed,
# so the route that failed before often exists now. If the retry fails, the A*-fail branch
# re-enters DETOUR toward a fresh frontier and the timer restarts (explore → retry → explore).
# 0 disables ONLY the timer (retry then happens on frontier arrival alone).
DETOUR_RETRY_STEPS = 100        # ~6.4 s at 32 ms/step; lower = retry pillar more eagerly

# --- BREADCRUMB RETRACE (robust BLUE→YELLOW return) ---
# The robot records the trail it drove to BLUE; to return it drives that trail IN REVERSE,
# point by point, bypassing A* and the junction map entirely (it physically drove this
# route, so it's guaranteed drivable). This also avoids the return-clipping because it
# retraces the same clean line it descended.
ENABLE_BREADCRUMB_RETRACE = False   # master toggle: drive recorded trail in reverse on return
RETRACE_RECORD_DIST = 0.15      # m — drop a breadcrumb each time the robot moves this far
RETRACE_REACH       = 0.25      # m — advance to the next breadcrumb when within this range
# 6.0 base speed for fast straights. The retrace already SLOWS at bends (scales by heading
# error, floored at MIN_SPEED_FACTOR), so the tight rung corners auto-crawl while the long
# straight sections run fast — much less total time than a flat slow speed.
RETRACE_SPEED       = 6.0       # base wheel speed while retracing (auto-slows at bends)
# End the trail-follow and drive STRAIGHT at YELLOW once it's visible and within this range,
# so the robot doesn't crawl the last few upper-zig-zag rungs one-by-one when the goal is
# already in sight. Larger = cuts to the pillar sooner (faster) but risks clipping a rung
# still between robot and pillar; yellow being visible means roughly clear line of sight.
RETRACE_YELLOW_EXIT_RANGE = 1.0  # m

# --- MAP FREEZE AFTER BLUE ---
# By the time the robot reaches BLUE it has driven the whole maze, so the map is COMPLETE.
# Freezing it then stops all further map writes (LiDAR/depth/green stamping) on the return
# trip, which prevents the three things that wreck the BLUE→YELLOW leg: green re-sealing the
# junction, odometry-drift wall smear, and collision-induced map collapse. The return then
# plans on the clean, complete map. Real-time collision avoidance + green escape still run
# (they read live sensors, not the map). PER-MAZE: only safe when the maze is fully mapped
# by the time BLUE is reached (true for Maze4). Set False if BLUE is reached early.
FREEZE_MAP_AFTER_BLUE = False

# --- MAP CONSTANTS ---
MAP_SIZE = 300                  # 300x300 pixel grid
RESOLUTION = 10.0 / MAP_SIZE    # metres per pixel (~0.033 m/px)

# --- SLAM: MAP UPDATE GATING ---
# LiDAR map gate — LiDAR is 360° and smears badly during turns; keep this tight.
# At angle_diff=0.3 rad: turn_speed=1.8, left=6.2, right=9.8, diff=3.6 → OK
# At angle_diff=0.5 rad: diff≈5.0 → borderline; at 0.55 rad diff>5 → blocked. Good.
MAP_SKIP_TURN_SPEED_DIFF = 6.0  # skip LiDAR map update when |left-right| > this

# Depth-camera map gate — depth cam has a narrow forward FOV so smearing during
# gentle/moderate turns is small. Use a looser gate so the floating wall accumulates
# hits even during PID approach curves. Only skip during genuine escape spins.
#   Centering spin (CENTERING_MAX_SPEED=3.0): diff=6.0 < 7.5 → still updates ✓
#   Escape spin (ESCAPE_TURN_SPEED=4.0): diff=8.0 > 7.5 → skipped ✓
DEPTH_MAP_SKIP_TURN_SPEED_DIFF = 4.0  # skip depth-camera update when |left-right| > this

MAP_UPDATE_INTERVAL = 2         # only update map every Nth step
WHEEL_IDLE_THRESHOLD = 0.1      # wheel speed below this is considered stopped

# --- PERFORMANCE: PERCEPTION RATE ---
# The heavy vision (4 full 640×480 HSV conversions + green blob extraction + per-pillar
# depth) is the dominant per-step cost; at every step it makes the sim compute-bound
# (Webots RTF ~0). Run it every Nth step instead — pillars/green/floor don't change in
# 0.1 s. CONTROL (path following, collision avoidance) still runs every step. 3 → ~3×
# faster sim. Raise for more speed (coarser perception); lower (2) for finer perception.
PERCEPTION_INTERVAL = 3

# --- SLAM: LOG-ODDS TUNING ---
# Evidence model: a cell needs ceil(WALL_THRESHOLD / OCC_INC) consistent hits to become a wall.
# 0.8 inc / 2.0 threshold → needs 3 hits → robust to single noisy readings.
#
# KEY RULE — FREE_UPDATE_CAP == WALL_THRESHOLD:
#   Any cell with log_odds >= 2.0 is a confirmed wall and is NEVER decremented
#   by free-space rays (e.g. LiDAR passing under a floating plank at close range).
#   This prevents the "map erases floating wall as robot approaches" bug.
# FIX C (Maze1, raised 0.8 → 1.0): make free-ray erosion WIN TIES against occupied hits.
# With FREE_DEC (1.0) > OCC_INC (0.8), a cell stays a wall only if hit on the MAJORITY of
# observations: 0.8·p ≥ 1.0·(1−p) → p ≥ 0.556. True wall CORES (hit ~every ray) survive;
# edge/jitter cells (hit only slightly more often than cleared — the exact cause of creeping
# fattening) now LOSE the race and decay back to free. Cleanest single anti-thickening knob;
# also keeps the goal-area gap from closing. Pairs with cap=3.5 (strong core immune → no holes).
LOG_ODDS_FREE_DEC = 0.3        # gentle erosion; equilibrium sits between wall & noise (was 1.0)
LOG_ODDS_OCC_INC = 0.8          # log-odds increment for occupied hit cell
# Raised 2.5 → 3.5 (Maze4 wall-thinning). The old rule FREE_UPDATE_CAP == WALL_THRESHOLD
# made EVERY confirmed wall cell (>=2.5) immune to free-ray erasure, so noise + odometry
# drift could only ADD wall cells, never remove them → walls grew thicker the longer the
# run (a one-way ratchet). With the cap at 3.5, only STRONGLY-confirmed cells (>3.5, hit
# ~5+ times = real solid walls and the 5.0 sticky green stamps) stay immune, while
# WEAKLY-confirmed cells (2.5–3.5 = noise/drift edge band) CAN erode when a free ray passes
# → walls self-thin back toward their true width. Safe here because Maze4 has no floating
# walls (ENABLE_FLOATING_WALL_MAPPING=False); for a floating-wall maze set this back == 2.5.
LOG_ODDS_FREE_UPDATE_CAP = 2  # cells below this erode freely; old self-thinning cap (was 2.5/3.5 ratchet). was 2
LOG_ODDS_MIN = -5.0             # lower clamp for log-odds grid
LOG_ODDS_MAX = 5.0              # upper clamp for log-odds grid
# Raised 2.0 → 2.5 (Maze4 tuning): requires 4 hits to confirm a wall cell
# (3×0.8=2.4 is NOT >2.5; 4×0.8=3.2 IS). LiDAR range noise spreads hits
# over a 2–3 cell band; the 3-hit rule was confirming those edge-noise cells
# and making walls appear 2–3× thicker than reality, narrowing A* corridors.
# 4 hits still confirms every real wall (20+ LiDAR rays hit a wall per scan).
# FREE_UPDATE_CAP raised to match so the invariant holds.
LOG_ODDS_WALL_THRESHOLD = 1   # confirm wall easily (~2 hits); pairs w/ cap=2.0 self-thinning (was 3.2)
LOG_ODDS_FREE_THRESHOLD = -0.5  # log-odds below this → cell is free
LOG_ODDS_VIRTUAL_OBS = 5.0      # log-odds value stamped for injected virtual obstacles

# --- MAP: NOISE FILTERING & INFLATION ---
MIN_WALL_BLOB_AREA = 4          # old value (was 8)
INFLATION_RADIUS = 4            # old working value (was 5); Maze1 corridors plan at 4 first try.
                               # With the single-radius fallback below, this is the ONLY radius,
                               # so "reduced inflation" / crawl never triggers.
LIDAR_UPDATE_STRIDE = 3         # every Nth LiDAR point used in map update (reverted: 2 caused noise explosion)
# Gap-closing: before planning, morphologically CLOSE the wall mask to seal gaps narrower
# than the robot (~6 px = 0.19 m). LiDAR noise / unmapped slivers leave 1–3 px holes in
# walls; A* would thread a path through such a FALSE opening, the robot drives there, then
# discovers it's solid and wastes time. Sealing sub-robot-width gaps removes those false
# passages while keeping real (≥6 px) corridors open. Kernel 5 closes gaps up to ~5 px.
GAP_CLOSE_KERNEL = 1            # DISABLED (was 3). Its dilation was the main wall-fattener;
                               # turning it off lets walls hold their true mapped width.

# --- PATH PLANNING: FALLBACK STRATEGY ---
# When A* fails at full inflation, retry with a smaller radius — but NEVER below the
# robot's actual half-width (~3 px = 0.10 m). At 2 px (0.067 m) or 0 px the path centreline
# is closer to the wall than the robot's own body, so the robot CANNOT follow it without
# clipping the wall (→ collision → wheel slip → odometry/pillar-dot corruption). Bottoming
# out at 3 px means A* either returns a physically drivable path or fails and the robot
# reroutes. Gap-closing (above) already removes sub-robot false passages, so 3–4 px covers
# every real corridor.
# Earlier values that produced clipping paths: (4, 2, 0) and (4, 3, 2, 1, 0).
ASTAR_INFLATION_FALLBACK = (4,2)          # SINGLE radius like old code → path_inflation always == INFLATION_RADIUS,
                                         # so the "reduced inflation" crawl branch never fires (full-speed or DETOUR).
SNAP_TO_FREE_SEARCH_RADIUS = 15        # pixel search radius to pull start/goal out of inflated walls

# --- MAP: FRONTIER EXPLORATION ---
MIN_FRONTIER_AREA = 15          # minimum frontier cluster size to be considered valid
FRONTIER_DILATE_ITER = 2        # dilation iterations when computing frontier mask





#Floating wall



# --- DEPTH CAMERA: FLOATING WALL SLICE ---
DEPTH_SAMPLE_STRIDE = 8         # column stride when sampling depth slice

FLOATING_WALL_MAX_RANGE = 2.0  # ignore depth readings beyond this for wall mapping (m)  value was 1,2




# --- DEPTH CAMERA: HEIGHT-FILTERED OBSTACLE MAPPING ---
# Instead of mapping every depth point in a fixed row band, we compute each point's REAL
# height above the ground and only map it if it lies in the robot's vertical envelope.
#   height = CAMERA_MOUNT_HEIGHT + depth * sin( atan( (CAMERA_CY - row) / CAMERA_FX ) )
# This does two things at once:
#   • FLOOR (height ≈ 0) is excluded → open ground is never mapped as a phantom wall.
#   • HIGH floating planks (e.g. the elevated plank guarding YELLOW in Maze3) are excluded
#     → the short robot correctly treats the space under them as drivable.
# A normal wall still maps because its LOWER body (0.07–0.20 m) sits inside the envelope.
CAMERA_MOUNT_HEIGHT     = 0.165  # depth-cam optical-centre height above ground (m) — from Rosbot proto
CAMERA_CY               = 240.0  # vertical principal point (px) = image height / 2 (480/2)
OBSTACLE_MIN_HEIGHT     = 0.07   # ignore points below this height (m) → floor / ground-noise
ROBOT_PASS_UNDER_HEIGHT = 0.20   # ignore points above this height (m) → robot drives UNDER them
DEPTH_HEIGHT_ROW_START  = 120    # top row scanned for height-filtered obstacles
DEPTH_HEIGHT_ROW_END    = 420    # bottom row scanned
DEPTH_HEIGHT_ROW_STRIDE = 12      # row stride when scanning (paired with DEPTH_SAMPLE_STRIDE columns)

# --- DEPTH CAMERA: "LiDAR-MISS = FLOATING" NORMAL-WALL SUPPRESSION ---
# The depth camera used to re-stamp NORMAL walls the LiDAR already maps 360°; that redundant
# double-mapping (plus odometry smear) fattened walls over time. A normal wall is hit by
# BOTH sensors; a FLOATING plank the LiDAR beam passes under is hit ONLY by the depth cam.
# When True, BOTH depth channels (geometric obstacle points AND the wood colour stamper)
# keep a point only if there is NO LiDAR return near it (robot-local frame) — so only
# genuine floating walls get depth-stamped; normal walls are left to the LiDAR alone.
# Set False to restore the original stamp-everything behaviour.
DEPTH_MAP_ONLY_LIDAR_MISSES = True
# Neighbourhood (cells) within which a LiDAR return counts as "covering" a depth point.
# Absorbs the LiDAR↔depth-cam mount offset + noise. Too small → normal walls leak through;
# too big → a plank hugging a real wall is suppressed (harmless — cell already occupied).
DEPTH_LIDAR_MATCH_CELLS = 3  #   was 3

# --- DEPTH CAMERA: OBSTACLE CHECK PATCH ---
DEPTH_PATCH_SIZE = 4            # half-size of pixel patch used in get_depth_at_pixel
DEPTH_CENTER_ROW_MIN_FRAC = 0.4 # centre patch vertical bounds (fraction of height)
DEPTH_CENTER_ROW_MAX_FRAC = 0.6
DEPTH_CENTER_COL_MIN_FRAC = 0.45
DEPTH_CENTER_COL_MAX_FRAC = 0.55

# --- DEPTH CAMERA: WIDE FRONT OBSTACLE CHECK ---
# The narrow centre patch (45-55% width, 40-60% height) misses floating walls
# that are slightly off-centre. This wider scan covers the central 80% of width
# and upper 67% of image height, used as a secondary close-range trigger.
DEPTH_FRONT_WIDE_ROW_FRAC  = 0.67  # scan rows 0 → 67% of image height
DEPTH_FRONT_WIDE_COL_MIN   = 0.10  # leftmost column fraction
DEPTH_FRONT_WIDE_COL_MAX   = 0.90  # rightmost column fraction
FLOATING_WALL_CLOSE_RANGE  = 0.35  # trigger escape if wide scan sees anything within this distance (m)

# --- LIDAR: OBSTACLE CHECK ---
LIDAR_FRONT_HALF_ANGLE = 0.35   # half-angle of front cone for lidar obstacle check (rad)

# --- PID CONTROLLER ---
PID_KP = 6.0
PID_KI = 0.1
PID_KD = 1.5
PID_INTEGRAL_MAX = 2.0          # anti-windup clamp on integral term

# --- PATH PLANNING ---
# Raised 10 → 15 (Maze4): replanning every 10 steps on a live-updating map made the
# path jump frame-to-frame (the "fluctuating" look). The route blue→yellow is stable,
# so a slightly longer interval holds one path longer = steadier following, while still
# replanning ~3×/sec to react to newly-mapped obstacles.
ASTAR_REPLAN_INTERVAL = 15      # replan A* every Nth main-loop step
# Unknown-space cost: cells with grid value 127 (unexplored) get this extra cost per step.
# Makes A* prefer known-free corridors over uncharted territory where a floating wall
# might be lurking. A path through unknown space still works if no known-free route exists.
UNKNOWN_SPACE_PENALTY = 2.0     # extra A* cost per step in unknown (gray) cells
ASTAR_GOAL_RADIUS = 6           # A* terminates when within this many pixels of goal
PLANNER_SAFE_DISTANCE_PX = 6.0  # clearance penalty applied within this pixel radius of walls
PLANNER_PENALTY_STRENGTH = 5.0  # weight of clearance penalty in A* cost

# --- PATH SMOOTHING (B-SPLINE) ---
# Lowered 0.2 → 0.08 (Maze4): s_val = smoothness × path_len. On the long blue→yellow
# route (~120 cells) 0.2 gave s≈24 — so much smoothing that the spline overshot the
# zig-zag's sharp reversals into visible LOOPS (the "random/fluctuating path"). 0.08
# tracks the raw A* path closely → no loops, stable shape between replans. Strictly
# SAFER for every maze: the raw A* path is collision-free, and lower smoothing can only
# hug it tighter (it's HIGHER smoothing that rounds corners into walls).
BSPLINE_SMOOTHNESS = 0.08       # smoothness factor passed to splprep
MIN_PATH_SMOOTH_LEN = 15        # raw path shorter than this is returned unsmoothed
MIN_SMOOTH_SAMPLES = 10         # minimum number of output samples from splev
SMOOTH_SAMPLE_DENSITY = 3       # output samples = input points × this factor

# --- PURE PURSUIT ---
PURE_PURSUIT_LOOKAHEAD = 15     # number of path points to look ahead (open space)
# In a tight corridor a far lookahead point sits PAST the next bend, so the robot shortcuts
# diagonally across the inside corner — straight into the wall (the Maze4 zig-zag hits). Use
# a much shorter lookahead on reduced-inflation paths so the robot hugs the centreline
# through each bend instead of cutting it.
TIGHT_PATH_LOOKAHEAD = 7        # lookahead points when the path used inflation < INFLATION_RADIUS
MIN_SPEED_FACTOR = 0.4          # minimum speed scaling on sharp turns
# PIVOT-IN-PLACE threshold (rad). When the heading error to the lookahead point exceeds
# this, the path follower sets forward speed to 0 and ROTATES IN PLACE to align before
# driving. A skid-steer robot rotates about its centre, so turning while moving forward
# swings the front corner outward into the rung tip (the Maze4 "clips every edge" bug).
# Pivoting first means the robot only translates when roughly aligned → corner stays clear.
# 0.6 rad ≈ 34°. Lower = pivots more often (safer corners, slower); higher = clips return.
PIVOT_IN_PLACE_ANGLE = 0.49
BASE_NAV_SPEED = 6.0           # base wheel speed for normal navigation
# Set 3.0 → 0.0 to KILL the blind-reverse bug. The no-path fallback in
# follow_path_steering is `return -SEARCH_TURN_SPEED, -SEARCH_TURN_SPEED`, i.e. BOTH
# wheels backward = a straight reverse with no rear-collision check — the robot backed
# into walls until it hit something (wheel slip → odometry drift → map collapse).
# With 0.0 the fallback returns (0, 0): the robot HOLDS STILL when it has no path and
# keeps replanning, instead of reversing. (This constant is used at exactly one place.)
SEARCH_TURN_SPEED = 0.0         # 0 = hold still when no path exists (was a buggy blind reverse)

# --- APPROACH SPEED REDUCTION ---
# When the robot is within APPROACH_SLOW_RADIUS metres of its target (blue or yellow
# pillar) it reduces speed by APPROACH_SPEED_FACTOR.  The slower approach gives the
# LiDAR/depth map more time to accumulate the required 3 log-odds hits on any
# floating wall that lies between the robot and the pillar, so the wall is confirmed
# and A* routes around it before the robot gets close enough to hit it.




APPROACH_SLOW_RADIUS = 1.0      # metres — start slowing down at this distance increse to 2 if lidar or depth cam miss the wall scan
APPROACH_SPEED_FACTOR = 0.55    # multiply BASE_NAV_SPEED by this inside slow radius

# When A* could only find a path at reduced inflation (a TIGHT pinch, e.g. a gap by a
# floating wall), the path runs close to walls. Crawl through it so the PID tracks the
# centreline precisely and the robot doesn't overshoot into the wall.
# Raised 0.5 → 0.70: 0.5 made the robot crawl even on long straight sections
# that happened to use 3px/4px inflation (common near the junction). 0.70 still
# slows down relative to full-inflation paths, but doesn't kill progress speed.
# TIGHT_PATH_LOOKAHEAD=7 handles corner precision; the speed doesn't need to.
TIGHT_PATH_SPEED_FACTOR = 0.9  # belt+suspenders: never apply a tight-path speed cut (was 0.70)




# --- ESCAPE MANEUVERS ---
ESCAPE_BACKUP_SPEED = 4.0       # wheel speed magnitude during backup
ESCAPE_TURN_SPEED = 4.0         # wheel speed magnitude during escape turn
ESCAPE_BACKUP_STEPS = 20        # maximum backup steps before turning (rear LiDAR/sensors stop early)
ESCAPE_TURN_STEPS = 15          # steps spent turning during escape

GREEN_ESCAPE_SPEED = 6.0        # spin speed for green-ground 180° escape
GREEN_ESCAPE_STEPS = 35         # steps for green-ground 180° spin





# --- GREEN GROUND: ACCURATE PERCEPTION-BASED MAPPING ---
# Old approach stamped ONE fixed block GREEN_WALL_DIST_AHEAD metres ahead at a fixed
# GREEN_WALL_BLOCK_SIZE. That only worked for Maze1's single patch. Other mazes have
# many green patches of different sizes/positions. We now DETECT each green blob with
# the camera, measure its real area, find its true world position via the depth camera,
# and stamp a virtual obstacle that matches the patch's actual location and size.
GREEN_MIN_BLOB_PIXELS  = 120    # ignore green blobs smaller than this (camera px) → kills specks/noise
# Lowered 2.5 → 1.2 (Maze4 junction fix): green is on the FLOOR, so its depth reading is a
# SLANT distance that OVER-projects the patch farther than it really is. At 2.5 m that smear
# was wide enough to seal the bottom-corridor↔zig-zag JUNCTION as a solid (sticky) obstacle,
# so A* found NO route blue→yellow ("completely blocked"). Only mapping green within 1.2 m —
# where the slant error is small and localisation is accurate — stamps the patch's true
# footprint without smearing across the corridor. The 0.60 m imminent-escape still protects


# green


# the robot from driving onto green it gets close to.
GREEN_MAP_MAX_RANGE    = 1.5   # only map green within this depth (m); beyond, floor depth is unreliable value was 1.2
GREEN_IMMINENT_RANGE   = 0.60   # trigger 180° escape if green centred ahead is closer than this (m)
GREEN_AHEAD_COL_FRAC   = 0.25   # a sample counts as "ahead" if its column is within ±this×width of centre
VIRTUAL_OBS_MERGE_DIST = 8      # px: merge repeated virtual-obstacle detections within this distance (dedup)


# Multi-point sampling: instead of one centroid square per blob we sample many points
# ACROSS each green blob, project each to the world via the depth camera, and stamp a
# small cell at each. This maps arbitrary / elongated patch shapes accurately, so A*
# routes around the patch's real footprint rather than an over-/under-sized square.
GREEN_SAMPLE_STRIDE    = 6      # sample every Nth green pixel within a blob (lower = denser, slower)
GREEN_POINT_RADIUS_PX  = 1      # half-size (px) of the map cell stamped per sampled green point

# --- FLOATING WALL: COLOUR + MULTI-POINT MAPPING (same idea as green paint) ---
# A long floating plank only partly fits the depth camera's FOV, and per-column sampling
# maps it as a thin/sparse line A* can thread. We instead colour-segment the WOOD planks,
# multi-point sample the whole silhouette, project each point with the depth camera, keep
# the ones in the robot's BLOCKING height envelope (floor + high pass-under planks excluded
# by the same height test used for depth obstacles), and stamp them SOLID + immediately.
# Colour confirmation makes the instant dense stamp safe (rejects depth noise / non-walls).
FLOATING_MIN_BLOB_PIXELS = 100  # ignore wood blobs smaller than this (px) → noise
FLOATING_SAMPLE_STRIDE   = 8    # sample every Nth wood pixel within a blob (walls are big → sparser)
# PER-MAZE TOGGLE. ON for mazes with ELEVATED planks LiDAR can't see (Maze3). OFF for mazes
# whose walls are all floored (Maze4/Maze5): there it only over-thickens walls (depth-error
# offset, sticky) — squeezing narrow corridors — and slows the sim (heavy wood-blob scan).
ENABLE_FLOATING_WALL_MAPPING = True

# --- STUCK DETECTION ---
# Robot is "stuck" if it moves less than STUCK_DISTANCE_THRESHOLD metres every
# STUCK_CHECK_INTERVAL steps. After STUCK_TRIGGER_COUNT consecutive stuck periods
# a deeper escape is forced (longer backup + more turn steps).
STUCK_CHECK_INTERVAL    = 50    # check progress every N main-loop steps (~1.6 s at 32 ms)
STUCK_DISTANCE_THRESHOLD = 0.08 # must move at least 8 cm per interval or count as stuck
STUCK_TRIGGER_COUNT      = 3    # consecutive stuck intervals before forced deep escape

# Deeper escape used when stuck is confirmed (more backup to clear tight spaces)
ESCAPE_BACKUP_STEPS_DEEP = 45
ESCAPE_TURN_STEPS_DEEP   = 40

# --- FAST WEDGE DETECTION ---
# Detect a physical wedge FAST (vs the 5 s STUCK detection above) by comparing how far the
# robot was COMMANDED to drive forward vs how far it actually moved. Using the forward
# command (left+right)/2 naturally ignores in-place pivots (where forward command ≈ 0), so
# this only fires when the robot is genuinely jammed while trying to translate.
# On a wedge we: mark the blockage just ahead (so A* routes around what the sensors couldn't
# fully map — e.g. an under-mapped floating wall), back out, and replan. Fully general.
# PER-MAZE TOGGLE. ON for mazes where the robot can get truly jammed at an under-mapped
# obstacle (Maze3 floating wall). OFF for mazes with legitimate TIGHT corridors (Maze4):
# there a slow crawl is normal, so the detector false-fires and stamps a blockage marker
# INSIDE the only corridor — sealing the path. With it off, the robot just crawls through;
# real jams still fall back to the slower STUCK detection + collision escape.
ENABLE_WEDGE_DETECTION = False
WEDGE_CHECK_INTERVAL = 12       # check every N steps (~0.4 s) — much faster than STUCK detection
# WEDGE_MIN_EXPECTED set to 9999: the condition in the main loop is
#   "if expected > WEDGE_MIN_EXPECTED and ..."
# A robot moving at max speed for one check interval covers at most
#   26 rad/s × 0.043 m × 12 × 0.032 s ≈ 0.43 m — nowhere near 9999.
# So this condition is PERMANENTLY False: the inner block (inject + escape)
# can never execute regardless of what the controller code does.
# Belt+suspenders: WEDGE_PROGRESS_FRAC=0.0 makes "moved < 0" also False.
WEDGE_MIN_EXPECTED   = 9999.0   # DISABLED — expected>9999 is always False
WEDGE_PROGRESS_FRAC  = 0.0      # DISABLED — moved<0 is always False
WEDGE_MARK_DIST      = 0.18     # inject the blockage marker this far ahead of the robot (m)
WEDGE_MARK_SIZE      = 0.16     # side length of the injected wedge marker (m)

# --- STATE MACHINE ---
WARMUP_STEPS = 10               # simulation steps run before odometry calibration
# One-time 360° pillar scan at mission start. The camera only sees ~60° forward, so a
# pillar BEHIND the start position (e.g. YELLOW in Maze2/Maze4) is never seen while the
# robot drives to BLUE — forcing a slow, noisy search after reaching BLUE. Spinning in
# place once at startup pre-locates every pillar visible from the start, so the robot
# knows the whole mission upfront and routes straight to YELLOW the moment BLUE is done.
SCAN_SPIN_SPEED = 3.0           # wheel speed (per side, opposite signs) for the startup spin
SCAN_MAX_STEPS  = 250           # safety cap on scan duration (~8 s); a full turn is ~150 steps
BLUE_PAUSE_STEPS = 62           # steps robot pauses at blue pillar (~2 s at 32 ms)
WAYPOINT_REACHED_RADIUS = 0.4   # distance threshold to consider a search waypoint reached (m)
PILLAR_REACHED_RADIUS = 0.40   # distance threshold to consider a pillar reached (m)
RANDOM_WP_RANGE = 4.0           # ±range for random fallback waypoint (m)

# --- ROBOT FOOTPRINT FREE-CARVING ---
# The robot CANNOT physically be inside a wall, so the cells under its body are
# guaranteed free. Every map update we stamp a free disc of this radius at the robot's
# position. As it drives, the union of these discs carves a guaranteed-passable corridor
# along its whole trajectory — so A* can always retrace where the robot has already been.
# This fixes the Maze4 "junction sealed behind me" bug: green/plank stamps landing BESIDE
# the driven path can no longer pinch the mapped corridor below robot width.
#   radius 3 px = the robot's actual HALF-WIDTH (body is ~6 px / 0.20 m). The disc stays
#   INSIDE the corridor the robot is physically driving, so it can NEVER erase a real wall
#   the robot's body didn't touch — while still freeing the full driven path for A*.
#   (radius 4 = 9 px was too wide: it bit ~1-2 px into the narrow zig-zag walls each side,
#   creating false openings → robot drove into the real wall → collision spiral.)
ROBOT_FREE_CARVE_PX = 2
# Value the carved cells are set to. NOT the hard floor (LOG_ODDS_MIN = -5): that takes
# ~10 LiDAR hits to rebuild, so a cell wrongly carved (e.g. on odometry drift) stays a
# phantom opening the robot drives into. A mild free value self-corrects: a REAL wall under
# the carve rebuilds in a few hits, while the junction's mis-stamped FLOOR (no LiDAR hits)
# stays cleared. Must be < LOG_ODDS_FREE_THRESHOLD so the cell renders/treats as free.
ROBOT_FREE_CARVE_VALUE = -2.0

# --- DISPLAY ---
DISPLAY_WINDOW_SIZE = 600       # pygame window side length (pixels)
DISPLAY_UPDATE_INTERVAL = 3     # render pygame display every Nth main-loop step


# ============================================================================
# STARTUP BANNER — prints the moment the controller imports constants.py.
# constants.py is NEVER open in the Webots editor, so this CANNOT be overwritten:
# if you see this block at sim start with the values below, the fixes are loaded.
# It also prints the ONE check for the .py file (which the Webots editor CAN revert):
# after "Reached BLUE", you must NOT see "360 scan" a second time.
# ============================================================================
print("=" * 64)
print("  MAZE4 TUNING ACTIVE  (constants.py loaded — cannot be reverted)")
print("  ----------------------------------------------------------------")
print(f"   purple-box/wedge : DISABLED   (WEDGE_MIN_EXPECTED={WEDGE_MIN_EXPECTED})")
print(f"   no-path action   : {'STOP (good)' if SEARCH_TURN_SPEED == 0.0 else 'REVERSE (BUG!)'}   (SEARCH_TURN_SPEED={SEARCH_TURN_SPEED})")
print(f"   floating wall detection  : {ENABLE_FLOATING_WALL_MAPPING} and  {FLOATING_WALL_MAX_RANGE}")
print(f"   bspline smooth   : {BSPLINE_SMOOTHNESS}   replan every {ASTAR_REPLAN_INTERVAL} steps")
print(f"   wall threshold   : {LOG_ODDS_WALL_THRESHOLD}   erode cap {LOG_ODDS_FREE_UPDATE_CAP} (walls self-thin)")
print(f"   footprint carve  : {ROBOT_FREE_CARVE_PX} px @ {ROBOT_FREE_CARVE_VALUE} (safe: won't erode zig-zag walls)")
print(f"   approach-via BLUE  : {BLUE_APPROACH_VIA}   YELLOW : {YELLOW_APPROACH_VIA}")

print("  ----------------------------------------------------------------")
print("   .py CHECK: after 'Reached BLUE' you must NOT see a 2nd '360 scan'.")
print("   If you do → Webots editor reverted the .py (close tab / File>Revert).")
print("=" * 64)



