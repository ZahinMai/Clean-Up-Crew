# controllers/collector_demo/collector_demo.py
# TurtleBot3 Burger + DWA local planner demo (Webots)
# Includes smoothing (anti-oscillation) and stuck-recovery.

from controller import Robot
import math, os, sys
from collections import deque

# ----- allow importing ../lib_shared -----------------------------------------
THIS_DIR = os.path.dirname(__file__)
CTRL_DIR = os.path.dirname(THIS_DIR)               # .../controllers
if CTRL_DIR not in sys.path:
    sys.path.append(CTRL_DIR)
# -----------------------------------------------------------------------------

from lib_shared.local_planner import DWA

# ---------- device names (must match your robot) -----------------------------
LEFT_MOTOR   = 'left wheel motor'
RIGHT_MOTOR  = 'right wheel motor'
LIDAR_NAME   = 'LDS-01'          # under extensionSlot; rename here if different
IMU_NAME     = 'inertial unit'   # under extensionSlot
COMPASS_NAME = 'compass'         # optional fallback (add if you like)
GPS_NAME     = 'gps'             # you added this under extensionSlot
# -----------------------------------------------------------------------------

# ---------- robot geometry (TB3 Burger approx) -------------------------------
WHEEL_RADIUS = 0.045
AXLE_LENGTH  = 0.160
# -----------------------------------------------------------------------------

# ---------- helpers ----------------------------------------------------------
def get_device(robot, name):
    try:
        return robot.getDevice(name)
    except:
        return None

def get_yaw(imu, compass):
    """Return yaw [rad] using IMU if available, compass otherwise, else 0."""
    if imu:
        r, p, y = imu.getRollPitchYaw()
        return y
    if compass:
        n = compass.getValues()  # [x, y, z]
        return math.atan2(n[0], n[2])
    return 0.0

def world_to_robot(dx, dz, yaw):
    """Transform world-frame goal offset (dx,dz) to robot frame (gx,gy)."""
    gx =  dx*math.cos(-yaw) - dz*math.sin(-yaw)
    gy =  dx*math.sin(-yaw) + dz*math.cos(-yaw)
    return gx, gy
# -----------------------------------------------------------------------------


def main():
    robot = Robot()
    ts = int(robot.getBasicTimeStep())

    # --- wheels ---
    lm = get_device(robot, LEFT_MOTOR)
    rm = get_device(robot, RIGHT_MOTOR)
    lm.setPosition(float('inf')); rm.setPosition(float('inf'))
    lm.setVelocity(0.0); rm.setVelocity(0.0)

    # --- sensors (robust enable) ---
    imu = get_device(robot, IMU_NAME)
    if imu: imu.enable(ts)
    compass = get_device(robot, COMPASS_NAME)
    if compass: compass.enable(ts)
    gps = get_device(robot, GPS_NAME); gps.enable(ts)
    lidar = get_device(robot, LIDAR_NAME); lidar.enable(ts)

    # --- DWA (tunable params) ---
    dwa = DWA({
        # keep conservative for tight cafeteria aisles
        "V_MAX":   0.25,  # m/s cap (helps stay under wheel limits)
        "SAFE":    0.10,  # hard min clearance after inflation (see local_planner)
        "CLEAR_N": 1.00,  # clearance normalization (1.0 m -> full credit)
        "GAMMA":   0.50,  # clearance weight
        "BETA":    0.35,  # goal progress
        "ALPHA":   0.30,  # heading alignment
        "EPS":     0.10   # smoothness (higher = less jerky)
    })

    # smoothing state
    prev = (0.0, 0.0)
    SMOOTH = 0.6  # 0..1 (higher = smoother, slower to react)

    # stuck detection state
    SPEED_WINDOW = deque(maxlen=20)  # ~1.5–2s history (depends on timestep)
    AVG_THRESHOLD = 0.02             # m/s; below this average we consider "stuck"
    last_unstuck = -10.0
    UNSTUCK_COOLDOWN = 1.5           # seconds of sim time between spins

    # --- simple patrol waypoints (x,z in world meters) ---
    # Put these in open aisles of your cafeteria (adjust to your layout).
    WAYPOINTS = [
        (0.90, 0.00),
        (0.90, 0.90),
        (-0.80, 0.90),
        (-0.80, 0.00),
    ]
    wp_i = 0

    while robot.step(ts) != -1:
        # Pose
        x, _, z = gps.getValues()
        yaw = get_yaw(imu, compass)

        # Waypoint management
        tx, tz = WAYPOINTS[wp_i]
        dx, dz = (tx - x), (tz - z)
        if math.hypot(dx, dz) < 0.18:
            wp_i = (wp_i + 1) % len(WAYPOINTS)
            continue

        # Lidar → (x,y) obstacle points in robot frame
        ranges = lidar.getRangeImage()
        hfov   = lidar.getFov()
        nscan  = lidar.getHorizontalResolution()
        obs = []
        for i, r in enumerate(ranges):
            if r == float('inf') or r <= 0.03:
                continue
            a = -hfov/2 + hfov * (i / (nscan - 1))
            obs.append((r*math.cos(a), r*math.sin(a)))

        # Goal in robot frame
        gx, gy = world_to_robot(dx, dz, yaw)

        # DWA (raw)
        v, w = dwa.get_safe_velocities(obs, (gx, gy), prev_cmd=prev, cur=prev)

        # --- Anti-oscillation smoothing -------------------------------------
        v = SMOOTH * prev[0] + (1.0 - SMOOTH) * v
        w = SMOOTH * prev[1] + (1.0 - SMOOTH) * w
        prev = (v, w)
        # ---------------------------------------------------------------------

        # --- Stuck detection & recovery --------------------------------------
        # track commanded linear speed magnitude
        SPEED_WINDOW.append(abs(v))
        now = robot.getTime()
        stuck = (len(SPEED_WINDOW) == SPEED_WINDOW.maxlen and
                 sum(SPEED_WINDOW) / len(SPEED_WINDOW) < AVG_THRESHOLD)

        if stuck and (now - last_unstuck) > UNSTUCK_COOLDOWN:
            # rotate gently in place to search for a free direction
            v = 0.00
            w = 0.5  # rad/s; 0.3..0.8 depending on clutter
            prev = (v, w)
            last_unstuck = now
        # ---------------------------------------------------------------------

        # Convert (v,w) -> wheel angular velocities
        wl = (v - 0.5*w*AXLE_LENGTH) / WHEEL_RADIUS
        wr = (v + 0.5*w*AXLE_LENGTH) / WHEEL_RADIUS

        # Clamp to motor max to avoid warnings
        max_w = min(lm.getMaxVelocity(), rm.getMaxVelocity())  # TB3 ~6.67 rad/s
        wl = max(-max_w, min(max_w, wl))
        wr = max(-max_w, min(max_w, wr))

        lm.setVelocity(wl)
        rm.setVelocity(wr)

        # Debug (≈1 Hz)
        if int(robot.getTime()) % 1 == 0:
            print(f"[DWA] wp={wp_i} v={v:.2f} w={w:.2f} wl={wl:.2f} wr={wr:.2f} obs={len(obs)}")

if __name__ == "__main__":
    main()
