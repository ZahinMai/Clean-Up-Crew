# Human_agent controller
# using shared DWA for obstacle avoidance.
# Folder name:  Human_agent
# File name:    Human_agent.py

from controller import Robot
import math
import os
import sys


# Import shared local planner (DWA)
THIS_DIR = os.path.dirname(__file__)
CTRL_DIR = os.path.dirname(THIS_DIR)
if CTRL_DIR not in sys.path:
    sys.path.append(CTRL_DIR)

from lib_shared.local_planner import DWA  # noqa: E402

# Robot / device names (TurtleBot3Burger defaults)
LEFT_MOTOR = "left wheel motor"
RIGHT_MOTOR = "right wheel motor"

R = 0.033   # wheel radius
L = 0.160   # axle length

LIDAR_NAME = "LDS-01"
IMU_NAME = "inertial unit"
COMPASS_NAME = "compass"
GPS_NAME = "gps"

# Simple rectangular patrol loop for the "human"
WAYPOINTS = [
    (1.40, 0.20),
    (1.40, 1.10),
    (0.10, 1.10),
    (-1.00, 1.10),
    (-1.00, 0.20),
    (0.10, 0.20),
]

# Small helpers
def dev(robot: Robot, name: str):
    """Safe device getr (returns None if missing)."""
    try:
        return robot.getDevice(name)
    except Exception:
        return None


def yaw_from(imu, compass) -> float:
    """
    Return yaw (heading) in radians.
    Prefer the compass 
    fall back to IMU if needed.
    """
    # Compass: vector towards north; yaw = 0 along +x
    if compass:
        try:
            n = compass.getValues()  # [x, y, z]
            return math.atan2(n[0], n[2])
        except Exception:
            pass

    # Fallback: IMU roll/pitch/yaw
    if imu:
        try:
            _, _, yaw = imu.getRollPitchYaw()
            if math.isfinite(yaw):
                return yaw
        except Exception:
            pass

    return 0.0


def world_to_robot(dx: float, dz: float, yaw: float):
    """Transform (dx, dz) in world frame into robot frame (gx, gy)."""
    gx = dx * math.cos(-yaw) - dz * math.sin(-yaw)
    gy = dx * math.sin(-yaw) + dz * math.cos(-yaw)
    return gx, gy


def get_lidar_points(lidar, max_r=3.0, min_r=0.08, downsample=3):
    """
    Convert lidar scan to a small set of (x, y) points in robot frame.
    downsample: only every Nth ray is used to keep DWA cheap.
    """
    if not lidar:
        return []

    ranges = lidar.getRangeImage()
    if not ranges:
        return []

    fov = lidar.getFov()
    n = len(ranges)
    if n < 2:
        return []

    pts = []
    for i in range(0, n, downsample):
        r = ranges[i]
        if not math.isfinite(r) or r < min_r or r > max_r:
            continue

        a = -fov * 0.5 + fov * i / (n - 1)
        x_o = r * math.cos(a)
        y_o = r * math.sin(a)
        # only keep points in front half-plane
        if x_o > -0.02:
            pts.append((x_o, y_o))

    return pts

# Main controller
def main():
    robot = Robot()
    ts = int(robot.getBasicTimeStep())

    # Motors
    lm = dev(robot, LEFT_MOTOR)
    rm = dev(robot, RIGHT_MOTOR)
    lm.setPosition(float("inf"))
    rm.setPosition(float("inf"))
    lm.setVelocity(0.0)
    rm.setVelocity(0.0)

    # Sensors
    imu = dev(robot, IMU_NAME)
    if imu:
        imu.enable(ts)

    compass = dev(robot, COMPASS_NAME)
    if compass:
        compass.enable(ts)

    gps = dev(robot, GPS_NAME)
    if gps:
        gps.enable(ts)

    lidar = dev(robot, LIDAR_NAME)
    if lidar:
        lidar.enable(ts)

    #  Lighter DWA configuration for "human" agent 
    dwa = DWA(
        {
            # Search / prediction parameters (reduced for speed)
            "T_PRED": 1.2,   # shorter prediction horizon
            "DT_SIM": 0.08,
            "NV": 5,
            "NW": 9,
            # Motion limits
            "V_MAX": 0.30,
            "W_MAX": 1.6,
            # Safety
            "SAFE": 0.08,
            "CLEAR_N": 0.6,
            # Cost weights (slightly goal-biased)
            "ALPHA": 0.22,   # heading
            "BETA": 0.34,    # progress
            "GAMMA": 0.22,   # clearance
            "DELTA": 0.40,   # speed
            "EPS": 0.05,     # smoothness
        }
    )

    # Patrol state
    wp_idx = 0
    prev_cmd = (0.0, 0.0)
    smooth = 0.2  # command smoothing factor

    # Stuck recovery
    last_move_time = 0.0
    last_x = None
    last_z = None
    recovery_until = 0.0
    recovery_sign = 1.0

    while robot.step(ts) != -1:
        # Basic safety: if GPS or lidar not ready, stop.
        if not gps or not lidar:
            lm.setVelocity(0.0)
            rm.setVelocity(0.0)
            continue

        now = robot.getTime()
        x, _, z = gps.getValues()
        yaw = yaw_from(imu, compass)

        #  Patrol waypoint logic 
        tx, tz = WAYPOINTS[wp_idx]
        dx = tx - x
        dz = tz - z
        if math.hypot(dx, dz) < 0.22:
            wp_idx = (wp_idx + 1) % len(WAYPOINTS)
            continue

        # Lidar processing (down-sampled) 
        lidar_pts = get_lidar_points(lidar)

        # Stuck detection (very light)
        if last_x is None:
            last_x, last_z = x, z
            last_move_time = now
        else:
            dist = math.hypot(x - last_x, z - last_z)
            if dist > 0.05:
                last_x, last_z = x, z
                last_move_time = now
            elif now - last_move_time > 3.0 and recovery_until <= now:
                # Flip recovery spin direction
                recovery_sign *= -1.0
                recovery_until = now + 2.0

        #Goal in robot frame 
        gx, gy = world_to_robot(dx, dz, yaw)

        #Command selection 
        if recovery_until > now:
            # Simple escape manoeuvre if stuck
            v = -0.18
            w = 1.0 * recovery_sign
        else:
            v_cmd, w_cmd = dwa.get_safe_velocities(
                lidar_points=lidar_pts,
                goal_vec=(gx, gy),
                prev_cmd=prev_cmd,
                cur=prev_cmd,
            )
            # Smooth commands to avoid jitter and CPU spikes
            v = smooth * prev_cmd[0] + (1.0 - smooth) * v_cmd
            w = smooth * prev_cmd[1] + (1.0 - smooth) * w_cmd

        # Clamp to TurtleBot limits
        max_v = 0.26
        max_w = 1.4
        v = max(-max_v, min(max_v, v))
        w = max(-max_w, min(max_w, w))

        prev_cmd = (v, w)

        # Convert (v, w) -> wheel speeds
        wl = (v - 0.5 * w * L) / R
        wr = (v + 0.5 * w * L) / R

        wheel_limit = min(lm.getMaxVelocity(), rm.getMaxVelocity())
        wl = max(-wheel_limit, min(wheel_limit, wl))
        wr = max(-wheel_limit, min(wheel_limit, wr))

        lm.setVelocity(wl)
        rm.setVelocity(wr)


if __name__ == "_main_":
    main()

