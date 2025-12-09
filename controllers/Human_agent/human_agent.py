from controller import Robot
import math
import os
import sys

THIS_DIR = os.path.dirname(__file__)
CTRL_DIR = os.path.dirname(THIS_DIR)
if CTRL_DIR not in sys.path:
    sys.path.append(CTRL_DIR)

from lib_shared.local_planner import DWA

LEFT = "left wheel motor"
RIGHT = "right wheel motor"
R = 0.033
L = 0.160

LIDAR_NAME = "LDS-01"
IMU_NAME = "inertial unit"
COMPASS_NAME = "compass"
GPS_NAME = "gps"

WAYPOINTS = [
    (1.40, 0.20),
    (1.40, 1.10),
    (0.10, 1.10),
    (-1.00, 1.10),
    (-1.00, 0.20),
    (0.10, 0.20),
]


def dev(robot, name):
    try:
        return robot.getDevice(name)
    except Exception:
        return None


def yaw_from(imu, compass):
    """
    Return yaw (heading) in radians.
    Prefer the compass (more stable for planar robots),
    fall back to IMU if no compass is available.
    """
    # Prefer compass: vector points towards north; yaw=0 along +x
    if compass:
        try:
            n = compass.getValues()  # [x, y, z]
            # Use same convention as your other controllers: atan2(x, z)
            return math.atan2(n[0], n[2])
        except Exception:
            pass

    # Fallback: inertial unit roll/pitch/yaw
    if imu:
        try:
            roll, pitch, yaw = imu.getRollPitchYaw()
            # Sometimes yaw can be NaN or inf during start-up
            if math.isfinite(yaw):
                return yaw
        except Exception:
            pass

    # Last resort
    return 0.0


def world_to_robot(dx, dz, yaw):
    gx = dx * math.cos(-yaw) - dz * math.sin(-yaw)
    gy = dx * math.sin(-yaw) + dz * math.cos(-yaw)
    return gx, gy


def main():
    robot = Robot()
    ts = int(robot.getBasicTimeStep())

    lm = dev(robot, LEFT)
    rm = dev(robot, RIGHT)
    lm.setPosition(float("inf"))
    rm.setPosition(float("inf"))
    lm.setVelocity(0.0)
    rm.setVelocity(0.0)

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

    dwa = DWA(
        {
            "V_MAX": 0.30,
            "SAFE": 0.08,
            "CLEAR_N": 0.6,
            "GAMMA": 0.22,
            "BETA": 0.34,
            "ALPHA": 0.18,
            "DELTA": 0.46,
            "EPS": 0.05,
        }
    )

    idx = 0
    prev = (0.0, 0.0)
    smooth = 0.15

    last_move_time = 0.0
    last_x = None
    last_z = None
    recovery_until = 0.0
    recovery_sign = 1.0

    while robot.step(ts) != -1:
        if not gps or not lidar:
            lm.setVelocity(0.0)
            rm.setVelocity(0.0)
            continue

        now = robot.getTime()
        x, _, z = gps.getValues()
        yaw = yaw_from(imu, compass)

        tx, tz = WAYPOINTS[idx]
        dx = tx - x
        dz = tz - z
        if math.hypot(dx, dz) < 0.22:
            idx = (idx + 1) % len(WAYPOINTS)
            continue

        ranges = lidar.getRangeImage()
        hfov = lidar.getFov()
        nscan = lidar.getHorizontalResolution()
        pts = []
        if nscan > 1:
            for i, r in enumerate(ranges):
                if r == float("inf") or r <= 0.04:
                    continue
                a = -hfov * 0.5 + hfov * i / (nscan - 1)
                x_o = r * math.cos(a)
                y_o = r * math.sin(a)
                pts.append((x_o, y_o))

        if last_x is None:
            last_x, last_z = x, z
            last_move_time = now
        else:
            dist = math.hypot(x - last_x, z - last_z)
            if dist > 0.05:
                last_x, last_z = x, z
                last_move_time = now
            elif now - last_move_time > 3.0 and recovery_until <= now:
                recovery_sign *= -1.0
                recovery_until = now + 2.0

        gx, gy = world_to_robot(dx, dz, yaw)

        if recovery_until > now:
            v = -0.18
            w = 1.2 * recovery_sign
        else:
            v_cmd, w_cmd = dwa.get_safe_velocities(pts, (gx, gy), prev_cmd=prev, cur=prev)
            v = smooth * prev[0] + (1.0 - smooth) * v_cmd
            w = smooth * prev[1] + (1.0 - smooth) * w_cmd

        max_v = 0.26
        max_w = 1.4
        v = max(-max_v, min(max_v, v))
        w = max(-max_w, min(max_w, w))

        prev = (v, w)

        wl = (v - 0.5 * w * L) / R
        wr = (v + 0.5 * w * L) / R

        max_wheel = min(lm.getMaxVelocity(), rm.getMaxVelocity())
        wl = max(-max_wheel, min(max_wheel, wl))
        wr = max(-max_wheel, min(max_wheel, wr))

        lm.setVelocity(wl)
        rm.setVelocity(wr)


if __name__ == "__main__":
    main()
