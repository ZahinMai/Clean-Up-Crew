from controller import Robot
import math
import os
import sys
from collections import deque

THIS_DIR = os.path.dirname(__file__)
CTRL_DIR = os.path.dirname(THIS_DIR)
if CTRL_DIR not in sys.path:
    sys.path.append(CTRL_DIR)

from lib_shared.local_planner import DWA

LEFT_MOTOR = "left wheel motor"
RIGHT_MOTOR = "right wheel motor"
LIDAR_NAME = "LDS-01"
IMU_NAME = "inertial unit"
COMPASS_NAME = "compass"
GPS_NAME = "gps"

WHEEL_RADIUS = 0.033
AXLE_LENGTH = 0.160


def get_device(robot, name):
    try:
        return robot.getDevice(name)
    except Exception:
        return None


def get_yaw(imu, compass):
    if imu:
        _, _, y = imu.getRollPitchYaw()
        return y
    if compass:
        n = compass.getValues()
        return math.atan2(n[0], n[2])
    return 0.0


def world_to_robot(dx, dz, yaw):
    gx = dx * math.cos(-yaw) - dz * math.sin(-yaw)
    gy = dx * math.sin(-yaw) + dz * math.cos(-yaw)
    return gx, gy


def main():
    robot = Robot()
    ts = int(robot.getBasicTimeStep())

    lm = get_device(robot, LEFT_MOTOR)
    rm = get_device(robot, RIGHT_MOTOR)
    lm.setPosition(float("inf"))
    rm.setPosition(float("inf"))
    lm.setVelocity(0.0)
    rm.setVelocity(0.0)

    imu = get_device(robot, IMU_NAME)
    if imu:
        imu.enable(ts)
    compass = get_device(robot, COMPASS_NAME)
    if compass:
        compass.enable(ts)
    gps = get_device(robot, GPS_NAME)
    if gps:
        gps.enable(ts)
    lidar = get_device(robot, LIDAR_NAME)
    if lidar:
        lidar.enable(ts)

    dwa = DWA(
        {
            "V_MAX": 0.20,
            "SAFE": 0.10,
            "CLEAR_N": 0.8,
            "GAMMA": 0.60,
            "BETA": 0.35,
            "ALPHA": 0.30,
            "EPS": 0.12,
        }
    )

    waypoints = [
        (0.90, 0.00),
        (0.90, 0.90),
        (-0.80, 0.90),
        (-0.80, 0.00),
    ]
    wp_idx = 0

    prev = (0.0, 0.0)
    smooth_gain = 0.65

    speed_window = deque(maxlen=25)
    avg_threshold = 0.02
    last_unstuck = -10.0
    unstuck_cooldown = 1.5
    turn_sign = 1.0

    while robot.step(ts) != -1:
        if not gps or not lidar:
            lm.setVelocity(0.0)
            rm.setVelocity(0.0)
            continue

        x, _, z = gps.getValues()
        yaw = get_yaw(imu, compass)

        tx, tz = waypoints[wp_idx]
        dx = tx - x
        dz = tz - z
        if math.hypot(dx, dz) < 0.20:
            wp_idx = (wp_idx + 1) % len(waypoints)
            continue

        ranges = lidar.getRangeImage()
        hfov = lidar.getFov()
        nscan = lidar.getHorizontalResolution()

        obs = []
        min_front = float("inf")
        left_score = 0.0
        right_score = 0.0
        front_width = math.radians(60.0)
        avoid_dist = 0.35

        if nscan > 1:
            for i, r in enumerate(ranges):
                if r == float("inf") or r <= 0.04:
                    continue
                a = -hfov * 0.5 + hfov * (i / (nscan - 1))
                x_obs = r * math.cos(a)
                y_obs = r * math.sin(a)
                obs.append((x_obs, y_obs))
                if abs(a) < front_width * 0.5:
                    if r < min_front:
                        min_front = r
                    val = max(0.0, avoid_dist - r)
                    if a >= 0.0:
                        left_score += val
                    else:
                        right_score += val

        gx, gy = world_to_robot(dx, dz, yaw)

        front_block = min_front < avoid_dist

        if front_block and gx > 0.0:
            if left_score > right_score:
                turn_sign = -1.0
            elif right_score > left_score:
                turn_sign = 1.0
            v = -0.10
            w = 0.9 * turn_sign
        else:
            v_cmd, w_cmd = dwa.get_safe_velocities(obs, (gx, gy), prev_cmd=prev, cur=prev)
            v = smooth_gain * prev[0] + (1.0 - smooth_gain) * v_cmd
            w = smooth_gain * prev[1] + (1.0 - smooth_gain) * w_cmd

        max_v = 0.12
        max_w = 0.9
        if v > max_v:
            v = max_v
        if v < -max_v:
            v = -max_v
        if w > max_w:
            w = max_w
        if w < -max_w:
            w = -max_w

        speed_window.append(abs(v))
        now = robot.getTime()
        stuck = (
            len(speed_window) == speed_window.maxlen
            and sum(speed_window) / len(speed_window) < avg_threshold
        )

        if stuck and (now - last_unstuck) > unstuck_cooldown:
            if left_score > right_score:
                turn_sign = -1.0
            elif right_score > left_score:
                turn_sign = 1.0
            v = -0.10
            w = 0.9 * turn_sign
            last_unstuck = now
            speed_window.clear()

        prev = (v, w)

        wl = (v - 0.5 * w * AXLE_LENGTH) / WHEEL_RADIUS
        wr = (v + 0.5 * w * AXLE_LENGTH) / WHEEL_RADIUS

        max_wheel = min(lm.getMaxVelocity(), rm.getMaxVelocity())
        if wl > max_wheel:
            wl = max_wheel
        if wl < -max_wheel:
            wl = -max_wheel
        if wr > max_wheel:
            wr = max_wheel
        if wr < -max_wheel:
            wr = -max_wheel

        lm.setVelocity(wl)
        rm.setVelocity(wr)

        if int(robot.getTime() * 2.0) % 4 == 0:
            print(
                f"[collector DWA] wp={wp_idx} v={v:.2f} w={w:.2f} wl={wl:.2f} wr={wr:.2f} obs={len(obs)}"
            )


if __name__ == "__main__":
    main()

