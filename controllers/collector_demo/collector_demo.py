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

    prev = (0.0, 0.0)
    smooth_gain = 0.6

    speed_window = deque(maxlen=25)
    avg_threshold = 0.02
    last_unstuck = -10.0
    unstuck_cooldown = 1.5
    turn_sign = 1.0

    while robot.step(ts) != -1:
        ranges = lidar.getRangeImage()
        hfov = lidar.getFov()
        nscan = lidar.getHorizontalResolution()

        obs = []
        a_best = 0.0
        best_score = -1.0
        max_consider = 1.2

        min_front = float("inf")
        front_left = 0.0
        front_right = 0.0
        front_width = math.radians(60.0)

        if nscan > 1:
            for i, r in enumerate(ranges):
                if r == float("inf") or r <= 0.04:
                    continue
                a = -hfov * 0.5 + hfov * (i / (nscan - 1))
                x_obs = r * math.cos(a)
                y_obs = r * math.sin(a)
                obs.append((x_obs, y_obs))

                if abs(a) <= math.radians(90.0):
                    score = min(r, max_consider)
                    if score > best_score:
                        best_score = score
                        a_best = a

                if abs(a) < front_width * 0.5:
                    if r < min_front:
                        min_front = r
                    val = max(0.0, 0.5 - r)
                    if a >= 0.0:
                        front_left += val
                    else:
                        front_right += val

        front_block = min_front < 0.32

        if best_score < 0.0:
            r_goal = 0.6
            a_goal = 0.0
        else:
            r_goal = min(best_score, 0.6)
            a_goal = a_best

        gx = r_goal * math.cos(a_goal)
        gy = r_goal * math.sin(a_goal)

        if front_block:
            if front_left > front_right:
                turn_sign = -1.0
            elif front_right > front_left:
                turn_sign = 1.0
            v = -0.10
            w = 0.9 * turn_sign
            v_cmd = v
            w_cmd = w
        else:
            v_cmd, w_cmd = dwa.get_safe_velocities(
                obs, (gx, gy), prev_cmd=prev, cur=prev
            )
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
            if front_left > front_right:
                turn_sign = -1.0
            elif front_right > front_left:
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
                f"[DWA] v={v:.2f} w={w:.2f} wl={wl:.2f} wr={wr:.2f} "
                f"front={min_front:.2f} obs={len(obs)}"
            )


if __name__ == "__main__":
    main()
