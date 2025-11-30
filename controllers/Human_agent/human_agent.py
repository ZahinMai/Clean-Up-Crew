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

    lm = get_device(robot, LEFT)
    rm = get_device(robot, RIGHT)
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
            "V_MAX": 0.18,
            "SAFE": 0.10,
            "CLEAR_N": 0.8,
            "GAMMA": 0.55,
            "BETA": 0.35,
            "ALPHA": 0.30,
            "EPS": 0.10,
        }
    )

    idx = 0
    prev = (0.0, 0.0)
    smooth_gain = 0.6

    while robot.step(ts) != -1:
        if not gps or not lidar:
            lm.setVelocity(0.0)
            rm.setVelocity(0.0)
            continue

        x, _, z = gps.getValues()
        yaw = get_yaw(imu, compass)

        tx, tz = WAYPOINTS[idx]
        dx = tx - x
        dz = tz - z
        if math.hypot(dx, dz) < 0.18:
            idx = (idx + 1) % len(WAYPOINTS)
            continue

        ranges = lidar.getRangeImage()
        hfov = lidar.getFov()
        nscan = lidar.getHorizontalResolution()
        obs = []
        if nscan > 1:
            for i, r in enumerate(ranges):
                if r == float("inf") or r <= 0.04:
                    continue
                a = -hfov * 0.5 + hfov * (i / (nscan - 1))
                x_obs = r * math.cos(a)
                y_obs = r * math.sin(a)
                obs.append((x_obs, y_obs))

        gx, gy = world_to_robot(dx, dz, yaw)

        v_cmd, w_cmd = dwa.get_safe_velocities(obs, (gx, gy), prev_cmd=prev, cur=prev)

        v = smooth_gain * prev[0] + (1.0 - smooth_gain) * v_cmd
        w = smooth_gain * prev[1] + (1.0 - smooth_gain) * w_cmd

        max_v = 0.12
        max_w = 0.8
        if v > max_v:
            v = max_v
        if v < -max_v:
            v = -max_v
        if w > max_w:
            w = max_w
        if w < -max_w:
            w = -max_w

        prev = (v, w)

        wl = (v - 0.5 * w * L) / R
        wr = (v + 0.5 * w * L) / R

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
                f"[human DWA] wp={idx} v={v:.2f} w={w:.2f} wl={wl:.2f} wr={wr:.2f} obs={len(obs)}"
            )


if __name__ == "__main__":
    main()
