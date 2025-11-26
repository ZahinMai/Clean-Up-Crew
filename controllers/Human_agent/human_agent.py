from controller import Robot
import math

LEFT = "left wheel motor"
RIGHT = "right wheel motor"
R = 0.033
L = 0.160

WAYPOINTS = [
    (1.40, 0.20),
    (1.40, 1.10),
    (0.10, 1.10),
    (-1.00, 1.10),
    (-1.00, 0.20),
    (0.10, 0.20),
]

SPEED = 0.18
WGAIN = 1.8
ARRIVE = 0.12

LIDAR_NAME = "LDS-01"
FRONT_WIDTH = math.radians(60.0)
AVOID_DIST = 0.35


def main():
    robot = Robot()
    ts = int(robot.getBasicTimeStep())

    lm = robot.getDevice(LEFT)
    rm = robot.getDevice(RIGHT)
    lm.setPosition(float("inf"))
    rm.setPosition(float("inf"))
    lm.setVelocity(0.0)
    rm.setVelocity(0.0)

    gps = robot.getDevice("gps") if robot.getDevice("gps") else None
    if gps:
        gps.enable(ts)
    compass = robot.getDevice("compass") if robot.getDevice("compass") else None
    if compass:
        compass.enable(ts)
    imu = (
        robot.getDevice("inertial unit")
        if robot.getDevice("inertial unit")
        else None
    )
    if imu:
        imu.enable(ts)
    lidar = robot.getDevice(LIDAR_NAME) if robot.getDevice(LIDAR_NAME) else None
    if lidar:
        lidar.enable(ts)

    def yaw():
        if imu:
            _, _, y = imu.getRollPitchYaw()
            return y
        if compass:
            n = compass.getValues()
            return math.atan2(n[0], n[2])
        return 0.0

    i = 0
    turn_sign = 1.0

    while robot.step(ts) != -1:
        if gps:
            x, _, z = gps.getValues()
        else:
            x, z = 0.0, 0.0

        tx, tz = WAYPOINTS[i]
        dx = tx - x
        dz = tz - z
        if math.hypot(dx, dz) < ARRIVE:
            i = (i + 1) % len(WAYPOINTS)
            continue

        th = yaw()
        gth = math.atan2(dz, dx)
        e = (gth - th + math.pi) % (2 * math.pi) - math.pi
        v = SPEED
        w = WGAIN * e

        front_min = float("inf")
        left_sum = 0.0
        right_sum = 0.0

        if lidar:
            ranges = lidar.getRangeImage()
            hfov = lidar.getFov()
            nscan = lidar.getHorizontalResolution()
            if nscan > 1:
                for idx, r in enumerate(ranges):
                    if r == float("inf") or r <= 0.04:
                        continue
                    a = -hfov * 0.5 + hfov * (idx / (nscan - 1))
                    if abs(a) < FRONT_WIDTH * 0.5:
                        if r < front_min:
                            front_min = r
                        val = max(0.0, AVOID_DIST - r)
                        if a >= 0.0:
                            left_sum += val
                        else:
                            right_sum += val

        block_front = front_min < AVOID_DIST

        if block_front:
            if left_sum > right_sum:
                turn_sign = -1.0
            elif right_sum > left_sum:
                turn_sign = 1.0
            v = -0.10
            w = 0.9 * turn_sign

        wl = (v - 0.5 * w * L) / R
        wr = (v + 0.5 * w * L) / R
        max_w = min(lm.getMaxVelocity(), rm.getMaxVelocity())
        wl = max(-max_w, min(max_w, wl))
        wr = max(-max_w, min(max_w, wr))
        lm.setVelocity(wl)
        rm.setVelocity(wr)


if __name__ == "__main__":
    main()
