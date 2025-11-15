from controller import Robot
import math

LEFT  = 'left wheel motor'
RIGHT = 'right wheel motor'
R = 0.033     # wheel radius
L = 0.160     # axle length

# a simple, predictable loop the DWA bot must avoid
WAYPOINTS = [
    ( 1.40,  0.20),
    ( 1.40,  1.20),
    ( 0.00,  1.20),
    (-1.10,  1.20),
    (-1.10,  0.20),
    ( 0.00,  0.20),
]

SPEED = 0.18  # m/s
WGAIN = 1.8   # turn gain
ARRIVE = 0.12

def main():
    robot = Robot()
    ts = int(robot.getBasicTimeStep())

    lm = robot.getDevice(LEFT); rm = robot.getDevice(RIGHT)
    lm.setPosition(float('inf')); rm.setPosition(float('inf'))
    lm.setVelocity(0); rm.setVelocity(0)

    gps = robot.getDevice('gps') if robot.getDevice('gps') else None
    if gps: gps.enable(ts)
    compass = robot.getDevice('compass') if robot.getDevice('compass') else None
    if compass: compass.enable(ts)
    imu = robot.getDevice('inertial unit') if robot.getDevice('inertial unit') else None
    if imu: imu.enable(ts)

    # crude yaw estimate: prefer imu, else compass, else dead-reckon heading=0
    def yaw():
        if imu:
            _,_,y = imu.getRollPitchYaw(); return y
        if compass:
            n = compass.getValues(); return math.atan2(n[0], n[2])
        return 0.0

    i = 0
    while robot.step(ts) != -1:
        # world pose
        if gps: x,_,z = gps.getValues()
        else:   x,z = 0.0,0.0  # not used for control if GPS absent

        tx,tz = WAYPOINTS[i]
        dx, dz = tx - x, tz - z
        if math.hypot(dx,dz) < ARRIVE:
            i = (i+1) % len(WAYPOINTS)
            continue

        # heading control toward next waypoint
        th = yaw()
        gth = math.atan2(dz, dx)
        e = (gth - th + math.pi) % (2*math.pi) - math.pi  # wrap
        v = SPEED
        w = WGAIN * e

        wl = (v - 0.5*w*L) / R
        wr = (v + 0.5*w*L) / R
        max_w = min(lm.getMaxVelocity(), rm.getMaxVelocity())
        wl = max(-max_w, min(max_w, wl))
        wr = max(-max_w, min(max_w, wr))
        lm.setVelocity(wl); rm.setVelocity(wr)

if __name__ == "__main__":
    main()
