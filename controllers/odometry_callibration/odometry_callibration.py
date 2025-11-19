import math
import sys

from controller import Robot

# Import odometry modules
from odometry import (sOdometryTrack, odometry_track_init, 
                      odometry_track_start_pos, odometry_track_step_pos)
from odometry_goto import (sOdometryGoto, odometry_goto_init, 
                           odometry_goto_start, odometry_goto_set_goal, 
                           odometry_goto_step)

# Global defines
PI = math.pi
TRUE = 1
FALSE = 0
NO_SIDE = -1
LEFT = 0
RIGHT = 1
WHITE = 0
BLACK = 1
SIMULATION = 0
REALITY = 2
TIME_STEP = 128  # [ms]

# Variables specific to the simulation
VERBOSE = 1
INCR = 10
MIN_SPEED = 20
MAX_SPEED = 200
INCREMENT_TEST = 10000
RATIO_TEST = 14300
SCALING_TEST = 0.3

# Wheel parameters
SPEED_UNIT = 0.00628
ENCODER_UNIT = 159.23

# Global variables
left_encoder_offset = 0.0
right_encoder_offset = 0.0
speed = [0, 0]
pspeed = [0, 0]
cpt = 0

# Instantiate odometry track and goto structures
ot = sOdometryTrack()
og = sOdometryGoto()

# Device references
left_motor = None
right_motor = None
left_position_sensor = None
right_position_sensor = None
robot = None


def step():
    if robot.step(TIME_STEP) == -1:
        sys.exit(0)


def init():
    global speed, pspeed
    speed[LEFT] = speed[RIGHT] = 0
    pspeed[LEFT] = pspeed[RIGHT] = 0
    left_motor.setVelocity(SPEED_UNIT * speed[LEFT])
    right_motor.setVelocity(SPEED_UNIT * speed[RIGHT])
    print("Init OK")


def print_position():
    print(f"Current position : ({ot.result.x:.6f},{ot.result.y:.6f},{ot.result.theta:.6f})")


def set_speed(l, r):
    global speed, pspeed
    pspeed[LEFT] = speed[LEFT]
    pspeed[RIGHT] = speed[RIGHT]
    speed[LEFT] = l
    speed[RIGHT] = r
    
    if pspeed[LEFT] != speed[LEFT] or pspeed[RIGHT] != speed[RIGHT]:
        left_motor.setVelocity(SPEED_UNIT * speed[LEFT])
        right_motor.setVelocity(SPEED_UNIT * speed[RIGHT])


def goto_position(x, y, theta):
    if VERBOSE > 0:
        print(f"Going to ({x}, {y}, {theta})")
    
    odometry_goto_set_goal(og, x, y, theta) # set goal
    
    # Move until goal is reached
    while og.result.atgoal == 0:
        # Update position
        left_pos = ENCODER_UNIT * (left_position_sensor.getValue() - left_encoder_offset)
        right_pos = ENCODER_UNIT * (right_position_sensor.getValue() - right_encoder_offset)
        odometry_track_step_pos(ot, left_pos, right_pos)
        odometry_goto_step(og) # compute motor speeds
        
        set_speed(og.result.speed_left, og.result.speed_right) # set speeds
        
        if VERBOSE > 1: print_position()
        
        step()
    
    if VERBOSE > 0: print_position()


def test_increment(val):
    global left_encoder_offset, right_encoder_offset
    # reset offsets
    left_encoder_offset = left_position_sensor.getValue()
    right_encoder_offset = right_position_sensor.getValue()
    init()
    step()
    # Linear speed increase over 100 increments
    while (ENCODER_UNIT * (left_position_sensor.getValue() - left_encoder_offset) < 100 or
           ENCODER_UNIT * (right_position_sensor.getValue() - right_encoder_offset) < 100):
        left_enc = ENCODER_UNIT * (left_position_sensor.getValue() - left_encoder_offset)
        right_enc = ENCODER_UNIT * (right_position_sensor.getValue() - right_encoder_offset)
        s = int(MAX_SPEED * (left_enc + right_enc) / 200)
        s = MIN_SPEED if s < MIN_SPEED else s
        s = MAX_SPEED if s > MAX_SPEED else s
        set_speed(s, s)
        step()

    # Max speed
    while (ENCODER_UNIT * (left_position_sensor.getValue() - left_encoder_offset) < val - 400 and
           ENCODER_UNIT * (right_position_sensor.getValue() - right_encoder_offset) < val - 400):
        set_speed(MAX_SPEED, MAX_SPEED)
        step()

    # Linear speed decrease over 400 increments
    while (ENCODER_UNIT * (left_position_sensor.getValue() - left_encoder_offset) < val or
           ENCODER_UNIT * (right_position_sensor.getValue() - right_encoder_offset) < val):
        left_enc = ENCODER_UNIT * (left_position_sensor.getValue() - left_encoder_offset)
        right_enc = ENCODER_UNIT * (right_position_sensor.getValue() - right_encoder_offset)
        s = int(MAX_SPEED * (val - (left_enc + right_enc) / 2) / 400)
        s = MIN_SPEED if s < MIN_SPEED else s
        s = MAX_SPEED if s > MAX_SPEED else s
        set_speed(s, s)
        step()

    print("INCREMENT test finished")
    init()



def test_ratio(val):
    global left_encoder_offset, right_encoder_offset
    init()
    
    # Linear speed increase over 100 increments
    while True:
        left_enc = ENCODER_UNIT * (left_position_sensor.getValue() - left_encoder_offset)
        right_enc = ENCODER_UNIT * (right_position_sensor.getValue() - right_encoder_offset)
        if left_enc <= -100 and right_enc >= 100:
            break
        
        s = int(MAX_SPEED * (-left_enc + right_enc) / 200)
        s = MIN_SPEED if s < MIN_SPEED else s
        s = MAX_SPEED if s > MAX_SPEED else s
        set_speed(-s, s)
        step()
    
    # Max speed rotation
    while True:
        left_enc = ENCODER_UNIT * (left_position_sensor.getValue() - left_encoder_offset)
        right_enc = ENCODER_UNIT * (right_position_sensor.getValue() - right_encoder_offset)
        if left_enc <= -val + 400 and right_enc >= val - 400:
            break
        set_speed(-MAX_SPEED, MAX_SPEED)
        step()
    
    # Linear speed decrease over 400 increments
    while True:
        left_enc = ENCODER_UNIT * (left_position_sensor.getValue() - left_encoder_offset)
        right_enc = ENCODER_UNIT * (right_position_sensor.getValue() - right_encoder_offset)
        if left_enc <= -val or right_enc >= val:
            break
        
        s = int(MAX_SPEED * (val - (-left_enc + right_enc) / 2) / 400)
        s = MIN_SPEED if s < MIN_SPEED else s
        s = MAX_SPEED if s > MAX_SPEED else s
        set_speed(-s, s)
        step()
    
    print("RATIO test finished")
    init()


def test_diameter():
    init()
    step()
    
    goto_position(0, 0.1, PI)
    goto_position(-0.1, 0.1, -PI / 2)
    goto_position(-0.1, 0, 0)
    goto_position(0, 0, PI / 2)
    
    print("DIAMETER test finished")
    init()


def test_scaling():
    global left_encoder_offset, right_encoder_offset
    init()
    
    left_pos = ENCODER_UNIT * (left_position_sensor.getValue() - left_encoder_offset)
    right_pos = ENCODER_UNIT * (right_position_sensor.getValue() - right_encoder_offset)
    odometry_track_step_pos(ot, left_pos, right_pos)
    
    while True:
        left_enc = ENCODER_UNIT * (left_position_sensor.getValue() - left_encoder_offset)
        right_enc = ENCODER_UNIT * (right_position_sensor.getValue() - right_encoder_offset)
        odometry_track_step_pos(ot, left_enc, right_enc)
        
        if left_enc >= 100 and right_enc >= 100:
            break
        
        s = int(MAX_SPEED * (left_enc + right_enc) / 200)
        s = MIN_SPEED if s < MIN_SPEED else s
        s = MAX_SPEED if s > MAX_SPEED else s
        set_speed(s, s)
        step()
    
    # Max speed
    while ot.result.y < SCALING_TEST - 0.05:
        left_pos = ENCODER_UNIT * (left_position_sensor.getValue() - left_encoder_offset)
        right_pos = ENCODER_UNIT * (right_position_sensor.getValue() - right_encoder_offset)
        odometry_track_step_pos(ot, left_pos, right_pos)
        set_speed(MAX_SPEED, MAX_SPEED)
        step()
    
    # Linear speed decrease over 400 increments
    while ot.result.y < SCALING_TEST:
        left_pos = ENCODER_UNIT * (left_position_sensor.getValue() - left_encoder_offset)
        right_pos = ENCODER_UNIT * (right_position_sensor.getValue() - right_encoder_offset)
        odometry_track_step_pos(ot, left_pos, right_pos)
        
        s = int(MAX_SPEED * (SCALING_TEST - ot.result.y) / 0.05)
        s = MIN_SPEED if s < MIN_SPEED else s
        s = MAX_SPEED if s > MAX_SPEED else s
        set_speed(s, s)
        step()
    
    print_position()
    print("SCALING test finished")
    init()


def main():
    global robot, left_motor, right_motor, left_position_sensor, right_position_sensor
    global left_encoder_offset, right_encoder_offset
    
    # Initialize robot
    robot = Robot()
    
    # Enable keyboard
    keyboard = robot.getKeyboard()
    keyboard.enable(TIME_STEP)
    
    # Get motor devices and set to velocity control
    left_motor = robot.getDevice("left wheel motor")
    right_motor = robot.getDevice("right wheel motor")
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    
    # Get position sensors and enable them
    left_position_sensor = robot.getDevice("left wheel sensor")
    right_position_sensor = robot.getDevice("right wheel sensor")
    left_position_sensor.enable(TIME_STEP)
    right_position_sensor.enable(TIME_STEP)
    
    # Required to get position sensor values
    robot.step(TIME_STEP)
    
    # Initialize the modules
    odometry_track_init()
    odometry_goto_init()
    
    # Initialize tracking and goto structures
    left_pos = ENCODER_UNIT * (left_position_sensor.getValue() - left_encoder_offset)
    right_pos = ENCODER_UNIT * (right_position_sensor.getValue() - right_encoder_offset)
    odometry_track_start_pos(ot, left_pos, right_pos)
    odometry_goto_start(og, ot)
    
    ot.result.x = 0.0
    ot.result.y = 0.0
    ot.result.theta = PI / 2
    
    init()
    
    # Main loop
    while robot.step(TIME_STEP) != -1:
        step()
        key = keyboard.getKey()
        l = speed[LEFT]
        r = speed[RIGHT]
        
        # Handle keyboard input
        if key == keyboard.UP:
            if speed[LEFT] < MAX_SPEED:
                l += INCR
            if speed[RIGHT] < MAX_SPEED:
                r += INCR
        elif key == keyboard.DOWN:
            if speed[LEFT] > -MAX_SPEED:
                l -= INCR
            if speed[RIGHT] > -(MAX_SPEED + 1):
                r -= INCR
        elif key == keyboard.LEFT:
            if speed[RIGHT] < MAX_SPEED:
                r += INCR
            if speed[LEFT] > -(MAX_SPEED + 1):
                l -= INCR
        elif key == keyboard.RIGHT:
            if speed[LEFT] < MAX_SPEED:
                l += INCR
            if speed[RIGHT] > -(MAX_SPEED + 1):
                r -= INCR
        elif key == ord('1'):
            test_increment(INCREMENT_TEST)
        elif key == ord('2'):
            test_ratio(int(RATIO_TEST))
        elif key == ord('3'):
            test_diameter()
        elif key == ord('4'):
            test_scaling()
        elif key == ord('S'):
            l = r = 0
        elif key == ord('R'):
            left_encoder_offset = left_position_sensor.getValue()
            right_encoder_offset = right_position_sensor.getValue()
            left_pos = ENCODER_UNIT * (left_position_sensor.getValue() - left_encoder_offset)
            right_pos = ENCODER_UNIT * (right_position_sensor.getValue() - right_encoder_offset)
            odometry_track_start_pos(ot, left_pos, right_pos)
        elif key == ord('P'):
            print_position()
        
        # Update odometry
        left_pos = ENCODER_UNIT * (left_position_sensor.getValue() - left_encoder_offset)
        right_pos = ENCODER_UNIT * (right_position_sensor.getValue() - right_encoder_offset)
        odometry_track_step_pos(ot, left_pos, right_pos)
        
        # Send speed values to motors
        set_speed(l, r)


if __name__ == "__main__":
    main()