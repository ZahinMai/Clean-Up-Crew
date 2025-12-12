# Vision Proof of Concept
# Demonstrates basic vision capabilities for real world applications.
# Implemented by: Abdullateef Vahora

from controller import Supervisor, Keyboard # Changed Robot to Supervisor
import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))
sys.path.append(parent_dir)
from lib_shared import vision

print("Vision Demo Initialising")

# Setup 
robot = Supervisor() # Must be Supervisor to use setLabel
timestep = int(robot.getBasicTimeStep())

# 1. Enable Keyboard
keyboard = robot.getKeyboard()
keyboard.enable(timestep)

# 2. Setup Motors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

MAX_SPEED = 6.28

# Enable camera
try:
    camera = robot.getDevice('camera')
    camera.enable(timestep)
except Exception as e:
    print(f"Error initializing camera: {e}")
    camera = None

# Main Loop
while robot.step(timestep) != -1:
    rubbish_spotted = vision.is_rubbish_visible(camera)
    
    label_text = f"Rubbish Visible: {rubbish_spotted}"
    label_color = int(0x00FF00 if rubbish_spotted else 0xFF0000) # Green if True, Red if False
    
    robot.setLabel(0, label_text, 0.5, 0.5, 0.1, label_color, 0.0, "Arial")

    # Rotation Logic
    key = keyboard.getKey()
    left_speed = 0.0
    right_speed = 0.0

    if key == Keyboard.LEFT:
        left_speed = -MAX_SPEED * 0.5
        right_speed = MAX_SPEED * 0.5
    elif key == Keyboard.RIGHT:
        left_speed = MAX_SPEED * 0.5
        right_speed = -MAX_SPEED * 0.5
    
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)