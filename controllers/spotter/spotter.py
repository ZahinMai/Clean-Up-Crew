# Spotter controller (Setup 3)
# Responsible for finding trash using an efficient search pattern.
# Uses: navigation, obstacle_avoidance, vision, communication
# Implemented by: Abdullateef Vahora

from controller import Robot

import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))
sys.path.append(parent_dir)

from lib_shared import vision

print("Spotter controller initializing")

# Setup 
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Enable the camera
try:
    camera = robot.getDevice('camera')
    camera.enable(timestep)
    width = camera.getWidth()
    height = camera.getHeight()
    print(f"Camera found. Dimensions: {width}x{height}")
except Exception as e:
    print(f"Error initializing camera: {e}")
    camera = None

# Main Loop
while robot.step(timestep) != -1:
    if camera:
        trash_spotted = vision.is_trash_visible(camera)

        print(f"Vision library check: {trash_spotted}")
    else:
        print("Camera not found, cannot run vision check.")