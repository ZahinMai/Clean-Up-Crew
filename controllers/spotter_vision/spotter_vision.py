# Vision Proof of Concept
# Demonstrates basic vision capabilities for real world applications.
# Implemented by: Abdullateef Vahora

from controller import Robot
import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))
sys.path.append(parent_dir)
from lib_shared import vision

print("Vision Demo Initialising")

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
    rubbish_spotted = vision.is_rubbish_visible(camera)

    print(f"[Vision] Is rubbish in field of view: {rubbish_spotted}")