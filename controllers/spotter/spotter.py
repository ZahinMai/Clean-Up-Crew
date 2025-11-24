# Spotter controller (Setup 3)
# Responsible for finding trash using an efficient search pattern.
# Uses: navigation, obstacle_avoidance, vision, communication
# Implemented by: Abdullateef Vahora

from controller import Robot
from collections import deque
import math
import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from lib_shared.global_planner import AStarPlanner
from lib_shared.map_module import get_map
from lib_shared.local_planner import DWA
from lib_shared import vision

MAP_ORIGIN_X = -6.1
MAP_ORIGIN_Y = -2.8   

LIDAR_NAME   = "lidar"
GPS_NAME     = "gps"
COMPASS_NAME = "compass"
CAMERA_NAME  = "camera"

WHEEL_RADIUS = 0.0205 
AXLE_LENGTH  = 0.052  
MAX_WHEEL_W  = 6.28   
# ==========================================

def get_yaw(compass):
    """
    Returns yaw in radians.
    """
    if not compass: return 0.0
    vals = compass.getValues()
    
    return math.atan2(vals[0], vals[1])

def world_to_robot(dx, dy, yaw):
    """
    Transform world-frame goal offset (dx, dy) to robot frame (gx, gy).
    """
    gx = dx * math.cos(-yaw) - dy * math.sin(-yaw)
    gy = dx * math.sin(-yaw) + dy * math.cos(-yaw)
    return gx, gy

def lidar_to_cartesian(lidar_data, fov_rad): 
    points = []
    if not lidar_data or len(lidar_data) < 2: return points

    angle_step = fov_rad / (len(lidar_data) - 1)
    current_angle = -fov_rad / 2.0
    
    SELF_COLLISION_RADIUS = 0.05 #Ignores its own body points
    
    for r in lidar_data:
        if SELF_COLLISION_RADIUS < r < 2.0: 
            
            lx = r * math.cos(current_angle)
            ly = r * math.sin(current_angle)
            points.append((lx, ly))
            
        current_angle += angle_step
    return points

def generate_lawnmower_path(grid, step=4):
    waypoints = []
    for c in range(1, grid.width - 1, step):
        row_indices = range(1, grid.height - 1, step)
        if (c // step) % 2 == 1:
            row_indices = reversed(row_indices) 
        for r in row_indices:
            if grid.is_free(r, c):
                waypoints.append((r, c))
    return waypoints

def main():
    print("Initializing Spotter Bot...")
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    # --- 1. Hardware ---
    gps = robot.getDevice(GPS_NAME)
    gps.enable(timestep)
    compass = robot.getDevice(COMPASS_NAME)
    compass.enable(timestep)
    camera = robot.getDevice(CAMERA_NAME)
    camera.enable(timestep)
    
    lidar = robot.getDevice(LIDAR_NAME)
    lidar_fov = 4.18
    if lidar:
        lidar.enable(timestep); lidar.enablePointCloud()
        lidar_fov = lidar.getFov()
    
    lm = robot.getDevice('left wheel motor')
    rm = robot.getDevice('right wheel motor')
    if not lm: lm = robot.getDevice('left wheel')
    if not rm: rm = robot.getDevice('right wheel')
    lm.setPosition(float('inf')); rm.setPosition(float('inf'))
    lm.setVelocity(0); rm.setVelocity(0)

    # Map & Logic
    print(f"Loading map with Origin: ({MAP_ORIGIN_X}, {MAP_ORIGIN_Y})")
    grid = get_map(origin=(MAP_ORIGIN_X, MAP_ORIGIN_Y))
    planner = AStarPlanner()

    for _ in range(10): robot.step(timestep)

    # Mission Setup 
    dwa = DWA({
        "RADIUS": 0.035,
        "SAFE": 0.015,
        "V_MAX": 0.12, 
        "ALPHA": 0.8,
        "BETA": 0.4,
        "GAMMA": 0.3,
        "EPS": 0.1
    })

    print("Generating path...")
    coverage_grid_points = generate_lawnmower_path(grid)
    mission_waypoints = []
    for (r, c) in coverage_grid_points:
        wx, wy = grid.grid_to_world(r, c)
        mission_waypoints.append((wx, wy))
    
    print(f"Mission: {len(mission_waypoints)} points.")
    curr_idx = 0
    curr_path = [] 
    
    prev_cmd = (0.0, 0.0) 
    SMOOTH = 0.6 
    
    SPEED_WINDOW = deque(maxlen=20)
    AVG_THRESHOLD = 0.01 
    last_unstuck_time = -10.0
    recovery_end_time = 0.0
    UNSTUCK_COOLDOWN = 2.0
    last_trash_report = 0

    # --- Main Loop ---
    while robot.step(timestep) != -1:
        rx, ry, _ = gps.getValues()
        yaw = get_yaw(compass)
        t = robot.getTime()

        # Vision Check
        if vision.is_trash_visible(camera):
            if t - last_trash_report > 5.0:
                print(f"TRASH FOUND at ({rx:.2f}, {ry:.2f}) !!")
                last_trash_report = t

        # Mission Management
        if curr_idx >= len(mission_waypoints):
            lm.setVelocity(0); rm.setVelocity(0)
            print("Mission Complete.")
            break

        final_goal = mission_waypoints[curr_idx]
        
        # Check if reached final goal node
        if math.hypot(final_goal[0] - rx, final_goal[1] - ry) < 0.20:
            print(f"Visited Mission Point #{curr_idx}")
            curr_idx += 1
            curr_path = [] 
            continue

        # Global Planning (A*)
        if not curr_path:
            start_node = grid.world_to_grid(rx, ry)
            end_node = grid.world_to_grid(final_goal[0], final_goal[1])
            
            if not grid.is_valid(*start_node):
                print(f"CRITICAL: Robot off-grid at {start_node}!")
                lm.setVelocity(0); rm.setVelocity(0); break

            grid_path = planner.plan(grid, start_node, end_node)
            if grid_path:
                grid_path = planner.smooth_path(grid, grid_path)
                curr_path = []
                for r, c in grid_path:
                    gx, gy = grid.grid_to_world(r, c)
                    curr_path.append((gx, gy))
            else:
                curr_idx += 1 
                continue

        # 5. Local Targeting. We aim for the next point in the path list
        target_world = final_goal
        if len(curr_path) > 0:
            target_world = curr_path[0]
            # If we are close to this intermediate point, remove it and aim for next
            if math.hypot(target_world[0]-rx, target_world[1]-ry) < 0.25:
                curr_path.pop(0)
                if len(curr_path) > 0:
                    target_world = curr_path[0]

        # Debug info
        if t % 1.0 < 0.05:
            print(f"Going to: ({target_world[0]:.2f}, {target_world[1]:.2f}) | Path len: {len(curr_path)}")

        # Transform to Robot Frame
        dx = target_world[0] - rx
        dy = target_world[1] - ry
        
        gx, gy = world_to_robot(dx, dy, yaw)

        # DWA & Actuation
        scan = lidar.getRangeImage() if lidar else []
        obs_points = lidar_to_cartesian(scan, lidar_fov)
        
        v, w = dwa.get_safe_velocities(obs_points, (gx, gy), prev_cmd=prev_cmd)
        
        v = SMOOTH * prev_cmd[0] + (1.0 - SMOOTH) * v
        w = SMOOTH * prev_cmd[1] + (1.0 - SMOOTH) * w
        prev_cmd = (v, w)

        # Stuck Recovery
        SPEED_WINDOW.append(abs(v))
        is_stuck = (len(SPEED_WINDOW) == SPEED_WINDOW.maxlen and 
                    sum(SPEED_WINDOW) / len(SPEED_WINDOW) < AVG_THRESHOLD)

        if is_stuck and (t - last_unstuck_time) > UNSTUCK_COOLDOWN:
            print(">>> STUCK! Spin recovery...")
            recovery_end_time = t + 0.4 
            last_unstuck_time = t
        
        if t < recovery_end_time:
            v = 0.0; w = MAX_WHEEL_W 
            prev_cmd = (0, 0)

        # Wheel Velocities
        wl = (v - w * AXLE_LENGTH / 2) / WHEEL_RADIUS
        wr = (v + w * AXLE_LENGTH / 2) / WHEEL_RADIUS
        wl = max(-MAX_WHEEL_W, min(MAX_WHEEL_W, wl))
        wr = max(-MAX_WHEEL_W, min(MAX_WHEEL_W, wr))
        lm.setVelocity(wl); rm.setVelocity(wr)

if __name__ == "__main__":
    main()