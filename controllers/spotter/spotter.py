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

# 2. Hardware Names
LIDAR_NAME   = "lidar"
GPS_NAME     = "gps"
COMPASS_NAME = "compass"
CAMERA_NAME  = "camera"

# 3. E-puck Specs
WHEEL_RADIUS = 0.0205 
AXLE_LENGTH  = 0.052  
MAX_WHEEL_W  = 6.28   
# ==========================================

def get_robot_pose(gps, compass):
    if not gps or not compass: return 0,0,0
    vals = gps.getValues()
    rx, ry = vals[0], vals[1] # Z-up world
    c_vals = compass.getValues()
    theta = math.atan2(c_vals[0], c_vals[1])
    return rx, ry, theta

def lidar_to_cartesian(lidar_data, fov_rad): 
    points = []
    if not lidar_data: return points
    if len(lidar_data) < 2: return points

    angle_step = fov_rad / (len(lidar_data) - 1)
    current_angle = -fov_rad / 2.0
    
    for r in lidar_data:
        if 0.03 < r < 2.0: 
            # Standard Webots/ROS Body Frame (X=Forward, Y=Left)
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
    print("Initializing Spotter Bot (Debug Mode)...")
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

    # --- 2. Logic & Calibration ---
    
    print(f"Loading map with Origin: ({MAP_ORIGIN_X}, {MAP_ORIGIN_Y})")
    grid = get_map(origin=(MAP_ORIGIN_X, MAP_ORIGIN_Y))
    planner = AStarPlanner()

    # --- CALIBRATION CHECK (CRITICAL) ---
    # Wait for GPS to warm up
    for _ in range(10): robot.step(timestep)
    
    rx, ry, _ = get_robot_pose(gps, compass)
    r_grid, c_grid = grid.world_to_grid(rx, ry)
    
    print("-" * 40)
    print(f"CALIBRATION CHECK:")
    print(f"Robot World Pose: ({rx:.2f}, {ry:.2f})")
    print(f"Mapped Grid Cell: ({r_grid}, {c_grid})")
    
    if grid.is_valid(r_grid, c_grid):
        status = "FREE" if grid.is_free(r_grid, c_grid) else "OCCUPIED (WALL)"
        val = grid.grid[r_grid][c_grid]
        print(f"Cell Status: {status} (Val: {val})")
        
        if not grid.is_free(r_grid, c_grid):
            print("\n!!! ERROR !!!")
            print("The robot thinks it is inside a wall.")
            print("Adjust MAP_ORIGIN_X or MAP_ORIGIN_Y until this says FREE.")
            print("-" * 40)
            # Stop here to prevent the loop failure
            return 
    else:
        print("\n!!! ERROR !!!")
        print("Robot is OFF THE MAP grid.")
        print("-" * 40)
        return
    print("-" * 40)

    # DWA Config
    dwa = DWA({
        "RADIUS": 0.01, "SAFE": 0.001, "BETA": 0.80, 
        "GAMMA": 0.05, "ALPHA": 0.15, "T_PRED": 1.0,  
        "A_V": 2.0, "V_MAX": 0.12, "EPS": 0.05  
    })

    # Nav State
    prev_cmd = (0.0, 0.0) 
    SMOOTH = 0.5 
    SPEED_WINDOW = deque(maxlen=20)
    AVG_THRESHOLD = 0.01 
    last_unstuck_time = -10.0
    recovery_end_time = 0.0
    UNSTUCK_COOLDOWN = 2.0

    # --- 3. Mission ---
    print("Generating coverage path...")
    coverage_grid_points = generate_lawnmower_path(grid)
    mission_waypoints = []
    
    for (r, c) in coverage_grid_points:
        wx, wy = grid.grid_to_world(r, c)
        mission_waypoints.append((wx, wy))
    
    print(f"Mission: {len(mission_waypoints)} points.")
    curr_idx = 0
    curr_path = [] 
    last_trash_report = 0

    # --- Main Loop ---
    while robot.step(timestep) != -1:
        rx, ry, rtheta = get_robot_pose(gps, compass)
        t = robot.getTime()

        # A. Vision
        if vision.is_trash_visible(camera):
            if t - last_trash_report > 5.0:
                print(f"!!! TRASH FOUND at ({rx:.2f}, {ry:.2f}) !!!")
                last_trash_report = t

        # B. Mission Logic
        if curr_idx >= len(mission_waypoints):
            lm.setVelocity(0); rm.setVelocity(0)
            print("Mission Complete.")
            break

        final_goal = mission_waypoints[curr_idx]
        dist_to_goal = math.hypot(final_goal[0] - rx, final_goal[1] - ry)

        if dist_to_goal < 0.20:
            print(f"Visited #{curr_idx}")
            curr_idx += 1
            curr_path = []
            continue

        # C. Path Planning (Global A*)
        if not curr_path:
            start_node = grid.world_to_grid(rx, ry)
            end_node = grid.world_to_grid(final_goal[0], final_goal[1])
            
            # --- RESTORED ERROR PRINTS ---
            if not grid.is_valid(*start_node):
                print(f"PLAN ERROR: Start node {start_node} is off-grid!")
                lm.setVelocity(0); rm.setVelocity(0)
                break # Hard stop
                
            grid_path = planner.plan(grid, start_node, end_node)
            if grid_path:
                grid_path = planner.smooth_path(grid, grid_path)
                curr_path = []
                for r, c in grid_path:
                    gx, gy = grid.grid_to_world(r, c)
                    curr_path.append((gx, gy))
            else:
                # DEBUG: Why did it fail?
                print(f"PLAN FAIL: No path from {start_node} to {end_node}. Robot at ({rx:.2f}, {ry:.2f})")
                print(f"Start Free? {grid.is_free(*start_node)} | End Free? {grid.is_free(*end_node)}")
                curr_idx += 1
                continue

        # D. Local Targeting
        target_world = final_goal
        if len(curr_path) > 1:
            target_world = curr_path[1]
            if math.hypot(target_world[0]-rx, target_world[1]-ry) < 0.3:
                curr_path.pop(0)

        # Transform
        dx = target_world[0] - rx
        dy = target_world[1] - ry
        local_x = dx * math.cos(-rtheta) - dy * math.sin(-rtheta)
        local_y = dx * math.sin(-rtheta) + dy * math.cos(-rtheta)

        # E. DWA + Recovery
        scan = lidar.getRangeImage() if lidar else []
        obs_points = lidar_to_cartesian(scan, lidar_fov)
        
        v, w = dwa.get_safe_velocities(obs_points, (local_x, local_y), prev_cmd=prev_cmd)
        
        # Smoother
        v = SMOOTH * prev_cmd[0] + (1.0 - SMOOTH) * v
        w = SMOOTH * prev_cmd[1] + (1.0 - SMOOTH) * w
        prev_cmd = (v, w)

        # Stuck Recovery
        SPEED_WINDOW.append(abs(v))
        is_stuck = (len(SPEED_WINDOW) == SPEED_WINDOW.maxlen and 
                    sum(SPEED_WINDOW) / len(SPEED_WINDOW) < AVG_THRESHOLD)

        if is_stuck and (t - last_unstuck_time) > UNSTUCK_COOLDOWN:
            print(">>> STUCK! Spin recovery...")
            recovery_end_time = t + 0.3 
            last_unstuck_time = t
        
        if t < recovery_end_time:
            v = 0.0; w = MAX_WHEEL_W 
            prev_cmd = (0, 0)

        # F. Actuate
        wl = (v - w * AXLE_LENGTH / 2) / WHEEL_RADIUS
        wr = (v + w * AXLE_LENGTH / 2) / WHEEL_RADIUS
        wl = max(-MAX_WHEEL_W, min(MAX_WHEEL_W, wl))
        wr = max(-MAX_WHEEL_W, min(MAX_WHEEL_W, wr))
        lm.setVelocity(wl); rm.setVelocity(wr)

if __name__ == "__main__":
    main()