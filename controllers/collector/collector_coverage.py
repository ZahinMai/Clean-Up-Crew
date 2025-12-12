# COVERAGE STRATEGY MODULE -> Abdullateef 
# Implements the "Coverage" setup logic.
# - Hardcoded waypoints for grid splitting.
# - Distance-based rubbish detection.

import math
from lib_shared.coverage import CoveragePlanner
from lib_shared.map_module import get_map

WHEEL_RADIUS = 0.033
AXLE_LENGTH = 0.16
CAUTION_ZONE = 0.5 # Define threshold for collision counting

# HARDCODED WAYPOINTS 
grid = get_map()
coverage_planner = CoveragePlanner(grid)
mission_waypoints = coverage_planner.generate_hardcoded_waypoints()

swarm_waypoints = {
    "collector_1": mission_waypoints[18:], # Left Side
    "collector_2": mission_waypoints[:18], # Right Side
}

baseline_waypoints = {
    "collector_1": [], # No waypoints
    "collector_2": mission_waypoints, # Full grid
}

def get_lidar_points(lidar, max_r=3.5, min_r=0.1):
    """Helper to get lidar points."""
    if not lidar: return []
    try: ranges = lidar.getRangeImage()
    except Exception: return []
    if not ranges: return []

    fov, n = lidar.getFov(), len(ranges)
    return [
        (r * math.cos(-fov / 2 + i / (n - 1) * fov),
         r * math.sin(-fov / 2 + i / (n - 1) * fov))
        for i, r in enumerate(ranges)
        if min_r <= r <= max_r and math.isfinite(r)
    ]

def run_coverage_setup(robot, rubbish_list, setup_mode):
    print(f"{robot.robot_id}: Starting COVERAGE mode logic.")
    waypoints = []
    if setup_mode == "SWARM":
        waypoints = swarm_waypoints.get(robot.robot_id, [])
    elif setup_mode == "BASELINE":
        waypoints = baseline_waypoints.get(robot.robot_id, [])
    current_wp_idx = 0

    if waypoints:
        robot.update_pose()
        robot.nav.plan_path((robot.rx, robot.ry), waypoints[current_wp_idx])

    collected_ids = set()

    # Main Loop
    while robot.step(robot.timestep) != -1:
        robot.update_pose()
        robot.handle_messages() 

        # METRIC: Distance Covered
        if robot.prev_rx is not None and robot.prev_ry is not None:
            step_dist = math.hypot(robot.rx - robot.prev_rx, robot.ry - robot.prev_ry)
            robot.dist_covered += step_dist
        robot.prev_rx = robot.rx
        robot.prev_ry = robot.ry

        # DISTANCE BASED RUBBISH DETECTION
        for item in rubbish_list:
            r_id = item['id']
            if r_id in collected_ids:
                continue

            dist = math.hypot(robot.rx - item['x'], robot.ry - item['y'])

            if dist < 0.5:
                print(f"!!! {robot.robot_id} DETECTED RUBBISH {r_id} !!!")

                # Stop briefly
                robot.lm.setVelocity(0)
                robot.rm.setVelocity(0)
                
                # Update idle time for the stop
                robot.total_idle_time += (robot.timestep / 1000.0)

                # Send collected message
                robot.comm.send({
                    "event": "collected",
                    "collector_id": robot.robot_id,
                    "task_id": r_id,
                    "x": robot.rx,
                    "y": robot.ry,
                })
                collected_ids.add(r_id)
                print(f"-> Sent collection confirmation for {r_id}")

        # NAVIGATION & COLLISION METRICS
        now = robot.getTime()
        lidar_pts = get_lidar_points(robot.lidar)

        # METRIC: Collision Counting
        min_dist = float('inf')
        if lidar_pts:
            min_dist = min(math.hypot(x, y) for x, y in lidar_pts)
        
        if min_dist < CAUTION_ZONE:
            if not robot.is_in_collision:
                robot.collision_count += 1
                robot.is_in_collision = True
        else:
            robot.is_in_collision = False
        # ----------------------------------

        # Update Navigator
        v, w, moving = robot.nav.update(
            pose=(robot.rx, robot.ry, robot.yaw),
            lidar_pts=lidar_pts,
            now=now,
        )

        wl = (v + w * AXLE_LENGTH * 0.5) / WHEEL_RADIUS
        wr = (v - w * AXLE_LENGTH * 0.5) / WHEEL_RADIUS
        limit = min(robot.lm.getMaxVelocity(), robot.rm.getMaxVelocity())

        robot.lm.setVelocity(max(-limit, min(limit, wl)))
        robot.rm.setVelocity(max(-limit, min(limit, wr)))

        # WAYPOINT SWITCHING
        if not moving:
            current_wp_idx += 1
            if current_wp_idx < len(waypoints):
                print(f"{robot.robot_id}: Moving to WP {current_wp_idx} {waypoints[current_wp_idx]}")
                success = robot.nav.plan_path((robot.rx, robot.ry), waypoints[current_wp_idx])
                if not success:
                    print(f"Plan failed to WP {current_wp_idx}, skipping...")
            else:
                print(f"{robot.robot_id}: Pattern complete. Stopping run.")
                
                # Stop motors
                robot.lm.setVelocity(0)
                robot.rm.setVelocity(0)
                
                return