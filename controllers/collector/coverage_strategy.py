# COVERAGE STRATEGY MODULE -> Abdullateef 
# Implements the "Coverage" setup logic.
# - Hardcoded waypoints for grid splitting.
# - Distance-based rubbish detection.

import math

WHEEL_RADIUS = 0.033
AXLE_LENGTH = 0.16

# HARDCODED WAYPOINTS 

COVERAGE_WAYPOINTS = { # TODO
    "collector_1": [
        (2.5, -0.5), (2.5, 3.5), # Right Aisle Up
        (0.0, 3.5),  (0.0, -0.5), # Center Aisle Down
        (-2.5, -0.5), (-2.5, 3.5), # Left Aisle Up
        (0.0, 0.0) # Return to center-ish
    ],
    "collector_2": [
        (2.5, -1.5), (2.5, -5.0), # Right Aisle Down
        (0.0, -5.0), (0.0, -1.5), # Center Aisle Up
        (-2.5, -1.5), (-2.5, -5.0), # Left Aisle Down
        (0.0, -3.0) # Return to center-ish
    ]
}

def get_lidar_points(lidar, max_r=3.5, min_r=0.1): # TODO
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

def run_coverage_setup(robot, rubbish_list):
    print(f"{robot.robot_id}: Starting COVERAGE mode logic.")
    
    waypoints = COVERAGE_WAYPOINTS.get(robot.robot_id, COVERAGE_WAYPOINTS["collector_1"]) # TODO
    current_wp_idx = 0
    
    if waypoints:
        robot.nav.plan_path((robot.rx, robot.ry), waypoints[current_wp_idx])
    
    collected_ids = set()

    # Main Loop
    while robot.step(robot.timestep) != -1:
        robot.update_pose()
        robot.handle_messages() # Handle incoming messages
        
        # DISTANCE BASED RUBBISH DETECTION
        for item in rubbish_list:
            r_id = item['id']
            if r_id in collected_ids:
                continue
            
            dist = math.hypot(robot.rx - item['x'], robot.ry - item['y'])
            
            # Detection Threshold
            if dist < 0.5:
                print(f"!!! {robot.robot_id} DETECTED RUBBISH {r_id} !!!")
                
                # Stop briefly
                robot.lm.setVelocity(0)
                robot.rm.setVelocity(0)
                
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

        # NAVIGATION
        now = robot.getTime()
        lidar_pts = get_lidar_points(robot.lidar)
        
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
            # Reached current waypoint
            current_wp_idx += 1
            if current_wp_idx < len(waypoints):
                print(f"{robot.robot_id}: Moving to WP {current_wp_idx} {waypoints[current_wp_idx]}")
                success = robot.nav.plan_path((robot.rx, robot.ry), waypoints[current_wp_idx])
                if not success:
                    print(f"Plan failed to WP {current_wp_idx}, skipping...")
            else:
                # If finished pattern, loop back to start to ensure full coverage
                print(f"{robot.robot_id}: Pattern complete. Restarting pattern.")
                current_wp_idx = 0
                robot.nav.plan_path((robot.rx, robot.ry), waypoints[current_wp_idx])