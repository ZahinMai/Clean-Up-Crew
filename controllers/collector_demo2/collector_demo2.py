# controllers/collector_demo/collector_demo.py
# TurtleBot3 Burger with A* Global Planning + DWA Local Planning
# Enhanced version with hierarchical navigation

from controller import Robot
import math, os, sys
from collections import deque

# allow importing from ../lib_shared
THIS_DIR = os.path.dirname(__file__)
CTRL_DIR = os.path.dirname(THIS_DIR)
if CTRL_DIR not in sys.path:
    sys.path.append(CTRL_DIR)

from lib_shared.local_planner import DWA
from lib_shared.navigation import AStarPlanner
from lib_shared.map import get_map


# Initialise device names and robot geometry
LEFT_MOTOR   = 'left wheel motor'
RIGHT_MOTOR  = 'right wheel motor'
LIDAR_NAME   = 'LDS-01'
IMU_NAME     = 'inertial unit'
COMPASS_NAME = 'compass'
GPS_NAME     = 'gps'
WHEEL_RADIUS = 0.033
AXLE_LENGTH  = 0.160

# Navigation parameters
WAYPOINT_TOLERANCE = 0.18      # meters - when to advance to next waypoint
REPLAN_INTERVAL = 5000.0          # seconds - minimum time between replans
REPLAN_DISTANCE = 0.8          # meters - replan if deviated this far
LOOKAHEAD = 2                  # number of waypoints to look ahead

# Helper functions
def get_device(robot, name):
    try:
        return robot.getDevice(name)
    except:
        return None

# get angle around vertical axis
def get_yaw(imu, compass):
    if imu:
        r, p, y = imu.getRollPitchYaw()
        return y
    if compass:
        n = compass.getValues()
        return math.atan2(n[0], n[2])
    return 0.0

# map world to robot frame
def world_to_robot(dx, dz, yaw):
    gx = dx*math.cos(-yaw) - dz*math.sin(-yaw)
    gy = dx*math.sin(-yaw) + dz*math.cos(-yaw)
    return gx, gy


class Navigator:    
    def __init__(self, occupancy_grid):
        self.grid = occupancy_grid
        self.astar = AStarPlanner(allow_diagonal=True)
        self.global_path = []
        self.global_path_world = []
        self.current_wp_idx = 0
        self.last_replan_time = -100.0
    
    def plan_path(self, start_x, start_z, goal_x, goal_z):
        # Convert to grid coordinates
        start_grid = self.grid.world_to_grid(start_x, start_z)
        goal_grid = self.grid.world_to_grid(goal_x, goal_z)
        
        # Check if positions are free
        if not self.grid.is_free(*start_grid):
            print(f"[Nav] Start {start_grid} occupied, finding free cell...")
            start_grid = self._find_free(start_grid)
            if not start_grid:
                return False
        
        if not self.grid.is_free(*goal_grid):
            print(f"[Nav] Goal {goal_grid} occupied, finding free cell...")
            goal_grid = self._find_free(goal_grid)
            if not goal_grid:
                return False
        
        # Plan path with A*, then smooth the path
        path = self.astar.plan(self.grid, start_grid, goal_grid)
        if not path:
            print(f"[Nav] A* failed to find path!")
            return False
        path = self.astar.smooth_path(self.grid, path) 
        
        # Convert to world coordinates
        self.global_path = path
        self.global_path_world = [
            self.grid.grid_to_world(r, c) for r, c in path
        ]
        self.current_wp_idx = 0
        
        print(f"[Nav] Path planned: {len(self.global_path)} waypoints")
        return True
    
    # Find nearest free cell within radius
    def _find_free(self, pos, radius=5):
        r, c = pos
        for d in range(1, radius + 1):
            for dr in range(-d, d + 1):
                for dc in range(-d, d + 1):
                    if self.grid.is_free(r + dr, c + dc):
                        return (r + dr, c + dc)
        return None
    
    def should_replan(self, robot_x, robot_z, time):
        if not self.global_path_world:
            return False
        
        if time - self.last_replan_time < REPLAN_INTERVAL:
            return False
        
        # Has robot deviated significantly from path?
        min_dist = min(
            math.hypot(wx - robot_x, wz - robot_z)
            for wx, wz in self.global_path_world[self.current_wp_idx:]
        )
        
        return min_dist > REPLAN_DISTANCE
    
    def get_next_waypoint(self, robot_x, robot_z):
        if not self.global_path_world:
            return None
        
        # Advance through reached waypoints
        while self.current_wp_idx < len(self.global_path_world):
            wx, wz = self.global_path_world[self.current_wp_idx]
            dist = math.hypot(wx - robot_x, wz - robot_z)
            
            if dist < WAYPOINT_TOLERANCE:
                self.current_wp_idx += 1
                print(f"[Nav] → Waypoint {self.current_wp_idx}/{len(self.global_path_world)}")
            else:
                break
        
        # Check if reached goal
        if self.current_wp_idx >= len(self.global_path_world):
            return None
        
        # Look ahead for smoother motion
        lookahead_idx = min(
            self.current_wp_idx + LOOKAHEAD,
            len(self.global_path_world) - 1
        )
        
        return self.global_path_world[lookahead_idx]
    
    def get_progress(self):
        if not self.global_path:
            return 0.0
        return 100.0 * self.current_wp_idx / len(self.global_path)


def main():
    robot = Robot()
    ts = int(robot.getBasicTimeStep())
    
    print("=" * 70)
    print("COLLECTOR DEMO - A* + DWA Navigation")
    print("=" * 70)

    # --- Initialize devices ---
    lm = get_device(robot, LEFT_MOTOR)
    rm = get_device(robot, RIGHT_MOTOR)
    lm.setPosition(float('inf'))
    rm.setPosition(float('inf'))
    lm.setVelocity(0.0)
    rm.setVelocity(0.0)

    imu = get_device(robot, IMU_NAME)
    if imu: imu.enable(ts)
    compass = get_device(robot, COMPASS_NAME)
    if compass: compass.enable(ts)
    gps = get_device(robot, GPS_NAME)
    gps.enable(ts)
    lidar = get_device(robot, LIDAR_NAME)
    lidar.enable(ts)

    # --- Load map and create navigator ---
    print("\n[Setup] Loading map...")
    grid = get_map()
    
    print("[Setup] Initializing navigator...")
    navigator = Navigator(grid)

    # DWA local planner with tuned parameters
    dwa = DWA({
        "V_MAX":   0.20,  # Conservative speed
        "SAFE":    0.12,  # Safety margin
        "CLEAR_N": 1.00,
        "GAMMA":   0.60,  # Higher clearance weight
        "BETA":    0.30,
        "ALPHA":   0.30,
        "EPS":     0.15   # More smoothing
    })

    # Waypoints to visit (world coordinates)
    MISSION = [
        (5.0, 5.0, "North-East"),
        (5.0, -5.0, "South-East"),
        (-5.0, -5.0, "South-West"),
        (-5.0, 5.0, "North-West"),
        (0.0, 0.0, "Center"),
    ]
    mission_idx = 0
    
    # State
    prev_cmd = (0.0, 0.0)
    SMOOTH = 0.7
    path_planned = False
    
    # Stuck detection
    SPEED_WINDOW = deque(maxlen=20)
    AVG_THRESHOLD = 0.02
    last_unstuck = -10.0
    UNSTUCK_COOLDOWN = 1.5
    
    print("[Setup] Starting mission!\n")

    while robot.step(ts) != -1:
        now = robot.getTime()
        
        # Get current pose
        x, _, z = gps.getValues()
        yaw = get_yaw(imu, compass)

        # Mission management
        if not path_planned:
            if mission_idx >= len(MISSION):
                print("\n" + "=" * 70)
                print("MISSION COMPLETE!")
                print("=" * 70)
                lm.setVelocity(0.0)
                rm.setVelocity(0.0)
                break
            
            goal_x, goal_z, goal_name = MISSION[mission_idx]
            print(f"\n{'=' * 70}")
            print(f"GOAL [{mission_idx + 1}/{len(MISSION)}]: {goal_name}")
            print(f"Target: ({goal_x:.1f}, {goal_z:.1f})")
            print(f"{'=' * 70}")
            
            if navigator.plan_path(x, z, goal_x, goal_z):
                path_planned = True
                navigator.last_replan_time = now
            else:
                print("[Warning] Planning failed, skipping goal...")
                mission_idx += 1
                continue
        
        # Check if should replan
        if navigator.should_replan(x, z, now):
            print("[Nav] Replanning...")
            goal_x, goal_z, _ = MISSION[mission_idx]
            if navigator.plan_path(x, z, goal_x, goal_z):
                navigator.last_replan_time = now
        
        # Get next waypoint
        waypoint = navigator.get_next_waypoint(x, z)
        
        if waypoint is None:
            # Goal reached!
            print(f"\n{'=' * 70}")
            print(f"✓ REACHED: {MISSION[mission_idx][2]}")
            print(f"Position: ({x:.2f}, {z:.2f})")
            print(f"{'=' * 70}\n")
            
            mission_idx += 1
            path_planned = False
            prev_cmd = (0.0, 0.0)
            continue
        
        # Get obstacles from LiDAR
        ranges = lidar.getRangeImage()
        hfov = lidar.getFov()
        nscan = lidar.getHorizontalResolution()
        obs = []
        for i, r in enumerate(ranges):
            if r != float('inf') and r > 0.03:
                a = -hfov/2 + hfov * (i / (nscan - 1))
                obs.append((r*math.cos(a), r*math.sin(a)))

        # Goal in robot frame
        dx = waypoint[0] - x
        dz = waypoint[1] - z
        gx, gy = world_to_robot(dx, dz, yaw)

        # DWA
        v, w = dwa.get_safe_velocities(obs, (gx, gy), prev_cmd=prev_cmd, cur=prev_cmd)

        # Smoothing
        v = SMOOTH * prev_cmd[0] + (1.0 - SMOOTH) * v
        w = SMOOTH * prev_cmd[1] + (1.0 - SMOOTH) * w
        prev_cmd = (v, w)

        # Stuck detection & recovery
        SPEED_WINDOW.append(abs(v))
        stuck = (len(SPEED_WINDOW) == SPEED_WINDOW.maxlen and
                 sum(SPEED_WINDOW) / len(SPEED_WINDOW) < AVG_THRESHOLD)

        if stuck and (now - last_unstuck) > UNSTUCK_COOLDOWN:
            print("[Recovery] Stuck, rotating...")
            v = 0.0
            w = 0.5
            prev_cmd = (v, w)
            last_unstuck = now
            navigator.last_replan_time = now - 10.0  # Force replan

        # Convert to wheel velocities
        wl = (v - 0.5*w*AXLE_LENGTH) / WHEEL_RADIUS
        wr = (v + 0.5*w*AXLE_LENGTH) / WHEEL_RADIUS

        max_w = min(lm.getMaxVelocity(), rm.getMaxVelocity())
        wl = max(-max_w, min(max_w, wl))
        wr = max(-max_w, min(max_w, wr))

        lm.setVelocity(wl)
        rm.setVelocity(wr)

        # Status (every second)
        if int(now) % 1 == 0:
            prog = navigator.get_progress()
            print(f"[Nav] Goal {mission_idx + 1}/{len(MISSION)} | "
                  f"Pos: ({x:.1f},{z:.1f}) | "
                  f"Progress: {prog:.0f}% | "
                  f"v={v:.2f} w={w:.2f} | obs={len(obs)}")


if __name__ == "__main__":
    main()