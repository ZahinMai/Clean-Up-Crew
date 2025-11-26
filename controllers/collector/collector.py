from controller import Robot
import math, os, sys
from collections import deque

# ---- Import Shared Library ---- #
if os.path.dirname(os.path.dirname(__file__)) not in sys.path:
    sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from lib_shared.global_planner import AStarPlanner
from lib_shared.map_module import visualise_robot_on_map, get_map
from lib_shared.local_planner import DWA
# ------------------------------ #

# Configuration
WHEEL_RADIUS, AXLE_LENGTH = 0.033, 0.16
MISSIONS = [(5.5, 1.6), (3.5, 2.0)]
DWA_ENABLED = False

dwa_config = {
    'DT_SIM': 0.15,
    'T_PRED': 1.0, # how far ahead to plan
    'NV': 7, # velocity s
    'NW': 10,
    'V_MAX': 0.35,
    'W_MAX': 2.0,
    'A_V': 1.0,
    'A_W': 2.0,
    'RADIUS': 0.09,
    'SAFE': 0.01,
    'ALPHA': 0.3,
    'BETA': 0.3,
    'GAMMA': 0.6,
    'DELTA': 0.1,
    'EPS': 0.1,
    'LIDAR_SKIP': 4,
}

def get_lidar_points(lidar, max_r=3.5, min_r=0.1):
    if not lidar: return []
    try: ranges = lidar.getRangeImage()
    except: return []
    if not ranges: return []
    
    fov = lidar.getFov(); n = len(ranges)
    points = []
    for i, r in enumerate(ranges):
        if r > max_r or r < min_r or not math.isfinite(r): continue
        a = -fov/2 + (i/(n-1))*fov
        points.append((r*math.cos(a), r*math.sin(a)))
    return points

def main():
    # Setup Robot & Devices
    robot = Robot()
    ts = int(robot.getBasicTimeStep())
    
    gps = robot.getDevice('gps'); gps.enable(ts)
    imu = robot.getDevice('inertial unit'); imu.enable(ts)
    compass = robot.getDevice('compass'); compass.enable(ts)
    lidar = robot.getDevice('LDS-01'); lidar.enable(ts); lidar.enablePointCloud()
    
    lm = robot.getDevice('left wheel motor'); lm.setPosition(float('inf')); lm.setVelocity(0)
    rm = robot.getDevice('right wheel motor'); rm.setPosition(float('inf')); rm.setVelocity(0)

    # Setup Planners
    grid = get_map(verbose=False)
    plan_grid = grid.inflate(0) # minimal inflation for planning
    astar = AStarPlanner()
    dwa = DWA(params=dwa_config)

    # Initialise Variables
    path_world = []     # List of (x, z) tuples
    path_idx = 0        # Current index in path
    mission_idx = 0     # Current goal index
    last_replan = 0.0
    last_vis = 0.0
    prev_cmd = (0.0, 0.0)

    # Helper to find path
    def plan_path(sx, sz, gx, gz):
        def valid_node(node): # Find nearest free node if start/goal are blocked
            if plan_grid.is_free(*node): return node
            q, seen = deque([node]), {node}
            while q:
                curr = q.popleft()
                if plan_grid.is_free(*curr): return curr
                for dr, dc in [(0,1),(0,-1),(1,0),(-1,0)]:
                    nr, nc = curr[0]+dr, curr[1]+dc
                    if (nr, nc) not in seen and 0<=nr<plan_grid.height and 0<=nc<plan_grid.width:
                        seen.add((nr,nc)); q.append((nr,nc))
            return None

        start = valid_node(plan_grid.world_to_grid(sx, sz))
        goal = valid_node(plan_grid.world_to_grid(gx, gz))
        if not start or not goal: return []
        
        nodes = astar.plan(plan_grid, start, goal)
        return [grid.grid_to_world(r,c) for r,c in nodes] if nodes else []

    # ---------------- Main loop -------------------- #
    while robot.step(ts) != -1:
        now = robot.getTime()
        if now < 1.0: continue

        rx, rz = gps.getValues()[0], gps.getValues()[1]
        yaw = 0.0
        if imu: yaw = imu.getRollPitchYaw()[2]
        elif compass: v = compass.getValues(); yaw = math.atan2(v[0], v[2])

        # --- Visualisation ---
        if now - last_vis > 0.5:
            vis_path = [grid.world_to_grid(wx, wz) for wx, wz in path_world]
            visualise_robot_on_map(grid, rx, rz, yaw, path=vis_path)
            last_vis = now

        # High-Level Logic (Mission & Re-planning)
        if mission_idx < len(MISSIONS):
            gx, gz = MISSIONS[mission_idx]
            # Plan if no path or time to replan
            if not path_world or (now - last_replan > 1.0):
                new_path = plan_path(rx, rz, gx, gz)
                if new_path:
                    path_world = new_path
                    path_idx = 0
                    last_replan = now
                elif not path_world: # Stuck? Skip mission
                    mission_idx += 1; continue
        else:
            lm.setVelocity(0); rm.setVelocity(0); break # Done

        # Path Following (Lookahead)
        target = None
        while path_idx < len(path_world):
            wx, wz = path_world[path_idx]
            if math.hypot(wx - rx, wz - rz) < 0.2: # Tolerance
                path_idx += 1
            else:
                target = path_world[min(path_idx + 1, len(path_world)-1)]
                break
        
        if not target: # Path finished
            mission_idx += 1; path_world = []; continue

        # ---- Use Local Planner (DWA) ----
        tx, tz = target
        
        # World to Robot Transform (Inlined)
        dx, dy = tx - rx, tz - rz
        gx_r = dx * math.cos(-yaw) - dy * math.sin(-yaw)
        gy_r = dx * math.sin(-yaw) + dy * math.cos(-yaw)

        # Get Velocities
        if DWA_ENABLED:
            v, w = dwa.get_safe_velocities(get_lidar_points(lidar), (gx_r, gy_r), prev_cmd, prev_cmd)
            
        else:
            # Simple Proportional fallback
            dist = math.hypot(gx_r, gy_r)
            ang = math.atan2(gy_r, gx_r)
            v = 0.8 * dist if abs(ang) < 1.0 else 0.0
            w = 1.5 * ang
            v = max(-0.35, min(0.35, v))
            w = max(-2.0, min(2.0, w))
            
        prev_cmd = (v, w)

        # Actuation (Differential Drive Mixing)
        wl = (v - (-w) * AXLE_LENGTH * 0.5) / WHEEL_RADIUS
        wr = (v + (-w) * AXLE_LENGTH * 0.5) / WHEEL_RADIUS
        
        limit = min(lm.getMaxVelocity(), rm.getMaxVelocity())
        lm.setVelocity(max(-limit, min(limit, wl)))
        rm.setVelocity(max(-limit, min(limit, wr)))

if __name__ == "__main__":
    main()