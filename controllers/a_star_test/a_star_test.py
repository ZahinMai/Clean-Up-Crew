from controller import Robot
import math, os, sys

# --- So I can import from lib_shared ---
THIS_DIR = os.path.dirname(__file__)
if os.path.dirname(THIS_DIR) not in sys.path: sys.path.append(os.path.dirname(THIS_DIR))
# --------------------------------------

from lib_shared.global_planner import AStarPlanner
from lib_shared.map_module import visualise_robot_on_map, get_map
from lib_shared.local_planner import DWA

# --- Constants ---
LEFT_MOTOR, RIGHT_MOTOR = 'left wheel motor', 'right wheel motor'
LIDAR_NAME, IMU_NAME, COMPASS_NAME, GPS_NAME = 'LDS-01', 'inertial unit', 'compass', 'gps'
WHEEL_RADIUS, AXLE_LENGTH = 0.033, 0.16 # from Webots docs

WAYPOINT_TOLERANCE = 0.2 # how close to waypoint to consider reached
REPLAN_INTERVAL = 0.5 # replan often to account for drift
LOOKAHEAD = 1 # waypoints ahead to target - help smooth turns
VIS_INTERVAL = 0.5 # seconds between map visualisations in console (debug)

# --- Helper functions (for imu & lidar) ---
def get_yaw(imu, compass): # yaw is the angle around the vertical axis
    if imu:
        _, _, y = imu.getRollPitchYaw()
        return y
    if compass:
        n = compass.getValues()
        return math.atan2(n[0], n[2])
    return 0.0

def process_lidar(lidar, max_range=3.5):
    ranges = lidar.getRangeImage()
    points = []
    n = len(ranges)
    fov = 2 * math.pi
    for i, r in enumerate(ranges):
        if r == float('inf') or r > max_range or r < 0.1: continue
        angle = -fov/2 + (i / (n - 1)) * fov
        lx = r * math.cos(angle)
        ly = r * math.sin(angle)
        points.append((lx, ly))
    return points

class Navigator:
    def __init__(self, grid):
        self.grid = grid
        self.plan_grid = grid.inflate(0) # no inflation for now
        self.astar = AStarPlanner()
        self.path_world = []
        self.idx = 0

    def plan(self, sx, sz, gx, gz):
        # Convert world to grid coordinates for planning
        s_grid = self.plan_grid.world_to_grid(sx, sz)
        g_grid = self.plan_grid.world_to_grid(gx, gz)
        
        print(f"[Nav] Plan request: World({sx:.2f},{sz:.2f}) -> Grid{s_grid}")
        
        # Check if start is free, relocate if blocked
        safe_start = s_grid if self.plan_grid.is_free(*s_grid) else self._bfs_free(s_grid)
        
        if not safe_start: # Can't relocate start -> give up
            print(f"[Nav] Blocked start {s_grid}. No escape.")
            return False 
        
        if safe_start != s_grid: # Log relocation
            print(f"[Nav] Relocated start to {safe_start}")

        # Check if goal is free, relocate if blocked
        safe_goal = g_grid if self.plan_grid.is_free(*g_grid) else self._bfs_free(g_grid)
        
        if not safe_goal: # Can't relocate goal -> give up
            print(f"[Nav] Blocked goal {g_grid}. No escape.")
            return False 

        # Plan the path
        path_indices = self.astar.plan(self.plan_grid, safe_start, safe_goal)
        if not path_indices:
            print("[Nav] No path found.")
            return False 

        self.path_world = [self.grid.grid_to_world(r, c) for r, c in path_indices]
        self.idx = 0 # reset waypoint index
        return True

    # to quickly find nearest free cell
    def _bfs_free(self, start, max_dist=8):
        for d in range(1, max_dist+1):
            for r in range(start[0]-d, start[0]+d+1):
                for c in range(start[1]-d, start[1]+d+1):
                    if self.plan_grid.is_free(r, c): return (r, c)
        return None

    # Get next target waypoint in world coordinates
    def get_target(self, rx, rz):
        if not self.path_world: return None
        while self.idx < len(self.path_world):
            wx, wz = self.path_world[self.idx]
            if math.hypot(wx-rx, wz-rz) < WAYPOINT_TOLERANCE: self.idx += 1
            else: break
        if self.idx >= len(self.path_world): return None
        return self.path_world[min(self.idx + LOOKAHEAD, len(self.path_world)-1)]

def main():
    robot = Robot()
    ts = int(robot.getBasicTimeStep())
    
    gps = robot.getDevice(GPS_NAME); gps.enable(ts)
    imu = robot.getDevice(IMU_NAME); imu.enable(ts)
    compass = robot.getDevice(COMPASS_NAME); compass.enable(ts)
    lidar = robot.getDevice(LIDAR_NAME); lidar.enable(ts); lidar.enablePointCloud()
    
    lm = robot.getDevice(LEFT_MOTOR); lm.setPosition(float('inf')); lm.setVelocity(0)
    rm = robot.getDevice(RIGHT_MOTOR); rm.setPosition(float('inf')); rm.setVelocity(0)

    grid = get_map(verbose=True)
    nav = Navigator(grid)
    dwa = DWA()
    
    MISSIONS = [(5.5, 1.6), (3.5, 3.5)]
    m_idx = 0
    path_planned = False
    last_vis = 0.0 
    last_replan = 0.0 
    prev_cmd = (0.0, 0.0) # this helps smooth DWA commands
    
    while robot.step(ts) != -1:
        now = robot.getTime()
        if now < 1.0: continue 
        
        rx, rz = gps.getValues()[0], gps.getValues()[1]
        yaw = get_yaw(imu, compass)

        # -- Visualise
        if now - last_vis > VIS_INTERVAL:
            path_idx = [grid.world_to_grid(wx, wz) for wx, wz in nav.path_world]
            visualise_robot_on_map(grid, rx, rz, yaw, path=path_idx)
            last_vis = now

        # -- Plan
        if not path_planned:
            if m_idx >= len(MISSIONS): lm.setVelocity(0); rm.setVelocity(0); print("Done!"); break
            gx, gz = MISSIONS[m_idx]
            if nav.plan(rx, rz, gx, gz): path_planned = True; last_replan = now
            else: print("Plan failed, skipping"); m_idx += 1; continue

        # -- Replan
        if now - last_replan > REPLAN_INTERVAL:
             gx, gz = MISSIONS[m_idx]
             if nav.plan(rx, rz, gx, gz): last_replan = now

        # -- Execute
        target = nav.get_target(rx, rz)
        if not target: print("Waypoint reached"); m_idx += 1; path_planned = False; lm.setVelocity(0); rm.setVelocity(0); continue
            
        tx, ty = target
        dx_world, dy_world = tx - rx, ty - rz
        gx = dx_world * math.cos(-yaw) - dy_world * math.sin(-yaw)
        gy = dx_world * math.sin(-yaw) + dy_world * math.cos(-yaw)
        
        v, w = dwa.get_safe_velocities(process_lidar(lidar), (gx, gy), prev_cmd, prev_cmd)
        prev_cmd = (v, w)

        w_act = -w # Flip if needed
        wl = (v - w_act * AXLE_LENGTH * 0.5) / WHEEL_RADIUS
        wr = (v + w_act * AXLE_LENGTH * 0.5) / WHEEL_RADIUS
        
        limit = 6.0
        lm.setVelocity(max(-limit, min(limit, wl)))
        rm.setVelocity(max(-limit, min(limit, wr)))

if __name__ == "__main__":
    main()