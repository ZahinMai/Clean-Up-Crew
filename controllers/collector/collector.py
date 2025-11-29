from controller import Robot
import math, os, sys
from collections import deque

# ---- To Import Shared Libraries ---- #
if os.path.dirname(os.path.dirname(__file__)) not in sys.path:
    sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from lib_shared.communication import Communication
from lib_shared.global_planner import AStarPlanner
from lib_shared.map_module import visualise_robot_on_map, get_map
from lib_shared.local_planner import DWA

WHEEL_RADIUS = 0.033
AXLE_LENGTH = 0.16
DWA_ENABLED = False  # To toggle DWA cuz it doesn't work

DWA_CONFIG = {
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
    
    fov = lidar.getFov()
    n = len(ranges)
    points = []
    for i, r in enumerate(ranges):
        if r > max_r or r < min_r or not math.isfinite(r): continue
        a = -fov/2 + (i/(n-1))*fov
        points.append((r*math.cos(a), r*math.sin(a)))
    return points

class Collector(Robot):
    def __init__(self):
        super().__init__()
        self.timestep = int(self.getBasicTimeStep())
        self.robot_id = "collector_1"
        self.state = "IDLE"
        
        # Commmunication Setup
        self.comm = Communication(self, channel=1)

        # Sensor & Actuator Setup
        self.gps = self.getDevice('gps')
        self.gps.enable(self.timestep)
        
        self.imu = self.getDevice('inertial unit')
        self.imu.enable(self.timestep)
        
        self.compass = self.getDevice('compass')
        self.compass.enable(self.timestep)
        
        self.lidar = self.getDevice('LDS-01')
        self.lidar.enable(self.timestep)
        self.lidar.enablePointCloud()
        
        self.lm = self.getDevice('left wheel motor')
        self.lm.setPosition(float('inf'))
        self.lm.setVelocity(0)
        
        self.rm = self.getDevice('right wheel motor')
        self.rm.setPosition(float('inf'))
        self.rm.setVelocity(0)

        # Planning Setup
        self.grid = get_map(verbose=False)
        self.plan_grid = self.grid.inflate(0) 
        self.astar = AStarPlanner()
        self.dwa = DWA(params=DWA_CONFIG)

        # Navigation Variables
        self.path_world = []     # List of (x, z) tuples
        self.path_idx = 0
        self.current_goal = None # (x, z)
        self.last_replan = 0.0
        self.last_vis = 0.0
        self.prev_cmd = (0.0, 0.0)
        
        # Robot Pose
        self.rx = 0.0
        self.rz = 0.0
        self.yaw = 0.0

    def update_pose(self):
        """Reads sensors to update robot's internal knowledge of position."""
        if self.gps.getSamplingPeriod() > 0:
            vals = self.gps.getValues()
            self.rx, self.rz = vals[0], vals[1]
        
        if self.imu.getSamplingPeriod() > 0:
            self.yaw = self.imu.getRollPitchYaw()[2]
        elif self.compass.getSamplingPeriod() > 0:
            v = self.compass.getValues()
            self.yaw = math.atan2(v[0], v[2])

    def send_idle_status(self):
        """Broadcasts availability."""
        self.comm.send({
            "event": "idle",
            "collector_id": self.robot_id,
            "pos": [self.rx, self.rz]
        })

    def stop_motors(self):
        self.lm.setVelocity(0)
        self.rm.setVelocity(0)

    def plan_path_to_goal(self, gx, gz):
        """Uses A* to find a path from current rx,rz to gx,gz."""
        def valid_node(node):
            if self.plan_grid.is_free(*node): return node
            # BFS to find nearest free node
            q, seen = deque([node]), {node}
            while q:
                curr = q.popleft()
                if self.plan_grid.is_free(*curr): return curr
                for dr, dc in [(0,1),(0,-1),(1,0),(-1,0)]:
                    nr, nc = curr[0]+dr, curr[1]+dc
                    if (nr, nc) not in seen and 0<=nr<self.plan_grid.height and 0<=nc<self.plan_grid.width:
                        seen.add((nr,nc)); q.append((nr,nc))
            return None

        start_node = valid_node(self.plan_grid.world_to_grid(self.rx, self.rz))
        goal_node = valid_node(self.plan_grid.world_to_grid(gx, gz))
        
        if not start_node or not goal_node:
            print("Plan failed: Start or Goal invalid")
            return False

        nodes = self.astar.plan(self.plan_grid, start_node, goal_node)
        if nodes:
            self.path_world = [self.grid.grid_to_world(r,c) for r,c in nodes]
            self.path_idx = 0
            self.current_goal = (gx, gz)
            print(f"Path planned to {gx}, {gz} with {len(nodes)} steps.")
            return True
        return False

    def update_navigation(self):
        """ Executes path following. Returns True if still moving, False if arrived."""
        now = self.getTime()

        # Visualisation
        if now - self.last_vis > 0.5:
            vis_path = [self.grid.world_to_grid(wx, wz) for wx, wz in self.path_world]
            visualise_robot_on_map(self.grid, self.rx, self.rz, self.yaw, path=vis_path)
            print("[Pose] Yaw: ...", self.yaw)
            self.last_vis = now

        # Path
        if not self.path_world:
            return False # No path, considered "arrived" or "idle"

        # 3. Path Following Logic (Pure Pursuit / Lookahead)
        target = None
        while self.path_idx < len(self.path_world):
            wx, wz = self.path_world[self.path_idx]
            # If close to current waypoint, look at next
            if math.hypot(wx - self.rx, wz - self.rz) < 0.2:
                self.path_idx += 1
            else:
                target = self.path_world[min(self.path_idx + 1, len(self.path_world)-1)]
                break
        
        # No target left -> end reached
        if not target:
            self.stop_motors()
            return False 

        # ------ Calculate Velocities ------
        tx, tz = target
        
        # Transform target to robot frame
        dx, dy = tx - self.rx, tz - self.rz
        gx_r = dx * math.cos(-self.yaw) - dy * math.sin(-self.yaw)
        gy_r = dx * math.sin(-self.yaw) + dy * math.cos(-self.yaw)

        # Account for dynamic obstacles w DWA
        if DWA_ENABLED:
            v, w = self.dwa.get_safe_velocities( get_lidar_points(self.lidar), (gx_r, gy_r), self.prev_cmd, self.prev_cmd)
        
        # Simple Proportional Controller
        else:
            dist = math.hypot(gx_r, gy_r)
            ang = math.atan2(gy_r, gx_r)
            v = 0.8 * dist if abs(ang) < 1.0 else 0.0
            w = 1.5 * ang
            
            # Clamp limits
            v = max(-0.35, min(0.35, v))
            w = max(-2.0, min(2.0, w))
            
        self.prev_cmd = (v, w)

        # Actuation (Differential Drive Mixing) & Clamping
        wl = (v - (-w) * AXLE_LENGTH * 0.5) / WHEEL_RADIUS
        wr = (v + (-w) * AXLE_LENGTH * 0.5) / WHEEL_RADIUS
        
        limit = min(self.lm.getMaxVelocity(), self.rm.getMaxVelocity())
        self.lm.setVelocity(max(-limit, min(limit, wl)))
        self.rm.setVelocity(max(-limit, min(limit, wr)))
        
        return True # Still moving

    def run(self):
        print("Collector Agent Started. Waiting for tasks...")
        
        while self.step(self.timestep) != -1:
            # Always update sensors
            self.update_pose()

            # --- Check Communication ---
            msg = self.comm.receive()
            if msg:
                # Filter for tasks assigned to this specific robot
                if msg.get("event") == "assign_task" and msg.get("collector_id") == self.robot_id:
                    print(f"Task Received: {msg}")
                    
                    # EXTRACT COORDINATES HERE
                    # Assuming msg contains 'target_x' and 'target_z' or similar. 
                    # If not, set hardcoded defaults or parse logic here.
                    target_x = msg.get("target_x", 3.5) 
                    target_z = msg.get("target_z", 2.0)
                    
                    # Plan path
                    success = self.plan_path_to_goal(target_x, target_z)
                    if success:
                        self.state = "NAVIGATING"
                    else:
                        print("Could not plan path to task location.")

            # --- Finite State Machine ---
            if self.state == "IDLE":
                self.stop_motors()
                # Periodically send status (every ~1 second)
                if self.getTime() % 1.0 < 0.1: 
                    self.send_idle_status()

            elif self.state == "NAVIGATING":
                # Execute move logic
                is_moving = self.update_navigation()
                
                if not is_moving:
                    print("Target Reached.")
                    # Task complete? Go back to IDLE or switch to PICKUP
                    # For now, let's go back to IDLE
                    self.state = "IDLE"
                    
            elif self.state == "GO_TO_TRASH":
                # Legacy state name handling if needed
                self.state = "NAVIGATING"

if __name__ == "__main__":
    agent = Collector()
    agent.run()