from controller import Robot
import math, os, sys
from collections import deque

# ---- To Import Shared Libraries ---- #
if os.path.dirname(os.path.dirname(__file__)) not in sys.path:
    sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from lib_shared.communication import Communication
from lib_shared.global_planner import AStarPlanner
from lib_shared.map_module import visualise_robot_on_map, get_map, debug_position
from lib_shared.local_planner import DWA, _wrap

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
        self.robot_id = self.getName()
        self.state = "IDLE"
        
        # Initialise communication on channel 1
        self.comm = Communication(self, channel=1)

        # ------ Sensor & Actuator Setup ------ #
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
        # ------------------------------------- #
        
        # ---------- Planning Setup ----------- #
        self.grid = get_map(verbose=False)
        self.plan_grid = self.grid.inflate(0)
        self.astar = AStarPlanner()
        self.dwa = DWA(params=DWA_CONFIG)
        # ------------------------------------- #

       # ---------- Navigation Setup ---------- #
        self.path_world = [] 
        self.path_idx = 0
        self.current_goal = None
        self.current_task_id = None
        self.last_replan = 0.0
        self.last_vis = 0.0
        self.last_idle_broadcast = 0.0
        self.prev_cmd = (0.0, 0.0)
        self.rx = 0.0
        self.rz = 0.0
        self.yaw = 0.0
        # ------------------------------------- #
        
        print(f"[{self.robot_id}] Initialized in IDLE state")

    def update_pose(self):
        """Reads sensors to update robot's internal knowledge of position."""
        if self.gps.getSamplingPeriod() > 0:
            vals = self.gps.getValues()
            self.rx, self.rz = vals[0], vals[1]
        
        if self.imu.getSamplingPeriod() > 0:
            self.yaw = self.imu.getRollPitchYaw()[2] - math.pi/2
        elif self.compass.getSamplingPeriod() > 0:
            v = self.compass.getValues()
            self.yaw = math.atan2(v[0], v[2])

    def send_idle_status(self):
        """Broadcasts availability to spotter."""
        success = self.comm.send({
            "event": "idle",
            "collector_id": self.robot_id,
            "pos": [self.rx, self.rz]
        })
        if success:
            print(f"[{self.robot_id}] Broadcast IDLE status at ({self.rx:.2f}, {self.rz:.2f})")
        return success

    def stop_motors(self):
        """Stop both motors."""
        self.lm.setVelocity(0)
        self.rm.setVelocity(0)

    def plan_path_to_goal(self, gx, gz):
        """Uses A* to find a path from current rx,rz to gx,gz."""
        print(f"[{self.robot_id}] Planning path from ({self.rx:.2f}, {self.rz:.2f}) to ({gx:.2f}, {gz:.2f})")
        
        def valid_node(node):
            if self.plan_grid.is_free(*node): 
                return node
            # BFS to find nearest free node
            q, seen = deque([node]), {node}
            while q:
                curr = q.popleft()
                if self.plan_grid.is_free(*curr): 
                    return curr
                for dr, dc in [(0,1),(0,-1),(1,0),(-1,0)]:
                    nr, nc = curr[0]+dr, curr[1]+dc
                    if (nr, nc) not in seen and 0<=nr<self.plan_grid.height and 0<=nc<self.plan_grid.width:
                        seen.add((nr,nc))
                        q.append((nr,nc))
            return None

        start_node = valid_node(self.plan_grid.world_to_grid(self.rx, self.rz))
        goal_node = valid_node(self.plan_grid.world_to_grid(gx, gz))
        
        if not start_node or not goal_node:
            print(f"[{self.robot_id}] ERROR: Planning failed - invalid start/goal")
            return False

        nodes = self.astar.plan(self.plan_grid, start_node, goal_node)
        if nodes:
            self.path_world = [self.grid.grid_to_world(r,c) for r,c in nodes]
            self.path_idx = 0
            self.current_goal = (gx, gz)
            print(f"[{self.robot_id}] ✓ Path found with {len(nodes)} waypoints")
            return True
        
        print(f"[{self.robot_id}] ERROR: No path found")
        return False

    def update_navigation(self):
        """Executes path following. Returns True if still moving, False if arrived."""
        now = self.getTime()
        # ----------- Visualisation ----------- #
        if now - self.last_vis > 0.5:
            vis_path = [self.grid.world_to_grid(wx, wz) for wx, wz in self.path_world]
            visualise_robot_on_map(self.grid, self.rx, self.rz, self.yaw, path=vis_path)
            debug_position(self.grid, self.robot_id, self.rx, self.rz, self.yaw)
            self.last_vis = now
        # ------------------------------------- #

        if not self.path_world: return False  # No path -> arrived

        # Path Following Logic (Pure Pursuit / Lookahead)
        target = None
        while self.path_idx < len(self.path_world):
            wx, wz = self.path_world[self.path_idx]
            # If close to current waypoint, look at next
            if math.hypot(wx - self.rx, wz - self.rz) < 0.25:
                self.path_idx += 1
            else:
                # Target the next waypoint
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
            v, w = self.dwa.get_safe_velocities(
                get_lidar_points(self.lidar), 
                (gx_r, gy_r), 
                self.prev_cmd, 
                self.prev_cmd
            )
        
        # Simple Proportional Controller
        else:
            # Calculate distance and angle
            dist = math.hypot(gx_r, gy_r)
            ang = _wrap(math.atan2(gy_r, gx_r))
            
            v = 0.5 * dist
            w = -1.5 * ang
    
            # Clamp limits
            v = max(-0.35, min(0.35, v))
            w = max(-2.0, min(2.0, w))
            
        self.prev_cmd = (v, w)

        # Actuation (Differential Drive Mixing) & Clamping
        wl = (v + w * AXLE_LENGTH * 0.5) / WHEEL_RADIUS
        wr = (v - w * AXLE_LENGTH * 0.5) / WHEEL_RADIUS
        
        limit = min(self.lm.getMaxVelocity(), self.rm.getMaxVelocity())
        self.lm.setVelocity(max(-limit, min(limit, wl)))
        self.rm.setVelocity(max(-limit, min(limit, wr)))
        
        return True  # Still moving

    def handle_auction_start(self, msg):
        """Handle incoming auction announcement."""
        if self.state != "IDLE":
            return  # Only bid if idle
        
        task_id = msg.get("task_id")
        pos = msg.get("pos")
        
        if not task_id or not pos:
            print(f"[{self.robot_id}] ERROR: Invalid auction message")
            return
        
        tx, tz = pos[0], pos[1]
        
        # Calculate BID (Cost = Euclidean Distance)
        cost = math.hypot(tx - self.rx, tz - self.rz)
        
        self.comm.send({
            "event": "bid",
            "collector_id": self.robot_id,
            "task_id": task_id,
            "cost": cost
        })
        
        print(f"[{self.robot_id}] Distance {cost:.2f} to trash for task_{task_id}")

    def handle_task_assignment(self, msg):
        """Handle winning a task auction."""
        if msg.get("collector_id") != self.robot_id:
            return  # Not for us
        
        task_id = msg.get("task_id")
        target_x = msg.get("target_x")
        target_z = msg.get("target_z")
        
        if None in [task_id, target_x, target_z]:
            print(f"[{self.robot_id}] ERROR: Invalid task assignment")
            return
        
        print(f"[{self.robot_id}] assigned {task_id}!")
        print(f"[{self.robot_id}]    Target: ({target_x:.2f}, {target_z:.2f})")
        
        self.current_task_id = task_id
        
        # Plan path to target
        success = self.plan_path_to_goal(target_x, target_z)
        if success:
            self.state = "NAVIGATING"
            print(f"[{self.robot_id}] → Transitioning to NAVIGATING")
        else:
            print(f"[{self.robot_id}] Planning failed, remaining IDLE")
            self.current_task_id = None

    def handle_messages(self):
        """Process all incoming messages."""
        # Check for messages (non-blocking)
        msg = self.comm.receive()
        
        if not msg:
            return
        
        event = msg.get("event")
        
        if event == "auction_start":
            self.handle_auction_start(msg)
        
        elif event == "assign_task":
            self.handle_task_assignment(msg)
        
        else:
            # Unknown event type
            pass

    def run(self):
        print(f"\n{'='*60}")
        print(f"Collector {self.robot_id} Started")
        print(f"Mode: Auction-Based Task Assignment")
        print(f"{'='*60}\n")
        
        while self.step(self.timestep) != -1:
            # Always update pose
            self.update_pose()
            
            # Process incoming messages
            self.handle_messages()
            
            # if self.getTime() % 2.0 < 0.1: self.debug_position() # POS DEBUG (FOR MAP CALLIBRATION)
               
            # --- Finite State Machine ---
            if self.state == "IDLE":
                self.stop_motors()
                
                # Periodically broadcast idle status (every 2 seconds)
                now = self.getTime()
                if now - self.last_idle_broadcast >= 2.0:
                    self.send_idle_status()
                    self.last_idle_broadcast = now

            elif self.state == "NAVIGATING":
                is_moving = self.update_navigation()
                
                if not is_moving:
                    print(f"[{self.robot_id}] Task {self.current_task_id} COMPLETE")
                    
                    # Notify spotter of completion
                    self.comm.send({
                        "event": "collected",
                        "collector_id": self.robot_id,
                        "task_id": self.current_task_id
                    })
                    
                    # Return to idle
                    self.current_task_id = None
                    self.path_world = []
                    self.state = "IDLE"
                    print(f"[{self.robot_id}] → Transitioning to IDLE")


if __name__ == "__main__":
    Collector().run()