# =============================================================================
# COLLECTOR ROBOT - Auction-Based Task Assignment with A* Navigation
# Author: ZAHIN
# =============================================================================
# Multi-agent collector that participates in auction system, uses A* for
# global planning, optional DWA for local avoidance. FSM: IDLE ↔ NAVIGATING
# =============================================================================

from controller import Robot
import math, os, sys
from collections import deque

# ---- To Import Shared Libraries ---- #
if os.path.dirname(os.path.dirname(__file__)) not in sys.path:
    sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from lib_shared.communication import Communication
from lib_shared.global_planner import AStarPlanner
from lib_shared.map_module import visualise_robot_on_map, get_map
from lib_shared.local_planner import DWA, _wrap
from lib_shared.dual_logger import Logger

# =============================================================================
# CONFIGURATION & UTILITIES
# =============================================================================

WHEEL_RADIUS = 0.0325
AXLE_LENGTH = 0.16
DWA_ENABLED = False  # Toggle for DWA vs simple controller

DWA_CONFIG = {
    'DT_SIM': 0.25, 'T_PRED': 1.0, 'NV': 7, 'NW': 10, 'V_MAX': 0.35, 'W_MAX': 2.0,
    'A_V': 1.0, 'A_W': 2.0, 'RADIUS': 0.09, 'SAFE': 0.15, 'ALPHA': 0.5, 'BETA': 0.3,
    'GAMMA': 0.1, 'DELTA': 0.1, 'EPS': 0.1, 'LIDAR_SKIP': 5,
}

def get_lidar_points(lidar, max_r=3.5, min_r=0.1):
    """Return filtered (x, y) lidar points in robot frame."""
    if not lidar: return []
    try: ranges = lidar.getRangeImage()
    except: return []
    if not ranges: return []
    
    fov, n = lidar.getFov(), len(ranges)
    return [(r*math.cos(-fov/2 + i/(n-1)*fov), r*math.sin(-fov/2 + i/(n-1)*fov))
            for i, r in enumerate(ranges) if min_r <= r <= max_r and math.isfinite(r)]

# =============================================================================
# COLLECTOR ROBOT
# =============================================================================

class Collector(Robot):
    """Auction-based collector robot with A* navigation."""
    def __init__(self):
        super().__init__()
        
        # Basic setup
        self.timestep = int(self.getBasicTimeStep())
        self.robot_id = self.getName()
        self.state = "IDLE"
        
        # Setup logging
        self.logger = Logger(prefix=self.robot_id, enabled=True)
        self.logger.start()
        
        # Communication
        self.comm = Communication(self, channel=1)

        # ------ Sensor & Actuator Setup ------ #
        self.gps = self.getDevice('gps'); self.gps.enable(self.timestep)
        self.imu = self.getDevice('inertial unit'); self.imu.enable(self.timestep)
        self.lidar = self.getDevice('LDS-01'); self.lidar.enable(self.timestep); self.lidar.enablePointCloud()
        self.lm = self.getDevice('left wheel motor'); self.lm.setPosition(float('inf')); self.lm.setVelocity(0)
        self.rm = self.getDevice('right wheel motor'); self.rm.setPosition(float('inf')); self.rm.setVelocity(0)

        # ---------- Planning Setup ----------- #
        self.grid = get_map(verbose=False)
        self.plan_grid = self.grid.inflate(0)
        self.astar = AStarPlanner()
        self.dwa = DWA(params=DWA_CONFIG)

        # ---------- Navigation Setup ---------- #
        self.path_world, self.path_idx = [], 0
        self.current_goal = self.current_task_id = None
        self.last_replan = self.last_vis = self.last_idle_broadcast = 0.0
        self.prev_cmd = (0.0, 0.0)
        self.rx = self.ry = self.yaw = 0.0

        print(f"Initialised in IDLE state")

    def update_pose(self):
        """Update robot pose from sensors."""
        if self.gps.getSamplingPeriod() > 0:
            self.rx, self.ry = self.gps.getValues()[:2]
        if self.imu.getSamplingPeriod() > 0:
            self.yaw = self.imu.getRollPitchYaw()[2]

    def send_idle_status(self):
        """Broadcast idle state and current location."""
        success = self.comm.send({"event": "idle", "collector_id": self.robot_id, "pos": [self.rx, self.ry]})
        if success: print(f"Broadcast IDLE at ({self.rx:.2f}, {self.ry:.2f})")
        return success

    def find_nearest_free(self, node):
        """BFS to find nearest free grid cell."""
        if self.plan_grid.is_free(*node): return node
        q, seen = deque([node]), {node}
        while q:
            r, c = q.popleft()
            if self.plan_grid.is_free(r, c): return (r, c)
            for dr, dc in [(0,1),(0,-1),(1,0),(-1,0)]:
                nr, nc = r+dr, c+dc
                if 0 <= nr < self.plan_grid.height and 0 <= nc < self.plan_grid.width and (nr,nc) not in seen:
                    seen.add((nr,nc)); q.append((nr,nc))
        return None

    def plan_path(self, gx, gz):
        """Plan A* path from current position to goal."""
        print(f"Planning: ({self.rx:.2f},{self.ry:.2f}) → ({gx:.2f},{gz:.2f})")
        
        start = self.find_nearest_free(self.plan_grid.world_to_grid(self.rx, self.ry))
        goal = self.find_nearest_free(self.plan_grid.world_to_grid(gx, gz))
        
        if not start or not goal:
            print(f"ERROR: Invalid start/goal"); return False

        nodes = self.astar.plan(self.plan_grid, start, goal)
        if nodes:
            self.path_world = [self.grid.grid_to_world(r, c) for r, c in nodes]
            self.path_idx, self.current_goal = 0, (gx, gz)
            print(f"✓ Path found: {len(nodes)} waypoints")
            return True
        
        print(f"ERROR: No path found"); return False

    def get_min_obstacle_distance(self, lidar_pts):
        """Get minimum distance to nearest obstacle."""
        if not lidar_pts: return float('inf')
        return min(math.hypot(x, y) for x, y in lidar_pts)

    def update_navigation(self):
        """Execute path following. Returns True if still moving."""
        now = self.getTime()
        
        # Visualize path periodically (collector_2 only)
        if now - self.last_vis > 2:
            vis = [self.grid.world_to_grid(x, z) for x, z in self.path_world]
            visualise_robot_on_map(self.grid, self.rx, self.ry, self.yaw, path=vis)
            self.last_vis = now

        if not self.path_world: return False

        # Find next target waypoint
        target = None
        while self.path_idx < len(self.path_world):
            wx, wz = self.path_world[self.path_idx]
            if math.hypot(wx - self.rx, wz - self.ry) < .1: 
                self.path_idx += 1
            else:
                target = self.path_world[self.path_idx]
                break

        if not target:
            self.lm.setVelocity(0); self.rm.setVelocity(0)
            return False

        # Transform target to robot frame
        dx, dy = target[0] - self.rx, target[1] - self.ry
        gx_r = dx*math.cos(-self.yaw) - dy*math.sin(-self.yaw)
        gy_r = dx*math.sin(-self.yaw) + dy*math.cos(-self.yaw)

        # Get lidar data and obstacle distance
        lidar_pts = get_lidar_points(self.lidar)
        min_obs_dist = self.get_min_obstacle_distance(lidar_pts)

        # Compute velocities
        if DWA_ENABLED:
            v, w = self.dwa.get_safe_velocities(lidar_pts, (gx_r, gy_r), self.prev_cmd, self.prev_cmd)
        else:
            dist, ang = math.hypot(gx_r, gy_r), _wrap(math.atan2(gy_r, gx_r))
            v = max(-.35, min(.35, .5*dist))
            w = max(-2.0, min(2.0, -2.5*ang))
            
            # Slow down near obstacles
            if min_obs_dist < 0.09:
                # closer the obstacle, slower the robot
                v *= min_obs_dist
            
            # Turn in place for large angle errors
            if abs(ang) > 0.5:
                v *= 0.3; w *= 1.5

        # Set wheel velocities
        self.prev_cmd = (v, w)
        wl = (v + w*AXLE_LENGTH*.5)/WHEEL_RADIUS
        wr = (v - w*AXLE_LENGTH*.5)/WHEEL_RADIUS
        limit = min(self.lm.getMaxVelocity(), self.rm.getMaxVelocity())
        self.lm.setVelocity(max(-limit, min(limit, wl)))
        self.rm.setVelocity(max(-limit, min(limit, wr)))
        return True

    def handle_auction_start(self, msg):
        """Submit bid if idle."""
        if self.state != "IDLE": return
        task_id, pos = msg.get("task_id"), msg.get("pos")
        if not task_id or not pos:
            print(f"ERROR: Invalid auction message"); return
        
        tx, ty = pos
        cost = math.hypot(tx - self.rx, ty - self.ry)
        self.comm.send({"event": "bid", "collector_id": self.robot_id, "task_id": task_id, "cost": cost})
        print(f"Bid {cost:.2f} for task_{task_id}")

    def handle_task_assignment(self, msg):
        """Accept task and transition to NAVIGATING."""
        if msg.get("collector_id") != self.robot_id: return
        
        task_id, x, z = msg.get("task_id"), msg.get("target_x"), msg.get("target_z")
        if None in [task_id, x, z]:
            print(f"ERROR: Invalid task assignment"); return

        print(f"Assigned {task_id} → ({x:.2f}, {z:.2f})")
        self.current_task_id = task_id

        if self.plan_path(x, z):
            self.state = "NAVIGATING"
        else:
            print(f"Planning failed, staying IDLE")
            self.current_task_id = None

    def handle_messages(self):
        """Process incoming messages."""
        msg = self.comm.receive()
        if not msg: return
        
        event = msg.get("event")
        if event == "auction_start": self.handle_auction_start(msg)
        elif event == "assign_task": self.handle_task_assignment(msg)


    def run(self):
        """Main control loop with finite state machine."""
        print(f"\n{'='*60}\nCollector Started\nMode: Auction-Based Task Assignment\n{'='*60}\n")
        
        try:
            while self.step(self.timestep) != -1:
                self.update_pose()
                self.handle_messages()
                
                if self.state == "IDLE":
                    # Stop and broadcast availability
                    self.lm.setVelocity(0); self.rm.setVelocity(0)
                    now = self.getTime()
                    if now - self.last_idle_broadcast >= 2.0:
                        self.send_idle_status(); self.last_idle_broadcast = now

                elif self.state == "NAVIGATING":
                    # Execute navigation
                    if not self.update_navigation():
                        print(f"Task {self.current_task_id} COMPLETE")
                        self.comm.send({"event": "collected", "collector_id": self.robot_id, "task_id": self.current_task_id})
                        self.current_task_id, self.path_world, self.state = None, [], "IDLE"
        
        finally: self.logger.stop()

if __name__ == "__main__":
    Collector().run()