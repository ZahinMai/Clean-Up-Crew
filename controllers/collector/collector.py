# ================================================ #
#  CONTROLLER FOR ALL COLLECTORS  -> AUTHOR: ZAHIN #
# ================================================ #
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
    'DT_SIM': 0.15, 'T_PRED': 1.0, 'NV': 7, 'NW': 10, 'V_MAX': 0.35, 'W_MAX': 2.0,
    'A_V': 1.0, 'A_W': 2.0, 'RADIUS': 0.09, 'SAFE': 0.01, 'ALPHA': 0.3, 'BETA': 0.3,
    'GAMMA': 0.6, 'DELTA': 0.1, 'EPS': 0.1, 'LIDAR_SKIP': 4,
}

def get_lidar_points(lidar, max_r=3.5, min_r=0.1):
    """Return filtered (x, y) lidar points in the robot frame."""
    if not lidar: return []
    try: ranges = lidar.getRangeImage()
    except: return []
    if not ranges: return []
    fov, n = lidar.getFov(), len(ranges)
    pts = []
    for i, r in enumerate(ranges):
        if min_r <= r <= max_r and math.isfinite(r):
            a = -fov/2 + (i/(n-1))*fov
            pts.append((r*math.cos(a), r*math.sin(a)))
    return pts


class Collector(Robot):
    def __init__(self):
        """Init collector robot, sensors, comm channel, and planning modules."""
        super().__init__()
        self.timestep = int(self.getBasicTimeStep())
        self.robot_id = self.getName()
        self.state = "IDLE"
        self.comm = Communication(self, channel=1)

        # ------ Sensor & Actuator Setup ------ #
        self.gps = self.getDevice('gps'); self.gps.enable(self.timestep)
        self.imu = self.getDevice('inertial unit'); self.imu.enable(self.timestep)
        self.compass = self.getDevice('compass'); self.compass.enable(self.timestep)
        self.lidar = self.getDevice('LDS-01'); self.lidar.enable(self.timestep); self.lidar.enablePointCloud()
        self.lm = self.getDevice('left wheel motor'); self.lm.setPosition(float('inf')); self.lm.setVelocity(0)
        self.rm = self.getDevice('right wheel motor'); self.rm.setPosition(float('inf')); self.rm.setVelocity(0)
        # ------------------------------------- #

        # ---------- Planning Setup ----------- #
        self.grid = get_map(verbose=False)
        self.plan_grid = self.grid.inflate(0)
        self.astar = AStarPlanner()
        self.dwa = DWA(params=DWA_CONFIG)
        # ------------------------------------- #

        # ---------- Navigation Setup ---------- #
        self.path_world, self.path_idx = [], 0
        self.current_goal = self.current_task_id = None
        self.last_replan = self.last_vis = self.last_idle_broadcast = 0.0
        self.prev_cmd = (0.0, 0.0)
        self.rx = self.rz = self.yaw = 0.0
        # ------------------------------------- #

        print(f"[{self.robot_id}] Initialized in IDLE state")

    def update_pose(self):
        """Read GPS, IMU/compass & update internal pose (rx, rz, yaw)."""
        if self.gps.getSamplingPeriod() > 0:
            self.rx, self.rz = self.gps.getValues()[0], self.gps.getValues()[1]
        if self.imu.getSamplingPeriod() > 0:
            self.yaw = self.imu.getRollPitchYaw()[2] - math.pi/2
        elif self.compass.getSamplingPeriod() > 0:
            v = self.compass.getValues()
            self.yaw = math.atan2(v[0], v[2])

    def send_idle_status(self):
        """Broadcast collector's idle state and current location."""
        success = self.comm.send({"event": "idle", "collector_id": self.robot_id, "pos": [self.rx, self.rz]})
        if success: print(f"[{self.robot_id}] Broadcast IDLE status at ({self.rx:.2f}, {self.rz:.2f})")
        return success

    def plan_path(self, gx, gz):
        """Run A* from current pos -> (gx, gz). Return True on success."""
        print(f"[{self.robot_id}] Planning path from ({self.rx:.2f}, {self.rz:.2f}) to ({gx:.2f}, {gz:.2f})")

        def valid(n):
            """Find nearest free grid node to n using BFS."""
            if self.plan_grid.is_free(*n): return n
            q, seen = deque([n]), {n}
            while q:
                r, c = q.popleft()
                if self.plan_grid.is_free(r, c): return (r, c)
                for dr, dc in [(0,1),(0,-1),(1,0),(-1,0)]:
                    nr, nc = r+dr, c+dc
                    if 0 <= nr < self.plan_grid.height and 0 <= nc < self.plan_grid.width and (nr,nc) not in seen:
                        seen.add((nr,nc)); q.append((nr,nc))
            return None

        start = valid(self.plan_grid.world_to_grid(self.rx, self.rz))
        goal  = valid(self.plan_grid.world_to_grid(gx, gz))
        
        if not start or not goal:
            print(f"[{self.robot_id}] ERROR: Planning failed - invalid start/goal")
            return False

        nodes = self.astar.plan(self.plan_grid, start, goal)
        if nodes:
            self.path_world = [self.grid.grid_to_world(r, c) for r, c in nodes]
            self.path_idx, self.current_goal = 0, (gx, gz)
            print(f"[{self.robot_id}] ✓ Path found with {len(nodes)} waypoints")
            return True

        print(f"[{self.robot_id}] ERROR: No path found")
        return False

    def update_navigation(self):
        """Follow current path using proportional control or DWA. Return True if still moving."""
        now = self.getTime()
        if now - self.last_vis > .5:
            vis = [self.grid.world_to_grid(x, z) for x, z in self.path_world]
            visualise_robot_on_map(self.grid, self.rx, self.rz, self.yaw, path=vis)
            debug_position(self.grid, self.robot_id, self.rx, self.rz, self.yaw)
            self.last_vis = now

        if not self.path_world: return False
        target = None

        while self.path_idx < len(self.path_world):
            wx, wz = self.path_world[self.path_idx]
            if math.hypot(wx - self.rx, wz - self.rz) < .25: self.path_idx += 1
            else:
                target = self.path_world[min(self.path_idx + 1, len(self.path_world)-1)]
                break

        if not target:
            self.lm.setVelocity(0); self.rm.setVelocity(0)
            return False

        dx, dy = target[0] - self.rx, target[1] - self.rz
        gx_r = dx*math.cos(-self.yaw) - dy*math.sin(-self.yaw)
        gy_r = dx*math.sin(-self.yaw) + dy*math.cos(-self.yaw)

        if DWA_ENABLED:
            v, w = self.dwa.get_safe_velocities(get_lidar_points(self.lidar), (gx_r, gy_r), self.prev_cmd, self.prev_cmd)
        else:
            dist = math.hypot(gx_r, gy_r)
            ang = _wrap(math.atan2(gy_r, gx_r))
            v = max(-.35, min(.35, .5*dist))
            w = max(-2.0, min(2.0, -1.5*ang))

        self.prev_cmd = (v, w)
        wl = (v + w*AXLE_LENGTH*.5)/WHEEL_RADIUS
        wr = (v - w*AXLE_LENGTH*.5)/WHEEL_RADIUS
        limit = min(self.lm.getMaxVelocity(), self.rm.getMaxVelocity())
        self.lm.setVelocity(max(-limit, min(limit, wl)))
        self.rm.setVelocity(max(-limit, min(limit, wr)))
        return True

    def handle_auction_start(self, msg):
        """Respond to auction announcements by submitting a bid when idle."""
        if self.state != "IDLE": return
        task_id, pos = msg.get("task_id"), msg.get("pos")
        if not task_id or not pos:
            print(f"[{self.robot_id}] ERROR: Invalid auction message"); return
        tx, tz = pos
        cost = math.hypot(tx - self.rx, tz - self.rz)
        self.comm.send({"event": "bid", "collector_id": self.robot_id, "task_id": task_id, "cost": cost})
        print(f"[{self.robot_id}] Distance {cost:.2f} to trash for task_{task_id}")

    def handle_task_assignment(self, msg):
        """Accept assigned task, plan the path, and switch to NAVIGATING."""
        if msg.get("collector_id") != self.robot_id: return
        task_id, x, z = msg.get("task_id"), msg.get("target_x"), msg.get("target_z")
        if None in [task_id, x, z]:
            print(f"[{self.robot_id}] ERROR: Invalid task assignment"); return

        print(f"[{self.robot_id}] assigned {task_id}!")
        print(f"[{self.robot_id}]    Target: ({x:.2f}, {z:.2f})")
        self.current_task_id = task_id

        if self.plan_path(x, z):
            self.state = "NAVIGATING"
            print(f"[{self.robot_id}] → Transitioning to NAVIGATING")
        else:
            print(f"[{self.robot_id}] Planning failed, remaining IDLE")
            self.current_task_id = None

    def handle_messages(self):
        """Receive and dispatch incoming communication events."""
        msg = self.comm.receive()
        if not msg: return
        event = msg.get("event")
        if   event == "auction_start": self.handle_auction_start(msg)
        elif event == "assign_task":   self.handle_task_assignment(msg)

    def run(self):
        """Main robot loop: handle messages, manage FSM, navigation, and task completion."""
        print(f"\n{'='*60}\nCollector {self.robot_id} Started\nMode: Auction-Based Task Assignment\n{'='*60}\n")
        
        while self.step(self.timestep) != -1:
            self.update_pose()
            self.handle_messages()

            if self.state == "IDLE":
                self.lm.setVelocity(0); self.rm.setVelocity(0)
                now = self.getTime()
                if now - self.last_idle_broadcast >= 2.0:
                    self.send_idle_status(); self.last_idle_broadcast = now

            elif self.state == "NAVIGATING":
                if not self.update_navigation():
                    print(f"[{self.robot_id}] Task {self.current_task_id} COMPLETE")
                    self.comm.send({"event": "collected", "collector_id": self.robot_id, "task_id": self.current_task_id})
                    self.current_task_id, self.path_world, self.state = None, [], "IDLE"
                    print(f"[{self.robot_id}] → Transitioning to IDLE")


if __name__ == "__main__":
    Collector().run()
