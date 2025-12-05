# Spotter Controller (Integrated Dispatcher)
# Roles: Coverage Search, Vision Detection, Task Auctioneer
# Implemented by: Abdullateef Vahora (Logic merged with Dispatcher)

import sys, os, math
from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict

from controller import Robot

# --- Import Shared Libraries ---
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

from lib_shared import vision
from spotter_coverage import CoveragePlanner
from lib_shared.communication import Communication
from lib_shared.global_planner import AStarPlanner
from lib_shared.local_planner import DWA
from lib_shared.map_module import get_map

# --- Configuration ---
@dataclass(frozen=True)
class RobotConfig:
    # Hardware
    WHEEL_RADIUS: float = 0.0205
    AXLE_LENGTH: float = 0.052
    MAX_WHEEL_OMEGA: float = 6.28
    
    # Sensors & Planning
    LIDAR_MAX_RANGE: float = 2.0
    LIDAR_SELF_COLLISION: float = 0.05
    WAYPOINT_TOLERANCE: float = 0.20
    PATH_NODE_TOLERANCE: float = 0.25
    SMOOTHING_FACTOR: float = 0.6
    LAWNMOWER_STEP: int = 4
    
    # Auction Settings
    AUCTION_DURATION: float = 2.0  # Seconds to wait for bids

# --- Hardware Abstraction ---
class HardwareInterface:
    def __init__(self, robot: Robot, config: RobotConfig):
        self.robot = robot
        self.cfg = config
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # Sensors
        self.gps = self.robot.getDevice("gps")
        self.gps.enable(self.timestep)
        self.compass = self.robot.getDevice("compass")
        self.compass.enable(self.timestep)
        self.camera = self.robot.getDevice("camera")
        self.camera.enable(self.timestep)
        self.lidar = self.robot.getDevice("lidar")
        if self.lidar:
            self.lidar.enable(self.timestep)
            self.lidar_fov = self.lidar.getFov()

        # Motors
        self.lm = self.robot.getDevice('left wheel motor')
        self.rm = self.robot.getDevice('right wheel motor')
        self.lm.setPosition(float('inf'))
        self.rm.setPosition(float('inf'))
        self.set_velocities(0, 0)

    def step(self):
        return self.robot.step(self.timestep)

    def get_time(self):
        return self.robot.getTime()

    def get_pose(self) -> Tuple[float, float, float]:
        """Returns (x, y, yaw)"""
        values = self.gps.getValues()
        cv = self.compass.getValues()
        yaw = math.atan2(cv[0], cv[1]) if cv else 0.0
        return values[0], values[1], yaw

    def get_lidar_points(self) -> List[Tuple[float, float]]:
        """Returns valid obstacle points in robot frame"""
        ranges = self.lidar.getRangeImage()
        if not ranges or len(ranges) < 2: return []

        points = []
        angle_step = self.lidar_fov / (len(ranges) - 1)
        cur_angle = -self.lidar_fov / 2.0

        for r in ranges:
            if self.cfg.LIDAR_SELF_COLLISION < r < self.cfg.LIDAR_MAX_RANGE:
                points.append((r * math.cos(cur_angle), r * math.sin(cur_angle)))
            cur_angle += angle_step
        return points

    def is_trash_visible(self):
        return vision.is_trash_visible(self.camera)

    def get_trash_world_pos(self) -> Tuple[float, float]:
        """Calculates approximate world position of trash detected ahead."""
        rx, ry, yaw = self.get_pose()
        # Assume trash is ~30cm in front of camera center
        tx = rx + 0.3 * math.cos(yaw)
        ty = ry + 0.3 * math.sin(yaw)
        return round(tx, 2), round(ty, 2)

    def set_velocities(self, v: float, w: float):
        wl = (v - w * self.cfg.AXLE_LENGTH / 2) / self.cfg.WHEEL_RADIUS
        wr = (v + w * self.cfg.AXLE_LENGTH / 2) / self.cfg.WHEEL_RADIUS
        limit = self.cfg.MAX_WHEEL_OMEGA
        self.lm.setVelocity(max(-limit, min(limit, wl)))
        self.rm.setVelocity(max(-limit, min(limit, wr)))

# --- Main Logic ---
class SpotterController:
    def __init__(self):
        print("Initializing Spotter (Manager) Controller.")
        self.config = RobotConfig()
        self.hw = HardwareInterface(Robot(), self.config)
        self.comm = Communication(self.hw.robot, channel=1)
        
        # Navigation
        self.grid = get_map()
        self.planner = AStarPlanner()
        self.dwa = DWA({
            "RADIUS": 0.035, "SAFE": 0.015, "V_MAX": 0.12, 
            "ALPHA": 0.8, "BETA": 0.4, "GAMMA": 0.3, "EPS": 0.1
        })

        # Coverage Mission
        print("Generating Coverage Path...")
        coverage = CoveragePlanner(self.grid, step=self.config.LAWNMOWER_STEP)
        self.mission_waypoints = coverage.generate_lawnmower_path()
        self.wp_idx = 0
        self.active_path = []
        self.prev_cmd = (0.0, 0.0)

        # State Management
        self.state = "PATROL" # States: PATROL, AUCTION
        self.handled_trash_locations = set() # To avoid re-detecting same trash
        
        # Auction Data
        self.current_auction = None 

    def run(self):
        # Stabilize sensors
        for _ in range(10): self.hw.step()
        
        print(f"Spotter Started. Mission Length: {len(self.mission_waypoints)} waypoints.")

        while self.hw.step() != -1:
            # 1. Handle Incoming Messages (Bids)
            self.process_incoming_messages()

            # 2. State Machine
            if self.state == "PATROL":
                self.run_patrol_logic()
            elif self.state == "AUCTION":
                self.run_auction_logic()

    def process_incoming_messages(self):
        """Checks for bids or task completions."""
        msg = self.comm.receive()
        if not msg: return

        # Handle Bids
        if self.state == "AUCTION" and msg.get("event") == "bid":
            if self.current_auction and msg["task_id"] == self.current_auction["id"]:
                print(f"   -> Bid received: Bot {msg['collector_id']} (Cost {msg['cost']:.2f})")
                self.current_auction["bids"].append(msg)
        
        # Handle Task Completion (Optional: Clear from memory if needed)
        elif msg.get("event") == "collected":
            print(f"Collector {msg.get('collector_id')} finished a task.")

    def run_patrol_logic(self):
        """Standard behaviour: Follow lawnmower path, look for trash."""
        rx, ry, yaw = self.hw.get_pose()
        
        # -- A. Check for Trash --
        if self.hw.is_trash_visible():
            tx, ty = self.hw.get_trash_world_pos()
            
            # Check if this specific location was already handled (Euclidean dist < 0.5m)
            is_new = True
            for (hx, hy) in self.handled_trash_locations:
                if math.hypot(tx - hx, ty - hy) < 0.5:
                    is_new = False
                    break
            
            if is_new:
                print(f"*** TRASH DETECTED at ({tx}, {ty}) ***")
                self.start_auction(tx, ty)
                return # Exit patrol logic immediately

        # -- B. Navigation (Waypoint Following) --
        if self.wp_idx >= len(self.mission_waypoints):
            self.hw.set_velocities(0, 0) # Mission Complete
            return

        goal = self.mission_waypoints[self.wp_idx]

        # 1. Check if Waypoint Reached
        if math.hypot(goal[0] - rx, goal[1] - ry) < self.config.WAYPOINT_TOLERANCE:
            self.wp_idx += 1
            self.active_path = [] # Force replan
            return

        # 2. Plan Path if needed
        if not self.active_path:
            start_node = self.grid.world_to_grid(rx, ry)
            end_node = self.grid.world_to_grid(*goal)
            
            # Simple check if start/end are valid
            if self.grid.is_free(*start_node):
                nodes = self.planner.plan(self.grid, start_node, end_node)
                if nodes:
                    nodes = self.planner.smooth_path(self.grid, nodes)
                    self.active_path = [self.grid.grid_to_world(r, c) for r, c in nodes]
                else:
                    print(f"Cannot reach waypoint {self.wp_idx}. Skipping.")
                    self.wp_idx += 1
            else:
                # Robot is stuck in wall/obstacle?
                print("Robot stuck (off-grid).") 

        # 3. Follow Path
        target = goal
        if self.active_path:
            target = self.active_path[0]
            # Pop path node if reached
            if math.hypot(target[0] - rx, target[1] - ry) < self.config.PATH_NODE_TOLERANCE:
                self.active_path.pop(0)
                if self.active_path: target = self.active_path[0]

        # 4. Move (DWA Local Planner)
        gx, gy = self.world_to_robot(rx, ry, yaw, *target)
        lidar_pts = self.hw.get_lidar_points()
        
        v_dwa, w_dwa = self.dwa.get_safe_velocities(lidar_pts, (gx, gy), prev_cmd=self.prev_cmd)
        
        # Smooth velocity
        alpha = self.config.SMOOTHING_FACTOR
        v = alpha * self.prev_cmd[0] + (1.0 - alpha) * v_dwa
        w = alpha * self.prev_cmd[1] + (1.0 - alpha) * w_dwa
        
        self.prev_cmd = (v, w)
        self.hw.set_velocities(v, w)

    def run_auction_logic(self):
        """Waits for bids and assigns the task."""
        # Stop moving while auctioning
        self.hw.set_velocities(0, 0)
        self.prev_cmd = (0, 0)
        
        if not self.current_auction:
            self.state = "PATROL"
            return

        # Check Timeout
        now = self.hw.get_time()
        elapsed = now - self.current_auction["start_time"]
        
        if elapsed > self.config.AUCTION_DURATION:
            bids = self.current_auction["bids"]
            task_id = self.current_auction["id"]
            pos = self.current_auction["pos"]
            
            if bids:
                # 1. Determine Winner (Lowest Cost)
                winner = min(bids, key=lambda x: x["cost"])
                print(f"[AUCTION] Winner: {winner['collector_id']} with cost {winner['cost']:.2f}")
                
                # 2. Send Assignment
                self.comm.send({
                    "event": "assign_task",
                    "collector_id": winner['collector_id'],
                    "task_id": task_id,
                    "target_x": pos[0],
                    "target_z": pos[1]
                })
                
                # 3. Mark as handled so we don't auction it again immediately
                self.handled_trash_locations.add(pos)
                
            else:
                print(f"[AUCTION] No bids for {task_id}. Ignoring trash for now.")
                # We add it to handled list to prevent infinite loop of re-auctioning same trash 
                # immediately. Ideally, we would add a 'cool-down' list.
                self.handled_trash_locations.add(pos)

            # 4. Reset and Resume Patrol
            self.current_auction = None
            self.state = "PATROL"
            print("Resuming Patrol.")

    def start_auction(self, tx, ty):
        """Initiates the auction process."""
        task_id = f"task_{int(self.hw.get_time() * 100)}"
        print(f"[AUCTION] Opening for {task_id} at ({tx}, {ty})")
        
        self.current_auction = {
            "id": task_id,
            "pos": (tx, ty),
            "start_time": self.hw.get_time(),
            "bids": []
        }
        
        # Broadcast
        self.comm.send({
            "event": "auction_start",
            "task_id": task_id,
            "pos": (tx, ty)
        })
        
        self.state = "AUCTION"

    def world_to_robot(self, rx, ry, yaw, tx, ty):
        """Helper for DWA local coordinates."""
        dx, dy = tx - rx, ty - ry
        gx = dx * math.cos(-yaw) - dy * math.sin(-yaw)
        gy = dx * math.sin(-yaw) + dy * math.cos(-yaw)
        return gx, gy

if __name__ == "__main__":
    SpotterController().run()