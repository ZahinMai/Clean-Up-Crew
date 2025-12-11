# Spotter controller (Setup 3)
# Responsible for finding trash using an efficient search pattern.
# Uses: navigation, obstacle_avoidance, vision, communication, auctioneer
# Implemented by: Abdullateef Vahora
# Auction integration by: Kunal (comm module) + merged test_spotter_comm logic

import sys
import os
import math
from dataclasses import dataclass
from typing import List, Tuple, Dict, Set

from controller import Robot, Keyboard

project_root = os.getcwd()
controllers_root = os.path.dirname(project_root)
sys.path.append(controllers_root)

from controllers.auctioneer.archive import vision
from lib_shared.coverage import CoveragePlanner
from lib_shared.communication import Communication
from lib_shared.global_planner import AStarPlanner
from lib_shared.local_planner import _wrap
from lib_shared.map_module import get_map


# CONFIG

@dataclass(frozen=True)
class RobotConfig:
    WHEEL_RADIUS: float = 0.0205
    AXLE_LENGTH: float = 0.052
    MAX_WHEEL_OMEGA: float = 6.28

    LIDAR_MAX_RANGE: float = 2.0
    LIDAR_SELF_COLLISION: float = 0.05

    WAYPOINT_TOLERANCE: float = 0.20
    PATH_NODE_TOLERANCE: float = 0.25
    SMOOTHING_FACTOR: float = 0.6

    LAWNMOWER_STEP: int = 4



# HARDWARE LAYER
class HardwareInterface:
    def __init__(self, robot: Robot, config: RobotConfig):
        self.robot = robot
        self.cfg = config
        self.timestep = int(self.robot.getBasicTimeStep())

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
        """Returns current robot state (x, y, yaw)."""
        values = self.gps.getValues()
        compass_values = self.compass.getValues()
        yaw = math.atan2(compass_values[0], compass_values[1]) if compass_values else 0.0
        return values[0], values[1], yaw

    def get_lidar_points(self) -> List[Tuple[float, float]]:
        """Converts valid lidar ranges to robot frame coordinates."""
        if not self.lidar:
            return []

        ranges = self.lidar.getRangeImage()
        if not ranges or len(ranges) < 2:
            return []

        points: List[Tuple[float, float]] = []
        angle_step = self.lidar_fov / (len(ranges) - 1)
        current_angle = -self.lidar_fov / 2.0

        for r in ranges:
            if self.cfg.LIDAR_SELF_COLLISION < r < self.cfg.LIDAR_MAX_RANGE:
                lx = r * math.cos(current_angle)
                ly = r * math.sin(current_angle)
                points.append((lx, ly))
            current_angle += angle_step

        return points

    def is_trash_visible(self):
        return vision.is_trash_visible(self.camera)

    def report_trash(self, comm: Communication, reported_trash: Set[Tuple[float, float]]):
        """Calculate trash coordinates 30cm ahead and broadcast once per spot."""
        rx, ry, yaw = self.get_pose()
        tx = round((rx + 0.3 * math.cos(yaw)), 2)
        ty = round((ry + 0.3 * math.sin(yaw)), 2)

        # Ignore if already reported nearby
        for (ex, ey) in reported_trash:
            if math.hypot(tx - ex, ty - ey) < 0.5:
                return

        print(f"[SPOTTER] NEW TRASH FOUND at ({tx}, {ty})")
        comm.send({
            "event": "trash_found",
            "pos": (tx, ty)
        })
        reported_trash.add((tx, ty))

    def set_velocities(self, v: float, w: float):
        wl = (v - w * self.cfg.AXLE_LENGTH / 2) / self.cfg.WHEEL_RADIUS
        wr = (v + w * self.cfg.AXLE_LENGTH / 2) / self.cfg.WHEEL_RADIUS

        wl = max(-self.cfg.MAX_WHEEL_OMEGA, min(self.cfg.MAX_WHEEL_OMEGA, wl))
        wr = max(-self.cfg.MAX_WHEEL_OMEGA, min(self.cfg.MAX_WHEEL_OMEGA, wr))

        self.lm.setVelocity(wl)
        self.rm.setVelocity(wr)


# MAIN CONTROLLER (COVERAGE + AUCTIONEER)

class SpotterController:
    def __init__(self):
        print("Initializing Spotter Controller (coverage + auctioneer).")
        self.config = RobotConfig()
        self.hardware = HardwareInterface(Robot(), self.config)

        # Coverage / navigation stack
        self.grid = get_map()
        self.planner = AStarPlanner()
        self.comm = Communication(self.hardware.robot)

        coverage_planner = CoveragePlanner(self.grid, step=self.config.LAWNMOWER_STEP)
        self.mission_waypoints = coverage_planner.generate_lawnmower_path()
        self.waypoints_idx = 0
        self.active_path: List[Tuple[float, float]] = []
        self.prev_cmd: Tuple[float, float] = (0.0, 0.0)

        self.reported_trash: Set[Tuple[float, float]] = set()

        #  AUCTION STATE (from test_spotter_comm) 
        self.manual = False  # keep for completeness
        self.keyboard = Keyboard()
        self.keyboard.enable(self.hardware.timestep)

        self.test_phase = 0
        self.start_time = 0.0
        self.spotters_idle: Dict[str, Tuple[float, float]] = {}
        self.spotters_busy: Set[str] = set()
        self.bids_received: Dict[int, List[Tuple[str, float]]] = {}
        self.current_task_id = 0
        self.tasks_completed = 0
        self.pending_assignment = None  # (task_id, location_idx) or None

        # Same test locations as auction tester
        # (x, z, description)
        self.test_locations: List[Tuple[float, float, str]] = [
            (1.5, 2.0, "Sanity Check (Near)"),
            (-4.0, 3.0, "Navigation (Behind Tables)"),
            (2.5, -2.5, "Human Avoidance Zone"),
            (-2.5, 3.5, "Multi-Agent Task A"),
            (2.5, 3.5, "Multi-Agent Task B"),
        ]

        self.print_header()

    
    # UTILS
    
    def world_to_robot(self, rx, ry, yaw, tx, ty):
        dx, dy = tx - rx, ty - ry
        return (dx * math.cos(-yaw) - dy * math.sin(-yaw),
                dx * math.sin(-yaw) + dy * math.cos(-yaw))

   
    # AUCTION COMMUNICATION WRAPPERS
    
    def send_message(self, msg: dict):
        self.comm.send(msg)

    def receive_messages(self) -> List[dict]:
        return self.comm.receive_all()

   
    # MESSAGE PROCESSING
  
    def process_idle(self, msg: dict):
        collector_id = msg.get("collector_id")
        pos = msg.get("pos")

        if collector_id and pos:
            if collector_id in self.spotters_busy:
                self.spotters_busy.remove(collector_id)
            self.spotters_idle[collector_id] = tuple(pos)
            print(f"  ✓ {collector_id} IDLE at ({pos[0]:.2f}, {pos[1]:.2f})")

    def process_bid(self, msg: dict):
        collector_id = msg.get("collector_id")
        task_id = msg.get("task_id")
        cost = msg.get("cost")

        if task_id is not None:
            if task_id not in self.bids_received:
                self.bids_received[task_id] = []
            self.bids_received[task_id].append((collector_id, cost))
            print(f"  → BID {collector_id}: {cost:.2f} for task {task_id}")

    def process_complete(self, msg: dict):
        collector_id = msg.get("collector_id")
        print(f"  ✅ {collector_id} COMPLETED task")

        self.tasks_completed += 1
        if collector_id in self.spotters_busy:
            self.spotters_busy.remove(collector_id)

   
    # AUCTION LOGIC
    
    def start_auction(self, idx: int | None = None):
        """Start new auction for target location."""
        if idx is None:
            idx = self.current_task_id % len(self.test_locations)

        x, z, name = self.test_locations[idx]
        self.current_task_id += 1
        task_id = self.current_task_id

        print(f"\n{'─'*70}")
        print(f"*AUCTION #{task_id}*: {name} at ({x:.2f}, {z:.2f})")
        print(f"{'─'*70}")

        self.send_message({
            "event": "auction_start",
            "task_id": task_id,
            "pos": [x, z],
        })

        self.bids_received[task_id] = []
        self.pending_assignment = (task_id, idx)

    def assign_task(self, task_id: int):
        """Assign task to lowest bidder."""
        if task_id not in self.bids_received or not self.bids_received[task_id]:
            print("  No bids received")
            return None

        bids = self.bids_received[task_id]
        winner_id, winner_cost = min(bids, key=lambda x: x[1])

        print(f"  Lowest path cost {winner_cost:.2f} ({winner_id})")
        print("  All bids:")
        for sid, cost in sorted(bids, key=lambda x: x[1]):
            marker = "→" if sid == winner_id else " "
            print(f"   {marker} {sid}: {cost:.2f}")

        _, idx = self.pending_assignment
        x, z, _ = self.test_locations[idx]

        self.send_message({
            "event": "assign_task",
            "collector_id": winner_id,
            "task_id": task_id,
            "target_x": x,
            "target_z": z,
        })

        self.spotters_busy.add(winner_id)
        if winner_id in self.spotters_idle:
            del self.spotters_idle[winner_id]
        self.pending_assignment = None

        return winner_id

   
    # DISPLAY / DEBUG
   
    def print_header(self):
        print("\n" + "=" * 70)
        print("SPOTTER AUCTION TEST (integrated into spotter controller)")
        if self.manual:
            print("Controls: [A] Auction [S] Status [R] Reset [Q] Quit")
        print("=" * 70 + "\n")

    def print_status(self):
        t = self.hardware.get_time()
        print(f"\n{'='*70}")
        print(f"STATUS (t={t:.1f}s)")
        print(f"{'='*70}")
        print(f"Idle robots: {len(self.spotters_idle)}")
        print(f"Busy robots: {len(self.spotters_busy)}")
        print(f"Auctions run: {self.current_task_id}")
        print(f"Tasks completed: {self.tasks_completed}")
        if self.current_task_id > 0:
            rate = self.tasks_completed / self.current_task_id * 100.0
            print(f"Success rate: {rate:.1f}%")
        print(f"{'='*70}\n")

   
    # OPTIONAL MANUAL MODE
    
    def handle_keyboard(self):
        key = self.keyboard.getKey()
        if key == ord('A'):
            self.start_auction()
        elif key == ord('S'):
            self.print_status()
        elif key == ord('R'):
            print("Reset auction stats")
            self.current_task_id = 0
            self.tasks_completed = 0
            self.bids_received.clear()
            self.pending_assignment = None
        elif key == ord('Q'):
            print("Quit requested (keyboard mode)")
            self.hardware.robot.simulationQuit(0)

   
    # AUTOMATED AUCTION SEQUENCE (LOOPED)
   
    def run_auction_auto(self):
        """Run automated auction sequence (looped, 12s per target)."""
        now = self.hardware.get_time()

        # Phase 0: wait for collectors to initialise and report idle
        if self.test_phase == 0:
            if now < 5.0:
                return
            if len(self.spotters_idle) < 1:
                return

            self.print_status()
            print("▶ Starting automated auctions\n")
            self.test_phase = 1
            self.start_time = now
            return

        # Active phases (1..N)
        if 1 <= self.test_phase <= len(self.test_locations):
            phase_start = self.start_time + (self.test_phase - 1) * 12.0

            # Start auction at start of phase
            if phase_start <= now < phase_start + 0.1:
                self.start_auction(self.test_phase - 1)

            # Assign task 2s after auction broadcast
            elif phase_start + 2.0 <= now < phase_start + 2.1:
                if self.pending_assignment:
                    task_id, _ = self.pending_assignment
                    self.assign_task(task_id)

            # After 12s, move to next phase; when finished, loop back
            elif now >= phase_start + 12.0:
                self.test_phase += 1
                if self.test_phase > len(self.test_locations):
                    self.test_phase = 1
                    self.start_time = now

   
    # COVERAGE + AUCTION MAIN LOOP
  
    def run(self):
        # Sensor warm-up
        for _ in range(10):
            self.hardware.step()

        while self.hardware.step() != -1:
            # Spotter coverage & vision 
            rx, ry, yaw = self.hardware.get_pose()
            obstacles = self.hardware.get_lidar_points()

            if self.hardware.is_trash_visible():
                self.hardware.report_trash(self.comm, self.reported_trash)

            if self.waypoints_idx > len(self.mission_waypoints):
                self.hardware.set_velocities(0, 0)
            else:
                goal = self.mission_waypoints[self.waypoints_idx]

                # Waypoint reached?
                if math.hypot(goal[0] - rx, goal[1] - ry) < self.config.WAYPOINT_TOLERANCE:
                    print(f"[SPOTTER] Visited waypoint #{self.waypoints_idx}: {goal}")
                    self.waypoints_idx += 1
                    self.active_path = []
                else:
                    # Plan A* path if needed
                    if not self.active_path:
                        start = self.grid.world_to_grid(rx, ry)
                        end = self.grid.world_to_grid(*goal)
                        if self.grid.is_free(*start):
                            path = self.planner.plan(self.grid, start, end)
                            if path:
                                path = self.planner.smooth_path(self.grid, path)
                                self.active_path = [self.grid.grid_to_world(r, c) for r, c in path]
                            else:
                                print("[SPOTTER] Path not found, skipping waypoint.")
                                self.waypoints_idx += 1
                        else:
                            print("[SPOTTER] Robot off-grid, stopping coverage.")
                            self.hardware.set_velocities(0, 0)

                    # Follow path towards target
                    target = goal
                    if self.active_path:
                        target = self.active_path[0]
                        if math.hypot(target[0] - rx, target[1] - ry) < self.config.PATH_NODE_TOLERANCE:
                            self.active_path.pop(0)
                            if self.active_path:
                                target = self.active_path[0]

                    gx, gy = self.world_to_robot(rx, ry, yaw, *target)
                    dist = math.hypot(gx, gy)
                    ang = _wrap(math.atan2(gy, gx))

                    v_dwa = max(-0.35, min(0.35, 0.5 * dist))
                    w_dwa = max(-2.0, min(2.0, -1.5 * ang))

                    v = self.config.SMOOTHING_FACTOR * self.prev_cmd[0] + \
                        (1.0 - self.config.SMOOTHING_FACTOR) * v_dwa
                    w = self.config.SMOOTHING_FACTOR * self.prev_cmd[1] + \
                        (1.0 - self.config.SMOOTHING_FACTOR) * w_dwa

                    self.prev_cmd = (v, w)
                    self.hardware.set_velocities(v, w)

            #  Auctioneer behaviour 
            for msg in self.receive_messages():
                event = msg.get("event")
                if event == "idle":
                    self.process_idle(msg)
                elif event == "bid":
                    self.process_bid(msg)
                elif event == "collected":
                    self.process_complete(msg)

            if self.manual:
                self.handle_keyboard()
            else:
                self.run_auction_auto()


if __name__ == "__main__":
    SpotterController().run()
