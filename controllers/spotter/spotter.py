# Spotter controller (Setup 3)
# Responsible for finding trash using an efficient search pattern.
# Uses: navigation, obstacle_avoidance, vision, communication
# Implemented by: Abdullateef Vahora

import sys
import os
import math
from collections import deque
from dataclasses import dataclass
from typing import List, Tuple

from controller import Robot

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from lib_shared.global_planner import AStarPlanner
from lib_shared.map_module import get_map
from lib_shared.local_planner import DWA
from lib_shared import vision


# Config
@dataclass(frozen=True)
class RobotConfig:
    WHEEL_RADIUS: float = 0.0205
    AXLE_LENGTH: float = 0.052
    MAX_WHEEL_OMEGA: float = 6.28
    
    LIDAR_FOV: float = 4.18
    LIDAR_MAX_RANGE: float = 2.0
    LIDAR_SELF_COLLISION: float = 0.05
    
    WAYPOINT_TOLERANCE: float = 0.20
    PATH_NODE_TOLERANCE: float = 0.25
    SMOOTHING_FACTOR: float = 0.6
    
    STUCK_AVG_THRESHOLD: float = 0.01
    STUCK_WINDOW: int = 20
    RECOVERY_DURATION: float = 0.4
    RECOVERY_COOLDOWN: float = 2.0
    
    MAP_ORIGIN: Tuple[float, float] = (-6.1, -2.8)
    LAWNMOWER_STEP: int = 4


# Hardware
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
        self.lidar_fov = config.LIDAR_FOV
        if self.lidar:
            self.lidar.enable(self.timestep)
            self.lidar.enablePointCloud()
            try:
                self.lidar_fov = self.lidar.getFov()
            except Exception:
                pass 

        self.lm = self.robot.getDevice('left wheel motor') or self.robot.getDevice('left wheel')
        self.rm = self.robot.getDevice('right wheel motor') or self.robot.getDevice('right wheel')
        self.lm.setPosition(float('inf'))
        self.rm.setPosition(float('inf'))
        self.set_velocities(0, 0)

    def step(self):
        """Advances the simulation by one timestep."""
        return self.robot.step(self.timestep)

    def get_time(self):
        return self.robot.getTime()

    def get_pose(self) -> Tuple[float, float, float]:
        """Returns current robot state (x, y, yaw)."""
        vals = self.gps.getValues()
        c_vals = self.compass.getValues()
        yaw = math.atan2(c_vals[0], c_vals[1]) if c_vals else 0.0
        return vals[0], vals[1], yaw

    def get_lidar_points(self) -> List[Tuple[float, float]]:
        """Converts valid lidar ranges to cartesian coordinates (robot frame)."""
        if not self.lidar: 
            return []
            
        ranges = self.lidar.getRangeImage()
        if not ranges or len(ranges) < 2: 
            return []

        points = []
        angle_step = self.lidar_fov / (len(ranges) - 1)
        current_angle = -self.lidar_fov / 2.0

        for r in ranges:
            # Filter out noise and self-collision points
            if self.cfg.LIDAR_SELF_COLLISION < r < self.cfg.LIDAR_MAX_RANGE:
                lx = r * math.cos(current_angle)
                ly = r * math.sin(current_angle)
                points.append((lx, ly))
            current_angle += angle_step
            
        return points

    def is_trash_visible(self):
        return vision.is_trash_visible(self.camera)

    def set_velocities(self, v: float, w: float):
        """Calculates and sets wheel speeds based on unicycle model (v, w)."""
        wl = (v - w * self.cfg.AXLE_LENGTH / 2) / self.cfg.WHEEL_RADIUS
        wr = (v + w * self.cfg.AXLE_LENGTH / 2) / self.cfg.WHEEL_RADIUS
        
        wl = max(-self.cfg.MAX_WHEEL_OMEGA, min(self.cfg.MAX_WHEEL_OMEGA, wl))
        wr = max(-self.cfg.MAX_WHEEL_OMEGA, min(self.cfg.MAX_WHEEL_OMEGA, wr))
        
        self.lm.setVelocity(wl)
        self.rm.setVelocity(wr)


# Main Controller
class SpotterController:
    def __init__(self):
        print("Initializing Spotter Controller...")
        self.config = RobotConfig()
        self.hw = HardwareInterface(Robot(), self.config)
        
        self.grid = get_map(origin=self.config.MAP_ORIGIN)
        self.planner = AStarPlanner()
        
        self.dwa = DWA({
            "RADIUS": 0.035, "SAFE": 0.015, "V_MAX": 0.12, 
            "ALPHA": 0.8, "BETA": 0.4, "GAMMA": 0.3, "EPS": 0.1
        })

        self.mission_waypoints = self._generate_lawnmower_path()
        self.wp_index = 0
        self.active_path = []
        self.prev_cmd = (0.0, 0.0)
        
        self.speed_history = deque(maxlen=self.config.STUCK_WINDOW)
        self.last_unstuck_time = -10.0
        self.recovery_end_time = 0.0
        self.last_trash_report = 0

    def _generate_lawnmower_path(self) -> List[Tuple[float, float]]:
        """Generates a zigzag coverage path across the map grid."""
        print("Generating coverage path.")
        waypoints = []
        step = self.config.LAWNMOWER_STEP
        
        for c in range(1, self.grid.width - 1, step):
            row_indices = range(1, self.grid.height - 1, step)
            # Reverse every odd column to create zigzag pattern
            if (c // step) % 2 == 1:
                row_indices = reversed(row_indices)
            
            for r in row_indices:
                if self.grid.is_free(r, c):
                    waypoints.append(self.grid.grid_to_world(r, c))
                    
        print(f"Mission: {len(waypoints)} points.")
        return waypoints

    def _check_stuck(self, v: float, t: float) -> bool:
        """Heuristic to detect if robot is stuck based on low avg velocity."""
        self.speed_history.append(abs(v))
        
        # If currently recovering, keep spinning
        if t < self.recovery_end_time:
            return True
            
        avg_speed = sum(self.speed_history) / len(self.speed_history)
        is_stuck = (len(self.speed_history) == self.speed_history.maxlen and 
                    avg_speed < self.config.STUCK_AVG_THRESHOLD)
        
        cooled_down = (t - self.last_unstuck_time) > self.config.RECOVERY_COOLDOWN
        
        if is_stuck and cooled_down:
            print("Robot is Stuck. Spinning...")
            self.recovery_end_time = t + self.config.RECOVERY_DURATION
            self.last_unstuck_time = t
            sys.exit(0)
            return True
            
        return False

    def _world_to_robot(self, rx, ry, yaw, tx, ty):
        """Transforms world target (tx, ty) into the robot's local frame."""
        dx, dy = tx - rx, ty - ry
        return (dx * math.cos(-yaw) - dy * math.sin(-yaw),
                dx * math.sin(-yaw) + dy * math.cos(-yaw))

    def run(self):
        # Sensor stabilization
        for _ in range(10): self.hw.step()
        
        while self.hw.step() != -1:
            rx, ry, yaw = self.hw.get_pose()
            t = self.hw.get_time()
            obstacles = self.hw.get_lidar_points()
            
            # Vision Logic
            if self.hw.is_trash_visible() and (t - self.last_trash_report > 5.0):
                print(f"TRASH FOUND at ({rx:.2f}, {ry:.2f})")
                self.last_trash_report = t

            # Mission Status Check
            if self.wp_index >= len(self.mission_waypoints):
                self.hw.set_velocities(0, 0)
                print("Mission Complete.")
                break

            goal = self.mission_waypoints[self.wp_index]
            
            # Check if waypoint reached
            if math.hypot(goal[0] - rx, goal[1] - ry) < self.config.WAYPOINT_TOLERANCE:
                print(f"Visited Waypoint #{self.wp_index}")
                self.wp_index += 1
                self.active_path = [] # Clear path to force re-plan for next point
                continue

            # Path Planning (A*)
            if not self.active_path:
                start = self.grid.world_to_grid(rx, ry)
                end = self.grid.world_to_grid(*goal)
                
                if self.grid.is_valid(*start):
                    path = self.planner.plan(self.grid, start, end)
                    if path:
                        path = self.planner.smooth_path(self.grid, path)
                        self.active_path = [self.grid.grid_to_world(r, c) for r, c in path]
                    else:
                        self.wp_index += 1 # Skip unreachable point
                        continue
                else:
                    print("Critical: Robot off-grid.")
                    break

            # Local Targeting. Aim for the first point in path, pop if close enough
            target = goal
            if self.active_path:
                target = self.active_path[0]
                if math.hypot(target[0]-rx, target[1]-ry) < self.config.PATH_NODE_TOLERANCE:
                    self.active_path.pop(0)
                    if self.active_path: target = self.active_path[0]

            # Control & Navigation
            gx, gy = self._world_to_robot(rx, ry, yaw, *target)
            v_dwa, w_dwa = self.dwa.get_safe_velocities(obstacles, (gx, gy), prev_cmd=self.prev_cmd)
            
            v = self.config.SMOOTHING_FACTOR * self.prev_cmd[0] + (1.0 - self.config.SMOOTHING_FACTOR) * v_dwa
            w = self.config.SMOOTHING_FACTOR * self.prev_cmd[1] + (1.0 - self.config.SMOOTHING_FACTOR) * w_dwa

            # Recovery
            if self._check_stuck(v, t):
                v, w = 0.0, self.config.MAX_WHEEL_OMEGA
                self.prev_cmd = (0, 0)
            else:
                self.prev_cmd = (v, w)

            self.hw.set_velocities(v, w)

            # Debug logging
            # if t % 2.0 < 0.05:
            #    print(f"Target: ({target[0]:.1f}, {target[1]:.1f}) | WP: {self.wp_index}")

if __name__ == "__main__":
    SpotterController().run()