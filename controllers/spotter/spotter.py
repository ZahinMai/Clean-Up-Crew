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

from lib_shared import vision
from spotter_coverage import CoveragePlanner
from lib_shared.communication import Communication
from lib_shared.global_planner import AStarPlanner
from lib_shared.local_planner import DWA
from lib_shared.map_module import get_map


# Config
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
        if self.lidar:
            self.lidar.enable(self.timestep)
            self.lidar_fov = self.lidar.getFov()

        self.lm = self.robot.getDevice('left wheel motor') or self.robot.getDevice('left wheel') # TODO: Confirm names
        self.rm = self.robot.getDevice('right wheel motor') or self.robot.getDevice('right wheel')
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
    
    def report_trash(self, comm, reported_trash):
        """Calculates trash's coordinates in relation to bot and reports if unreported."""

        # Calculate Trash Position (30cm in front of robot)
        rx, ry, yaw = self.get_pose()
        tx = round((rx + 0.3 * math.cos(yaw)), 2)
        ty = round((ry + 0.3 * math.sin(yaw)), 2)

        # Ignore if we have already reported trash within 0.5m of this spot
        for (ex, ey) in reported_trash:
            if math.hypot(tx - ex, ty - ey) < 0.5:
                return

        # 4. Report and Save
        print(f"NEW TRASH FOUND at ({tx}, {ty})")
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


# Main Controller
class SpotterController:
    def __init__(self):
        print("Initializing Spotter Controller.")
        self.config = RobotConfig()
        self.hardware = HardwareInterface(Robot(), self.config)
        
        self.grid = get_map()
        self.planner = AStarPlanner()
        self.dwa = DWA({
            "RADIUS": 0.035, "SAFE": 0.015, "V_MAX": 0.12, 
            "ALPHA": 0.8, "BETA": 0.4, "GAMMA": 0.3, "EPS": 0.1
        })
        self.comm = Communication(self.hardware.robot)

        coverage_planner = CoveragePlanner(self.grid, step=self.config.LAWNMOWER_STEP)
        self.mission_waypoints = coverage_planner.generate_lawnmower_path()
        self.waypoints_idx = 0
        self.active_path = []
        self.prev_cmd = (0.0, 0.0)
        
        self.reported_trash = set()

    def world_to_robot(self, rx, ry, yaw, tx, ty):
        """Transforms world target (tx, ty) into the robot's local frame."""
        dx, dy = tx - rx, ty - ry
        return (dx * math.cos(-yaw) - dy * math.sin(-yaw),
                dx * math.sin(-yaw) + dy * math.cos(-yaw))

    def run(self):
        # Sensor stabilisation
        for _ in range(10): self.hardware.step()
        
        while self.hardware.step() != -1:
            rx, ry, yaw = self.hardware.get_pose()
            t = self.hardware.get_time()
            obstacles = self.hardware.get_lidar_points()
            
            # Vision Logic
            if self.hardware.is_trash_visible():
                self.hardware.report_trash(self.comm, self.reported_trash)

            # Mission Status Check
            if self.waypoints_idx > len(self.mission_waypoints):
                self.hardware.set_velocities(0, 0)
                print("Spotter Mission Complete.")
                break

            goal = self.mission_waypoints[self.waypoints_idx]
            
            # Check if waypoint reached
            if math.hypot(goal[0] - rx, goal[1] - ry) < self.config.WAYPOINT_TOLERANCE:
                print(f"Visited Waypoint #{self.waypoints_idx}: {goal}")
                self.waypoints_idx += 1
                self.active_path = [] # Clear path to force planning for next point
                continue

            # Path Planning (A*)
            if not self.active_path:
                start = self.grid.world_to_grid(rx, ry)
                end = self.grid.world_to_grid(*goal)
                if self.grid.is_free(*start):
                    path = self.planner.plan(self.grid, start, end)
                    if path:
                        path = self.planner.smooth_path(self.grid, path)
                        self.active_path = [self.grid.grid_to_world(r, c) for r, c in path]
                    else:
                        print("Path not found, skipping waypoint.")
                        self.waypoints_idx += 1 # Skip unreachable point
                        continue
                else:
                    print("Robot off-grid.")
                    break

            # Aim for the first point in path, pop if close enough
            target = goal
            if self.active_path: 
                target = self.active_path[0]
                if math.hypot(target[0]-rx, target[1]-ry) < self.config.PATH_NODE_TOLERANCE:
                    self.active_path.pop(0)
                    if self.active_path: target = self.active_path[0]

            # Control and Navigation
            gx, gy = self.world_to_robot(rx, ry, yaw, *target)
            v_dwa, w_dwa = self.dwa.get_safe_velocities(obstacles, (gx, gy), prev_cmd=self.prev_cmd)
            
            v = self.config.SMOOTHING_FACTOR * self.prev_cmd[0] + (1.0 - self.config.SMOOTHING_FACTOR) * v_dwa
            w = self.config.SMOOTHING_FACTOR * self.prev_cmd[1] + (1.0 - self.config.SMOOTHING_FACTOR) * w_dwa

            self.hardware.set_velocities(v, w)


if __name__ == "__main__":
    SpotterController().run()