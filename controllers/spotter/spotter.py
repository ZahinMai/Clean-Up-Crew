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


# Coverage Planner
class CoveragePlanner:
    def __init__(self, grid_obj, step: int):
        self.grid = grid_obj
        self.step = step
        self.working_grid = [row[:] for row in self.grid.grid] # Copy of the grid (0=Free, 1=Obstacle, 2=Visited)
        self.zones = []
        self.corners = { # Config to help with joining zones
            'TL': {'is_top': True,  'is_right': False},
            'BL': {'is_top': False, 'is_right': False},
            'TR': {'is_top': True,  'is_right': True},
            'BR': {'is_top': False, 'is_right': True}
        }

    def generate_lawnmower_path(self) -> List[Tuple[float, float]]:
        print("Generating Coverage Path.")
        
        self._decompose_grid()
        
        if not self.zones:
            return []

        ordered_zones = self._order_zones_by_adjacency()
        
        # Final path generation
        final_waypoints = []
        
        for i, zone in enumerate(ordered_zones):
            if i == 0:
                # First corner reflects the Spotter bot's starting position
                chosen_corner = 'BL'
            else:
                # For following zones, find the closest corner to previous end point
                previous_point = final_waypoints[-1]
                chosen_corner = self._get_closest_corner(zone, previous_point)
            
            zone_path = self._generate_zone_zigzag(zone, chosen_corner)
            final_waypoints.extend(zone_path)

        print(f"Mission: {len(final_waypoints)} points.")
        return final_waypoints
    
    def _decompose_grid(self):
        """Iteratively finds the largest valid rectangle of free space starting from top-left."""
        rows = self.grid.height
        cols = self.grid.width
        
        while True:
            # Find the first '0' (free space)
            found = False
            r_start, c_start = -1, -1
            
            for r in range(rows):
                for c in range(cols):
                    if self.working_grid[r][c] == 0:
                        r_start, c_start = r, c
                        found = True
                        break
                if found: break
            
            if not found:
                break # No more free space
            
            # Expand Width (Right)
            c_end = c_start
            while (c_end + 1 < cols) and (self.working_grid[r_start][c_end + 1] == 0):
                c_end += 1
            
            # Expand Height (Down)
            r_end = r_start
            while r_end + 1 < rows:
                next_r = r_end + 1
                is_row_free = True
                
                # For every new row we add, the entire width must be free (prevents L-shaped 'rectangles').
                for k in range(c_start, c_end + 1):
                    if self.working_grid[next_r][k] != 0:
                        is_row_free = False
                        break
                
                if is_row_free:
                    r_end += 1
                else:
                    break # Stop expanding down if we hit an obstacle
            
            # Save the valid rectangle
            zone = {
                'r_min': r_start, 'r_max': r_end,
                'c_min': c_start, 'c_max': c_end,
                'id': len(self.zones)
            }
            self.zones.append(zone)
            
            # Mark area as Visited (2)
            for r in range(r_start, r_end + 1):
                for c in range(c_start, c_end + 1):
                    self.working_grid[r][c] = 2

    def _are_zones_touching(self, z1, z2):
        """Returns True if zones share a border (corners are neighbours)."""
        # Vertical neighbours
        is_vert_touching = (z1['r_max'] + 1 == z2['r_min']) or (z2['r_max'] + 1 == z1['r_min'])
        is_horz_overlap = not (z1['c_max'] < z2['c_min'] or z2['c_max'] < z1['c_min'])
        
        if is_vert_touching and is_horz_overlap: return True

        # Horizontal neighbours
        is_horz_touching = (z1['c_max'] + 1 == z2['c_min']) or (z2['c_max'] + 1 == z1['c_min'])
        is_vert_overlap = not (z1['r_max'] < z2['r_min'] or z2['r_max'] < z1['r_min'])

        if is_horz_touching and is_vert_overlap: return True
        
        return False # Not touching

    def _order_zones_by_adjacency(self):
        """Greedy chain to pick next touching zone."""
        ordered = []
        visited_ids = set()
        
        current_idx = 0
        
        while current_idx is not None:
            ordered.append(self.zones[current_idx])
            visited_ids.add(current_idx)
            
            next_idx = None
            for i, zone in enumerate(self.zones):
                if i in visited_ids: continue
                
                # Prevents joining zones that are separated by obstacles
                if self._are_zones_touching(self.zones[current_idx], zone):
                    next_idx = i
                    break
            
            current_idx = next_idx 
            
        return ordered

    def _get_closest_corner(self, zone, current_bot_pos):
        """Finds the corner of the zone closest to the current bot's position."""
        zone_corners = {
            'TL': (zone['r_min'], zone['c_min']),
            'BL': (zone['r_max'], zone['c_min']),
            'TR': (zone['r_min'], zone['c_max']),
            'BR': (zone['r_max'], zone['c_max'])
        }
        
        best_key = 'TL'
        min_distance = float('inf')
        
        for key, (r, c) in zone_corners.items():
            wx, wy = self.grid.grid_to_world(r, c)

            distance = (wx - current_bot_pos[0])**2 + (wy - current_bot_pos[1])**2
            
            if distance < min_distance:
                min_distance = distance
                best_key = key
                
        return best_key

    def _generate_zone_zigzag(self, zone, start_key):
        """Generates waypoints for a given zone starting from a specified corner."""
        waypoints = []
        r_min, r_max = zone['r_min'], zone['r_max']
        c_min, c_max = zone['c_min'], zone['c_max']
        step = self.step
        corner = self.corners[start_key]

        # Order columns and rows based on starting corner
        cols = range(c_min, c_max + 1, step)
        if corner['is_right']:
            cols = range(c_max, c_min - 1, -step)

        for i, c in enumerate(cols):
            is_going_down = (i % 2 == 0) if corner['is_top'] else (i % 2 != 0)
            if is_going_down:
                rows = range(r_min, r_max + 1, step)
            else:
                rows = range(r_max, r_min - 1, -step)
                
            for r in rows:
                waypoints.append(self.grid.grid_to_world(r, c))
                
        return waypoints


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