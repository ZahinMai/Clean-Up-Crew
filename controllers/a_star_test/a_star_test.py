# Complete navigation controller collectors
# Uses A* global planner and DWA local planner
# Author: Zahin Maisa

from controller import Robot
import math, os, sys
from collections import deque

THIS_DIR = os.path.dirname(__file__)
CTRL_DIR = os.path.dirname(THIS_DIR)
if CTRL_DIR not in sys.path:
    sys.path.append(CTRL_DIR)

from lib_shared.local_planner import DWA
from lib_shared.global_planner import AStarPlanner, OccupancyGrid
from lib_shared.map import get_map
from lib_shared.map_visualiser import visualise_robot_on_map, print_coordinate_info, check_map_alignment

LEFT_MOTOR = 'left wheel motor'
RIGHT_MOTOR = 'right wheel motor'
LIDAR_NAME = 'LDS-01'
IMU_NAME = 'inertial unit'
COMPASS_NAME = 'compass'
GPS_NAME = 'gps'

WHEEL_RADIUS = 0.033
AXLE_LENGTH = 0.160

WAYPOINT_TOLERANCE = 0.18
REPLAN_INTERVAL = 5.0
REPLAN_DISTANCE = 0.8
LOOKAHEAD = 2

def get_device(robot, name):
    try:
        return robot.getDevice(name)
    except:
        return None

def get_yaw(imu, compass):
    if imu:
        r, p, y = imu.getRollPitchYaw()
        return y
    if compass:
        n = compass.getValues()
        return math.atan2(n[0], n[2])
    return 0.0

def world_to_robot(dx, dz, yaw):
    gx = dx * math.cos(-yaw) - dz * math.sin(-yaw)
    gy = dx * math.sin(-yaw) + dz * math.cos(-yaw)
    return gx, gy

class SimpleNavigator:
    def __init__(self, occupancy_grid):
        self.grid = occupancy_grid
        self.inflated_grid = self.grid.inflate(radius=1)
        self.astar = AStarPlanner(allow_diagonal=True)
        self.global_path = []
        self.global_path_world = []
        self.current_wp_idx = 0
        self.last_replan_time = -100.0

    def plan_path(self, start_x, start_z, goal_x, goal_z):
        grid_to_plan = self.inflated_grid
        start_grid = grid_to_plan.world_to_grid(start_x, start_z)
        goal_grid = grid_to_plan.world_to_grid(goal_x, goal_z)

        if not grid_to_plan.is_free(*start_grid):
            start_grid = self._find_free(start_grid, grid=grid_to_plan)
            if not start_grid:
                return False

        if not grid_to_plan.is_free(*goal_grid):
            goal_grid = self._find_free(goal_grid, grid=grid_to_plan)
            if not goal_grid:
                return False

        path = self.astar.plan(grid_to_plan, start_grid, goal_grid)
        if not path:
            return False

        self.global_path = path
        self.global_path_world = [self.grid.grid_to_world(r, c) for r, c in path]
        self.current_wp_idx = 0
        return True

    def _find_free(self, pos, radius=5, grid=None):
        if grid is None:
            grid = self.grid
        r, c = pos
        for d in range(1, radius + 1):
            for dr in range(-d, d + 1):
                for dc in range(-d, d + 1):
                    if abs(dr) + abs(dc) == 0: continue
                    nr, nc = r + dr, c + dc
                    if grid.is_free(nr, nc):
                        return (nr, nc)
        return None

    def should_replan(self, robot_x, robot_z, time):
        if not self.global_path_world:
            return False
        if time - self.last_replan_time < REPLAN_INTERVAL:
            return False
        min_dist = min(math.hypot(wx - robot_x, wz - robot_z) for wx, wz in self.global_path_world[self.current_wp_idx:])
        return min_dist > REPLAN_DISTANCE

    def get_next_waypoint(self, robot_x, robot_z):
        if not self.global_path_world:
            return None
        while self.current_wp_idx < len(self.global_path_world):
            wx, wz = self.global_path_world[self.current_wp_idx]
            dist = math.hypot(wx - robot_x, wz - robot_z)
            if dist < (WAYPOINT_TOLERANCE * 1.5):
                self.current_wp_idx += 1
            else:
                break
        if self.current_wp_idx >= len(self.global_path_world):
            return None
        lookahead_idx = min(self.current_wp_idx + LOOKAHEAD, len(self.global_path_world) - 1)
        return self.global_path_world[lookahead_idx]

    def get_progress(self):
        if not self.global_path:
            return 0.0
        return 100.0 * self.current_wp_idx / len(self.global_path)

def main():
    robot = Robot()
    ts = int(robot.getBasicTimeStep())

    lm = get_device(robot, LEFT_MOTOR)
    rm = get_device(robot, RIGHT_MOTOR)
    lm.setPosition(float('inf'))
    rm.setPosition(float('inf'))
    lm.setVelocity(0.0)
    rm.setVelocity(0.0)

    imu = get_device(robot, IMU_NAME)
    if imu: imu.enable(ts)
    compass = get_device(robot, COMPASS_NAME)
    if compass: compass.enable(ts)
    gps = get_device(robot, GPS_NAME)
    gps.enable(ts)
    lidar = get_device(robot, LIDAR_NAME)
    lidar.enable(ts)
    lidar.enablePointCloud()

    grid = get_map()
    check_map_alignment(grid)
    navigator = SimpleNavigator(grid)

    dwa = DWA({
        "V_MAX": 0.22,
        "W_MAX": 2.5,
        "RADIUS": 0.18,
        "SAFE": 0.14,
        "DT_CTRL": ts/1000.0,
        "ALPHA": 0.30,
        "BETA": 0.45,
        "GAMMA": 0.10,
    })
    prev_cmd = (0.0, 0.0)

    MISSION = [
        (5.0, 5.0, "Goal 1"),
        (5.0, -5.0, "Goal 2"),
        (-5.0, -5.0, "Goal 3"),
        (-5.0, 5.0, "Goal 4"),
        (0.0, 0.0, "Center"),
    ]
    mission_idx = 0
    path_planned = False

    while robot.step(ts) != -1:
        now = robot.getTime()
        x, _, z = gps.getValues()
        yaw = get_yaw(imu, compass)

        if not path_planned:
            if mission_idx >= len(MISSION):
                lm.setVelocity(0.0)
                rm.setVelocity(0.0)
                break
            goal_x, goal_z, _ = MISSION[mission_idx]
            print_coordinate_info(grid, x, z)
            if navigator.plan_path(x, z, goal_x, goal_z):
                path_planned = True
                navigator.last_replan_time = now
                visualise_robot_on_map(grid, x, z, yaw, goal_x, goal_z, navigator.global_path)
            else:
                mission_idx += 1
                continue

        if navigator.should_replan(x, z, now):
            goal_x, goal_z, _ = MISSION[mission_idx]
            if navigator.plan_path(x, z, goal_x, goal_z):
                navigator.last_replan_time = now

        waypoint = navigator.get_next_waypoint(x, z)
        if waypoint is None:
            mission_idx += 1
            path_planned = False
            prev_cmd = (0.0, 0.0)
            lm.setVelocity(0.0)
            rm.setVelocity(0.0)
            continue

        ranges = lidar.getRangeImage()
        hfov = lidar.getFov()
        nscan = lidar.getHorizontalResolution()
        obs = []
        for i, r in enumerate(ranges[::5]):
            if r != float('inf') and r < 2.0:
                idx_original = i * 5
                alpha = -hfov/2 + hfov * (idx_original / (nscan - 1))
                ox = r * math.cos(alpha)
                oy = r * math.sin(alpha)
                obs.append((ox, oy))

        dx = waypoint[0] - x
        dz = waypoint[1] - z
        gx, gy = world_to_robot(dx, dz, yaw)

        v, w = dwa.get_safe_velocities(obs, (gx, gy), prev_cmd=prev_cmd)
        prev_cmd = (v, w)

        wl = (v - 0.5*w*AXLE_LENGTH) / WHEEL_RADIUS
        wr = (v + 0.5*w*AXLE_LENGTH) / WHEEL_RADIUS
        max_w = min(lm.getMaxVelocity(), rm.getMaxVelocity())
        wl = max(-max_w, min(max_w, wl))
        wr = max(-max_w, min(max_w, wr))

        lm.setVelocity(wl)
        rm.setVelocity(wr)

        if int(now) % 2 == 0 and int(now * 10) % 10 == 0:
            visualise_robot_on_map(grid, x, z, yaw, MISSION[mission_idx][0], MISSION[mission_idx][1], navigator.global_path)

if __name__ == "__main__":
    main()