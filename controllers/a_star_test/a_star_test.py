from controller import Robot
import math, os, sys

# ----- allow importing ../lib_shared -----------------------------------------
THIS_DIR = os.path.dirname(__file__)
CTRL_DIR = os.path.dirname(THIS_DIR)
if CTRL_DIR not in sys.path:
    sys.path.append(CTRL_DIR)

from lib_shared.global_planner import AStarPlanner, OccupancyGrid
from lib_shared.map_module import visualise_robot_on_map, print_coordinate_info, check_map_alignment, get_map

LEFT_MOTOR = 'left wheel motor'
RIGHT_MOTOR = 'right wheel motor'
LIDAR_NAME = 'LDS-01'
IMU_NAME = 'inertial unit'
COMPASS_NAME = 'compass'
GPS_NAME = 'gps'

WHEEL_RADIUS = 0.033
AXLE_LENGTH = 0.16

WAYPOINT_TOLERANCE = 0.1
REPLAN_INTERVAL = 0.1 # Replan often to correct drift
REPLAN_DISTANCE = 0.1
LOOKAHEAD = 1

def get_yaw(imu, compass):
    if imu:
        r, p, y = imu.getRollPitchYaw()
        return y
    if compass:
        n = compass.getValues()
        return math.atan2(n[0], n[2])
    return 0.0

class SimpleNavigator:
    def __init__(self, occupancy_grid):
        self.grid = occupancy_grid
        self.inflated_grid = self.grid.inflate(radius=0) # Inflate by 1 cell (0.25m) for safety
        self.astar = AStarPlanner()
        self.global_path = [] # path in grid coordinates
        self.global_path_world = [] # path in world coordinates
        self.current_wp_idx = 0 # index of current waypoint
        self.last_replan_time = -100.0 # this ensures immediate first plan

    def plan_path(self, start_x, start_z, goal_x, goal_z):
        grid_to_plan = self.inflated_grid
        start_grid = grid_to_plan.world_to_grid(start_x, start_z)
        goal_grid = grid_to_plan.world_to_grid(goal_x, goal_z)

        if not grid_to_plan.is_free(*start_grid):
            print(f"[Nav] Start blocked. Finding free cell...")
            start_grid = self._find_free(start_grid, grid=grid_to_plan)
            if not start_grid: return False

        if not grid_to_plan.is_free(*goal_grid):
            print(f"[Nav] Goal blocked. Finding free cell...")
            goal_grid = self._find_free(goal_grid, grid=grid_to_plan)
            if not goal_grid: return False

        path = self.astar.plan(grid_to_plan, start_grid, goal_grid)
        if not path:
            print("[Nav] No path found!")
            return False

        self.global_path = path
        self.global_path_world = [self.grid.grid_to_world(r, c) for r, c in path]
        self.current_wp_idx = 0
        return True

    def _find_free(self, pos, radius=8, grid=None):
        if grid is None: grid = self.grid
        r, c = pos
        for d in range(1, radius + 1):
            for dr in range(-d, d + 1):
                for dc in range(-d, d + 1):
                    if abs(dr) != d and abs(dc) != d: continue 
                    nr, nc = r + dr, c + dc
                    if grid.is_free(nr, nc): return (nr, nc)
        return None

    def should_replan(self, robot_x, robot_z, time):
        if not self.global_path_world: return False
        if time - self.last_replan_time < REPLAN_INTERVAL: return False
        # Simple distance check to current waypoint
        if self.current_wp_idx < len(self.global_path_world):
             wx, wz = self.global_path_world[self.current_wp_idx]
             dist = math.hypot(wx - robot_x, wz - robot_z)
             if dist > REPLAN_DISTANCE: return True
        return False

    def get_next_waypoint(self, robot_x, robot_z):
        if not self.global_path_world: return None
        while self.current_wp_idx < len(self.global_path_world):
            wx, wz = self.global_path_world[self.current_wp_idx]
            dist = math.hypot(wx - robot_x, wz - robot_z)
            if dist < WAYPOINT_TOLERANCE:
                self.current_wp_idx += 1
            else:
                break
        if self.current_wp_idx >= len(self.global_path_world): return None
        lookahead_idx = min(self.current_wp_idx + LOOKAHEAD, len(self.global_path_world) - 1)
        return self.global_path_world[lookahead_idx]

    def get_progress(self):
        if not self.global_path: return 0.0
        return 100.0 * self.current_wp_idx / len(self.global_path)

def main():
    robot = Robot()
    ts = int(robot.getBasicTimeStep())

    # get & setup motors
    lm = robot.getDevice(LEFT_MOTOR)
    rm = robot.getDevice(RIGHT_MOTOR)
    lm.setPosition(float('inf'))
    rm.setPosition(float('inf'))
    lm.setVelocity(0.0)
    rm.setVelocity(0.0)

    # get & enable sensors
    imu = robot.getDevice(IMU_NAME)
    if imu: imu.enable(ts) # not all robots have IMU
    compass = robot.getDevice(COMPASS_NAME)
    if compass: compass.enable(ts) # not all robots have compass
    gps = robot.getDevice(GPS_NAME)
    gps.enable(ts)
    
    grid = get_map()
    check_map_alignment(grid)
    navigator = SimpleNavigator(grid)

    V_MAX = 0.3 # forward speed 
    W_MAX = 2.5 # sharpness of turns           
    KP_TURN = 2.0 # turning gain
    
    MISSION = [(3.5, 3.0, "Goal 1")]
    mission_idx = 0
    path_planned = False

    # ------ Main control loop ------
    while robot.step(ts) != -1:
        now = robot.getTime()
        x, _, z = gps.getValues()
        yaw = get_yaw(imu, compass)

        if not path_planned:
            if mission_idx >= len(MISSION):
                lm.setVelocity(0.0)
                rm.setVelocity(0.0)
                print("Mission Complete")
                break
            goal_x, goal_z, _ = MISSION[mission_idx]
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

        navigator.get_progress()
        waypoint = navigator.get_next_waypoint(x, z)
        if waypoint is None:
            mission_idx += 1
            path_planned = False
            lm.setVelocity(0.0)
            rm.setVelocity(0.0)
            continue

        # Steering by adjusting wheel speed ratio
        dx = waypoint[0] - x
        dz = waypoint[1] - z
        
        # Calculate target angle in standard math frame
        target_angle = math.atan2(-dz, dx) # North is -Z.
        
        # Calculate error & wrap to [-pi, pi]
        error = target_angle - yaw
        while error > math.pi: error -= 2 * math.pi
        while error < -math.pi: error += 2 * math.pi

        # adjust v,w based on angle error, normalise to max speeds
        w = KP_TURN * error
        v = V_MAX * max(0.0, 1.0 - abs(error) / 0.5) # Slow down if error > 0.5 rad

        v = max(0.0, min(V_MAX, v))
        w = max(-W_MAX, min(W_MAX, w))

        wl = (v - 0.5 * w * AXLE_LENGTH) / WHEEL_RADIUS
        wr = (v + 0.5 * w * AXLE_LENGTH) / WHEEL_RADIUS
        
        max_wheel_v = min(lm.getMaxVelocity(), rm.getMaxVelocity())
        wl = max(-max_wheel_v, min(max_wheel_v, wl))
        wr = max(-max_wheel_v, min(max_wheel_v, wr))

        lm.setVelocity(wl)
        rm.setVelocity(wr)

        if int(now) % 2 == 0:
            visualise_robot_on_map(grid, x, z, yaw, MISSION[mission_idx][0], MISSION[mission_idx][1], navigator.global_path)

if __name__ == "__main__":
    main()