from controller import Robot
import math, os, sys
from collections import deque
from typing import Optional, Tuple, List

# ---- So I can import lib_shared ---- #
THIS_DIR = os.path.dirname(__file__)
PARENT_DIR = os.path.dirname(THIS_DIR)
if PARENT_DIR not in sys.path:
    sys.path.insert(0, PARENT_DIR)
# ----------------------------------- #

from lib_shared.global_planner import AStarPlanner
from lib_shared.map_module import visualise_robot_on_map, get_map
from lib_shared.local_planner import DWA, _ang

# Devices & physical constants
LEFT_MOTOR, RIGHT_MOTOR = 'left wheel motor', 'right wheel motor'
LIDAR_NAME, IMU_NAME, COMPASS_NAME, GPS_NAME = 'LDS-01', 'inertial unit', 'compass', 'gps'
WHEEL_RADIUS, AXLE_LENGTH = 0.033, 0.16

# Behavioral constants
WAYPOINT_TOLERANCE = 0.2
REPLAN_INTERVAL = 1.0
LOOKAHEAD = 1
VIS_INTERVAL = 0.5
DWA_ENABLED = True # TURNED OFF BECAUSE IT DOESN"T WORK WELL RN

# DWA custom config (overrides dwa module defaults)
dwa_config = {
    'DT_SIM': 0.5, # lower sim dt for faster computation
    'T_PRED': 1.5, # shorter prediction horizon for faster computation
    'V_MAX': 0.35,
    'W_MAX': 2.0,
    'ACC_V': 1.0,
    'ACC_W': 2.0,
    'RADIUS': 0.09,
    'SAFE': 0.05,
    'NV': 5, # fewer velocity samples
    'NW': 11, # fewer angular velocity samples
    'LIDAR_SKIP': 4, # skip lidar points for faster computation
    'W_HEAD': 0.8, 
    'W_CLEAR': 0.2,
    'W_VEL': 0.2,
}

def get_yaw(imu, compass) -> float:
    if imu:
        try: return float(imu.getRollPitchYaw()[2])
        except Exception:pass
    if compass:
        try:
            c = compass.getValues()
            return math.atan2(c[0], c[2])
        except Exception: pass
    return 0.0


def process_lidar(lidar, max_range: float = 3.5, min_range: float = 0.1) -> List[Tuple[float, float]]:
    points: List[Tuple[float, float]] = []
    if not lidar: return points
    try:ranges = lidar.getRangeImage()
    except Exception: return points
    if not ranges:return points

    n = len(ranges)
    if n == 0:
        return points

    # Prefer device-provided FOV if available; else assume full circle.
    try:
        fov = float(lidar.getFov())
        if fov <= 0:
            fov = 2 * math.pi
    except Exception:
        fov = 2 * math.pi

    # Compute angle for each point and filter by range
    for i, r in enumerate(ranges):
        if not math.isfinite(r) or r > max_range or r < min_range:
            continue
        angle = -fov / 2 + (i / (n - 1)) * fov if n > 1 else 0.0
        points.append((r * math.cos(angle), r * math.sin(angle)))
    return points





class Navigator:
    def __init__(self, grid):
        self.grid = grid
        self.plan_grid = grid.inflate(0)
        self.astar = AStarPlanner()
        self.path_world: List[Tuple[float, float]] = []
        self.idx = 0

    def plan(self, sx: float, sz: float, gx: float, gz: float) -> bool:
        s_grid = self.plan_grid.world_to_grid(sx, sz)
        g_grid = self.plan_grid.world_to_grid(gx, gz)

        start = s_grid if self._is_free(s_grid) else self._bfs_free(s_grid)
        if start is None:
            return False

        goal = g_grid if self._is_free(g_grid) else self._bfs_free(g_grid)
        if goal is None:
            return False

        path_indices = self.astar.plan(self.plan_grid, start, goal)
        if not path_indices:
            return False

        self.path_world = [self.grid.grid_to_world(r, c) for r, c in path_indices]
        self.idx = 0
        return True

    def _is_free(self, idx: Tuple[int, int]) -> bool:
        try:
            return self.plan_grid.is_free(*idx)
        except Exception:
            return False

    def _bfs_free(self, start: Tuple[int, int], max_dist: int = 8) -> Optional[Tuple[int, int]]:
        q = deque([start])
        seen = {start}
        dist = 0
        while q and dist <= max_dist:
            for _ in range(len(q)):
                r, c = q.popleft()
                if self._is_free((r, c)):
                    return (r, c)
                for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    nr, nc = r + dr, c + dc
                    if (nr, nc) not in seen:
                        seen.add((nr, nc))
                        q.append((nr, nc))
            dist += 1
        return None

    def get_target(self, rx: float, rz: float) -> Optional[Tuple[float, float]]:
        if not self.path_world:
            return None
        while self.idx < len(self.path_world):
            wx, wz = self.path_world[self.idx]
            if math.hypot(wx - rx, wz - rz) < WAYPOINT_TOLERANCE:
                self.idx += 1
            else:
                break
        if self.idx >= len(self.path_world):
            return None
        return self.path_world[min(self.idx + LOOKAHEAD, len(self.path_world) - 1)]


def _get_device(robot: Robot, name: str, ts: int = 0, enable_pointcloud: bool = False):
    dev = robot.getDevice(name)
    if ts:
        if name == LIDAR_NAME and enable_pointcloud:
            dev.enablePointCloud()
        dev.enable(ts)
    return dev


def world_to_robot(dx: float, dy: float, yaw: float) -> Tuple[float, float]:
    cos_y, sin_y = math.cos(-yaw), math.sin(-yaw)
    x_r = dx * cos_y - dy * sin_y
    y_r = dx * sin_y + dy * cos_y
    return x_r, y_r


def wheel_velocities_from(v: float, w: float, wheel_radius: float, axle_length: float) -> Tuple[float, float]:
    w_act = -w  # negative due to coordinate sign convention
    wl = (v - w_act * axle_length * 0.5) / wheel_radius
    wr = (v + w_act * axle_length * 0.5) / wheel_radius
    return wl, wr


def main():
    robot = Robot()
    ts = int(robot.getBasicTimeStep())

    gps = _get_device(robot, GPS_NAME, ts)
    imu = _get_device(robot, IMU_NAME, ts)
    compass = _get_device(robot, COMPASS_NAME, ts)
    lidar = _get_device(robot, LIDAR_NAME, ts, enable_pointcloud=True)

    lm = robot.getDevice(LEFT_MOTOR)
    rm = robot.getDevice(RIGHT_MOTOR)
    for m in (lm, rm):
        m.setPosition(float('inf'))
        m.setVelocity(0.0)

    grid = get_map(verbose=True)
    nav = Navigator(grid)

    wheel_limit = min(lm.getMaxVelocity(), rm.getMaxVelocity())
    robot_v_max = wheel_limit * WHEEL_RADIUS
    robot_w_max = wheel_limit * WHEEL_RADIUS * 2.0 / AXLE_LENGTH
    dwa = DWA(params=dwa_config)

    MISSIONS = [(5.5, 1.6), (3.5, 2.0)]
    m_idx = 0
    path_planned = False
    last_vis = last_replan = 0.0
    prev_cmd = (0.0, 0.0)

    while robot.step(ts) != -1:
        now = robot.getTime()
        if now < 1.0:
            continue

        rx, rz = gps.getValues()[0], gps.getValues()[1]
        yaw = get_yaw(imu, compass)

        if now - last_vis > VIS_INTERVAL:
            path_idx = [grid.world_to_grid(wx, wz) for wx, wz in nav.path_world] if nav.path_world else []
            visualise_robot_on_map(grid, rx, rz, yaw, path=path_idx)
            last_vis = now

        if not path_planned:
            if m_idx >= len(MISSIONS):
                lm.setVelocity(0.0)
                rm.setVelocity(0.0)
                break
            gx, gz = MISSIONS[m_idx]
            if nav.plan(rx, rz, gx, gz):
                path_planned = True
                last_replan = now
            else:
                m_idx += 1
                continue

        if now - last_replan > REPLAN_INTERVAL:
            gx, gz = MISSIONS[m_idx]
            if nav.plan(rx, rz, gx, gz):
                last_replan = now

        target = nav.get_target(rx, rz)
        if not target:
            m_idx += 1
            path_planned = False
            lm.setVelocity(0.0)
            rm.setVelocity(0.0)
            continue

        tx, ty = target
        dx_world, dy_world = tx - rx, ty - rz
        gx_r, gy_r = world_to_robot(dx_world, dy_world, yaw)

        if DWA_ENABLED:
            v, w = dwa.get_safe_velocities(process_lidar(lidar), (gx_r, gy_r), prev_cmd, prev_cmd)
        else:
            # NOTE: atan2 inherently handles normalization, but _ang strictens [-pi, pi)
            ang = _ang(math.atan2(gy_r, gx_r))
            dist = math.hypot(gx_r, gy_r)
            
            K_V = 0.8 # velocity gain
            K_W = - 1.5
            
            v = K_V * dist
            w = K_W * ang

            if abs(ang) > math.pi / 2: v = 0.0 # stop if target is just behind
            
            # bound by robot limits
            v = max(-robot_v_max, min(robot_v_max, v))
            w = max(-robot_w_max, min(robot_w_max, w))

        prev_cmd = (v, w)

        wl, wr = wheel_velocities_from(v, w, WHEEL_RADIUS, AXLE_LENGTH)
        limit = min(lm.getMaxVelocity(), rm.getMaxVelocity())
        lm.setVelocity(max(-limit, min(limit, wl)))
        rm.setVelocity(max(-limit, min(limit, wr)))


if __name__ == "__main__":
    main()