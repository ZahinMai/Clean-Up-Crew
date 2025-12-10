# =============================================================================
# NAVIGATION LOGIC - FOR ALL ROBOTS (CAN BE TUNED)            -> Author: ZAHIN
# =============================================================================
import math
from collections import deque
from typing import List, Tuple, Optional

from .global_planner import AStarPlanner
from .map_module import visualise_robot_on_map, get_map
from .local_planner import DWA, _wrap


class Navigator:
    """
    Handles global A* path planning and local path following (DWA or simple)
    for differential-drive robots.

    It is robot-agnostic: you give it pose, lidar points, and time; it gives
    you back (v, w) and whether it's still following a path.
    """

    def __init__(
        self,
        *,
        dwa_enabled: bool = True,
        dwa_config: Optional[dict] = None,
        map_inflation: int = 0,
        vis_period: Optional[float] = 2.0,
        safety_radius: float = 0.09,
        simple_v_max: float = 0.35,
        simple_w_max: float = 2.0,
    ) -> None:
        # Map + inflated planning grid
        self.grid = get_map(verbose=False)
        self.plan_grid = self.grid.inflate(map_inflation)

        # Global planner
        self.astar = AStarPlanner()

        # Local planner (DWA)
        self.dwa_enabled = dwa_enabled
        self.dwa = DWA(dwa_config) if dwa_enabled else None

        # Behaviour tunables
        self.vis_period = vis_period
        self.safety_radius = safety_radius
        self.simple_v_max = simple_v_max
        self.simple_w_max = simple_w_max

        # Internal state
        self.last_vis: float = 0.0
        self.path_world: List[Tuple[float, float]] = []
        self.path_idx: int = 0
        self.current_goal: Optional[Tuple[float, float]] = None
        self.prev_cmd: Tuple[float, float] = (0.0, 0.0)

    # ------------------------------------------------------------------ #
    # Global planning                                                    #
    # ------------------------------------------------------------------ #

    def _find_nearest_free(self, node: Tuple[int, int]) -> Optional[Tuple[int, int]]:
        """BFS to nearest free grid cell from the given node."""
        if self.plan_grid.is_free(*node):
            return node

        q, seen = deque([node]), {node}
        while q:
            r, c = q.popleft()
            if self.plan_grid.is_free(r, c):
                return (r, c)
            for dr, dc in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                nr, nc = r + dr, c + dc
                if (
                    0 <= nr < self.plan_grid.height
                    and 0 <= nc < self.plan_grid.width
                    and (nr, nc) not in seen
                ):
                    seen.add((nr, nc))
                    q.append((nr, nc))
        return None

    def plan_path(
        self,
        start_world: Tuple[float, float],
        goal_world: Tuple[float, float],
    ) -> bool:
        """
        Plan an A* path from start_world (x, y) to goal_world (x, y).
        Returns True on success, False if no path.
        """
        sx, sy = start_world
        gx, gy = goal_world

        print(f"Planning: ({sx:.2f},{sy:.2f}) → ({gx:.2f},{gy:.2f})")

        start_node = self._find_nearest_free(self.plan_grid.world_to_grid(sx, sy))
        goal_node = self._find_nearest_free(self.plan_grid.world_to_grid(gx, gy))

        if not start_node or not goal_node:
            print("ERROR: Invalid start/goal")
            return False

        nodes = self.astar.plan(self.plan_grid, start_node, goal_node)
        if nodes:
            self.path_world = [self.grid.grid_to_world(r, c) for r, c in nodes]
            self.path_idx = 0
            self.current_goal = (gx, gy)
            print(f"✓ Path found: {len(nodes)} waypoints")
            return True

        print("ERROR: No path found")
        self.path_world = []
        self.path_idx = 0
        self.current_goal = None
        return False

    # ------------------------------------------------------------------ #
    # Local path following                                               #
    # ------------------------------------------------------------------ #

    @staticmethod
    def _min_obstacle_distance(lidar_pts: List[Tuple[float, float]]) -> float:
        if not lidar_pts:
            return float("inf")
        return min(math.hypot(x, y) for x, y in lidar_pts)

    def _simple_controller(
        self,
        gx_r: float,
        gy_r: float,
        min_obs_dist: float,
    ) -> Tuple[float, float]:
        """Go-to-goal with obstacle slowdown, no DWA."""
        dist = math.hypot(gx_r, gy_r)
        ang = _wrap(math.atan2(gy_r, gx_r))

        v = max(-self.simple_v_max, min(self.simple_v_max, 0.5 * dist))
        w = max(-self.simple_w_max, min(self.simple_w_max, -2.5 * ang))

        # Slow down near obstacles
        if min_obs_dist < self.safety_radius:
            v *= min_obs_dist

        # Turn in place for large heading errors
        if abs(ang) > 0.5:
            v *= 0.3
            w *= 1.5

        return v, w

    def update(
        self,
        *,
        pose: Tuple[float, float, float],
        lidar_pts: List[Tuple[float, float]],
        now: float,
    ) -> Tuple[float, float, bool]:
        """
        Main navigation update.

        Args:
          pose: (x, y, yaw) in world frame.
          lidar_pts: list of (x, y) points in robot frame (x forward, y left).
          now: current simulation time in seconds.

        Returns:
          (v, w, moving)
            v, w: commanded linear and angular velocities
            moving: True if following a path, False if goal reached or no path
        """
        rx, ry, yaw = pose

        # Visualise path periodically (if enabled)
        if self.vis_period is not None and now - self.last_vis > self.vis_period:
            vis = [self.grid.world_to_grid(x, z) for x, z in self.path_world]
            visualise_robot_on_map(self.grid, rx, ry, yaw, path=vis)
            self.last_vis = now

        if not self.path_world:
            return 0.0, 0.0, False

        # Find next target waypoint
        target = None
        while self.path_idx < len(self.path_world):
            wx, wz = self.path_world[self.path_idx]
            if math.hypot(wx - rx, wz - ry) < 0.1:
                self.path_idx += 1
            else:
                target = self.path_world[self.path_idx]
                break

        # Goal reached (no more waypoints)
        if target is None:
            self.clear_path()
            return 0.0, 0.0, False

        # Transform waypoint into robot frame
        dx, dy = target[0] - rx, target[1] - ry
        gx_r = dx * math.cos(-yaw) - dy * math.sin(-yaw)
        gy_r = dx * math.sin(-yaw) + dy * math.cos(-yaw)

        min_obs_dist = self._min_obstacle_distance(lidar_pts)

        # Choose controller
        if self.dwa_enabled and self.dwa is not None:
            v, w = self.dwa.get_safe_velocities(
                lidar_pts,
                (gx_r, gy_r),
                self.prev_cmd,
                self.prev_cmd,
            )
        else:
            v, w = self._simple_controller(gx_r, gy_r, min_obs_dist)

        self.prev_cmd = (v, w)
        return v, w, True

    # ------------------------------------------------------------------ #
    # Utility                                                            #
    # ------------------------------------------------------------------ #

    def has_path(self) -> bool:
        return bool(self.path_world)

    def clear_path(self) -> None:
        self.path_world = []
        self.path_idx = 0
        self.current_goal = None
        self.prev_cmd = (0.0, 0.0)
