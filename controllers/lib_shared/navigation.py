# ============================================= #
#  NAVIGATION MODULE          -> AUTHOR: ZAHIN  #
# ============================================= #
# A* path lanning w/ dynamic obstacle avoidance #
# Robot-agnostic: input pose/lidar, output (v,w)#
# ============================================= #

import math
from collections import deque
from typing import List, Tuple, Optional

from .global_planner import AStarPlanner
from .map_module import visualise_map, get_map
from .obstacle_avoidance import SimpleAvoidance

class Navigator:
    """Handles path planning and following for differential-drive robots."""
    def __init__(
        self,
        map_inflation: int = 0,
        vis_period: float = 6.0,
        max_v: float = 0.35,
        max_w: float = 2.0
    ):
        # Map + inflated planning grid
        self.grid = get_map()
        self.plan_grid = self.grid.inflate_obstacles(map_inflation)
        self.astar = AStarPlanner()
        self.avoidance = SimpleAvoidance()
        
        # Config
        self.vis_period = vis_period
        self.max_v = max_v
        self.max_w = max_w
        
        # State
        self.path_world: List[Tuple[float, float]] = []
        self.path_idx = 0
        self.last_vis = 0.0
    
    def plan_path(self, start: Tuple[float, float], goal: Tuple[float, float]) -> bool:
        """Plan A* path from start to goal (world coordinates)."""
        sx, sy = start
        gx, gy = goal
        print(f"Planning: ({sx:.2f},{sy:.2f}) → ({gx:.2f},{gy:.2f})")
        
        # Convert to grid and find nearest free cells
        start_node = self._nearest_free(self.plan_grid.world_to_grid(sx, sy))
        goal_node = self._nearest_free(self.plan_grid.world_to_grid(gx, gy))
        
        if not start_node or not goal_node:
            print("ERROR: Invalid start/goal")
            return False
        
        # Run A*
        nodes = self.astar.plan(self.plan_grid, start_node, goal_node)
        if nodes:
            self.path_world = [self.grid.grid_to_world(r, c) for r, c in nodes]
            self.path_idx = 0
            print(f"✓ Path: {len(nodes)} waypoints")
            return True
        
        print("ERROR: No path found")
        return False
    
    def update(self,  pose: Tuple[float, float, float], lidar_pts: List[Tuple[float, float]], now: float ) -> Tuple[float, float, bool]:
        """Update navigation state and compute velocities"""
        rx, ry, yaw = pose

        # Visualise path periodically (if enabled)
        if self.vis_period is not None and now - self.last_vis > self.vis_period:
            vis = [self.grid.world_to_grid(x, z) for x, z in self.path_world]
            visualise_map(self.grid, rx, ry, yaw, path=vis)
            self.last_vis = now
        
        if not self.path_world:
            return 0.0, 0.0, False

        # Find next target waypoint
        target = None
        while self.path_idx < len(self.path_world):
            wx, wy = self.path_world[self.path_idx]
            if math.hypot(wx - rx, wy - ry) < 0.1:
                self.path_idx += 1
            else:
                target = (wx, wy)
                break

        # Goal reached (no more waypoints)
        if not target:
            self.clear_path()
            return 0.0, 0.0, False
        
        # Transform target to robot frame
        dx, dy = target[0] - rx, target[1] - ry
        gx_r = dx * math.cos(-yaw) - dy * math.sin(-yaw)
        gy_r = dx * math.sin(-yaw) + dy * math.cos(-yaw)
        
        # Compute velocities with obstacle avoidance
        v, w = self.avoidance.get_avoidance_velocities(
            lidar_pts, (gx_r, gy_r), self.max_v, self.max_w
        )
        
        return v, w, True
    
    def clear_path(self):
        """Clear current path."""
        self.path_world = []
        self.path_idx = 0
    
    def _nearest_free(self, node: Tuple[int, int]) -> Optional[Tuple[int, int]]:
        """BFS to find nearest free grid cell."""
        if self.plan_grid.is_free(*node):
            return node
        
        q, seen = deque([node]), {node}
        while q:
            r, c = q.popleft()
            if self.plan_grid.is_free(r, c):
                return (r, c)
            
            for dr, dc in [(0,1), (0,-1), (1,0), (-1,0)]:
                nr, nc = r + dr, c + dc
                if (0 <= nr < self.plan_grid.height and 
                    0 <= nc < self.plan_grid.width and 
                    (nr, nc) not in seen):
                    seen.add((nr, nc))
                    q.append((nr, nc))
        
        return None