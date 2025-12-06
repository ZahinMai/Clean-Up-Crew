# ============================================= #
#  A* PATH PLANNING        -> AUTHOR: ZAHIN     #
# ============================================= #

import math
from heapq import heappush, heappop
from typing import List, Tuple, Optional, Dict

# Simple grid map where 0=free, 1=obstacle. Coordinates: (row, col).
class OccupancyGrid:
    def __init__(self, width: int, height: int, cell_size: float, origin: Tuple[float, float]):
        self.width, self.height = width, height
        self.cell_size = cell_size
        self.origin = origin
        self.grid = [[0] * width for _ in range(height)]
        self.max_y = origin[1] + height * cell_size

    @classmethod
    def from_string(cls, map_str: str, cell_size: float, origin: Tuple[float, float]) -> 'OccupancyGrid':
        lines = [l.rstrip('\n') for l in map_str.strip().splitlines() if l.strip()]
        h, w = len(lines), max(len(l) for l in lines)
        obj = cls(w, h, cell_size, origin)
        for r, line in enumerate(lines):
            for c, ch in enumerate(line):
                if ch == '#':
                    obj.grid[r][c] = 1
        return obj

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        row = math.floor((x - self.origin[0]) / self.cell_size)
        col = math.floor((y - self.origin[1]) / self.cell_size)
        return row, col

    def grid_to_world(self, r: int, c: int) -> Tuple[float, float]:
        x = r * self.cell_size + self.origin[0] + self.cell_size * 0.5
        y = c * self.cell_size + self.origin[1] + self.cell_size * 0.5
        return round(x, 2), round(y, 2)

    # CRITICAL: Kept for API compatibility with your map_module.py
    def is_valid(self, r: int, c: int) -> bool:
        return 0 <= r < self.height and 0 <= c < self.width

    def is_free(self, r: int, c: int) -> bool:
        return self.is_valid(r, c) and self.grid[r][c] == 0

    def inflate(self, radius: int) -> 'OccupancyGrid':
        new_grid = OccupancyGrid(self.width, self.height, self.cell_size, self.origin)
        # Deep copy the grid to avoid reference issues
        new_grid.grid = [row[:] for row in self.grid]
        
        obstacles = [(r, c) for r in range(self.height) for c in range(self.width) if self.grid[r][c] != 0]
        for r, c in obstacles:
            # Optimized bounds to avoid checking indices that will definitely be invalid
            r_min = max(0, r - radius)
            r_max = min(self.height, r + radius + 1)
            c_min = max(0, c - radius)
            c_max = min(self.width, c + radius + 1)
            
            for nr in range(r_min, r_max):
                for nc in range(c_min, c_max):
                    new_grid.grid[nr][nc] = 1
        return new_grid


# Uses 8 directional movement, Euclidean distance heuristic, and path smoothing
class AStarPlanner:
    def __init__(self):
        diag_cost = math.sqrt(2)
        # (delta_row, delta_col, move_cost)
        self.moves = [(-1, 0, 1), (1, 0, 1), (0, -1, 1), (0, 1, 1)] + \
                     [(-1, -1, diag_cost), (-1, 1, diag_cost), (1, -1, diag_cost), (1, 1, diag_cost)]

    def plan(self, grid: OccupancyGrid, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        """Return grid cell path from start to goal or None if unreachable."""
        if not (grid.is_free(*start) and grid.is_free(*goal)):
            return None

        # Optimization: Use Parent Pointers (came_from) instead of storing full path in heap.
        # Heap items: (f_score, start_node)
        queue = [(0.0, start)]
        
        g_scores = {start: 0.0}
        came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}

        while queue:
            _, curr = heappop(queue)

            if curr == goal:
                return self._reconstruct_path(came_from, curr)

            curr_g = g_scores[curr]

            for dr, dc, step_cost in self.moves:
                nxt = (curr[0] + dr, curr[1] + dc)
                
                if not grid.is_free(*nxt):
                    continue
                
                # Prevent cutting corners through obstacles
                if abs(dr) == 1 and abs(dc) == 1:
                    if not (grid.is_free(curr[0] + dr, curr[1]) and grid.is_free(curr[0], curr[1] + dc)):
                        continue

                new_g = curr_g + step_cost
                
                # Only explore if we found a cheaper path to 'nxt'
                if new_g < g_scores.get(nxt, float('inf')):
                    came_from[nxt] = curr
                    g_scores[nxt] = new_g
                    h = math.hypot(goal[0] - nxt[0], goal[1] - nxt[1])
                    heappush(queue, (new_g + h, nxt))
        
        return None

    def _reconstruct_path(self, came_from: Dict, current: Tuple[int, int]) -> List[Tuple[int, int]]:
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]

    def smooth_path(self, grid: OccupancyGrid, path: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        if len(path) <= 2: return path 

        smoothed = [path[0]]
        i, n = 0, len(path)
        while i < n - 1:
            j = n - 1
            # Greedy check from the end backwards
            while j > i:
                if self._line_of_sight(grid, path[i], path[j]):
                    break
                j -= 1
            
            if j == i: # Should not happen if path is valid, but safety valve
                j = i + 1
            
            smoothed.append(path[j])
            i = j
        return smoothed

    def _line_of_sight(self, grid: OccupancyGrid, p1: Tuple[int, int], p2: Tuple[int, int]) -> bool:
        """Bresenham's Line Algorithm."""
        r0, c0 = p1
        r1, c1 = p2
        dr = abs(r1 - r0)
        dc = abs(c1 - c0)
        sr = 1 if r0 < r1 else -1
        sc = 1 if c0 < c1 else -1
        err = dr - dc
        r, c = r0, c0

        while True:
            if not grid.is_free(r, c):
                return False
            if (r, c) == (r1, c1):
                return True
            e2 = 2 * err
            if e2 > -dc:
                err -= dc
                r += sr
            if e2 < dr:
                err += dr
                c += sc