import math
from heapq import heappush, heappop
from typing import List, Tuple, Optional


# Simple grid map where 0=free, 1=obstacle. Coordinates: (row, col).
class OccupancyGrid:
    def __init__(self, width: int, height: int, cell_size: float, origin: Tuple[float, float]):
        self.width, self.height = width, height
        self.cell_size = cell_size
        self.origin = origin  # (x_origin, y_origin)
        self.grid = [[0] * width for _ in range(height)]
        self.max_y = origin[1] + height * cell_size  # y of grid top edge

    # create grid from string representation (MAP_STR)
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


    # Convert between world coordinates and grid indices
    # NOTE: back and forth conversions accumulate errors fast due to flooring
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        # +X increases row, +Y increases col (90° rotated world)
        row = math.floor((x - self.origin[0]) / self.cell_size)
        col = math.floor((y - self.origin[1]) / self.cell_size)
        return row, col

    def grid_to_world(self, r: int, c: int) -> Tuple[float, float]:
        # Inverse of world_to_grid
        x = r * self.cell_size + self.origin[0] + self.cell_size * 0.5
        y = c * self.cell_size + self.origin[1] + self.cell_size * 0.5
        return round(x, 2), round(y, 2)

    def is_valid(self, r: int, c: int) -> bool:
        return 0 <= r < self.height and 0 <= c < self.width

    def is_free(self, r: int, c: int) -> bool:
        return self.is_valid(r, c) and self.grid[r][c] == 0

    # this creates a new inflated grid where obstacles are expanded by 'radius' cells
    def inflate(self, radius: int) -> 'OccupancyGrid':
        new_grid = OccupancyGrid(self.width, self.height, self.cell_size, self.origin)
        new_grid.grid = [row[:] for row in self.grid]  # copy to avoid modifying self
        obstacles = [(r, c) for r in range(self.height) for c in range(self.width) if self.grid[r][c] != 0]
        for r, c in obstacles:
            for dr in range(-radius, radius + 1):
                for dc in range(-radius, radius + 1):
                    nr, nc = r + dr, c + dc
                    if self.is_valid(nr, nc):
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

        # Heap items: (f = g+h, g, current, path)
        queue = [(0.0, 0.0, start, [start])]
        costs = {start: 0.0}

        while queue:
            _, g_curr, curr, path = heappop(queue)
            if curr == goal:
                return path

            if g_curr > costs.get(curr, float('inf')):
                # outdated queue entry
                continue

            for dr, dc, step_cost in self.moves:
                nxt = (curr[0] + dr, curr[1] + dc)
                if not grid.is_free(*nxt):
                    continue
                # Prevent cutting corners through obstacles
                if abs(dr) == 1 and abs(dc) == 1:
                    if not (grid.is_free(curr[0] + dr, curr[1]) and grid.is_free(curr[0], curr[1] + dc)):
                        continue

                new_g = g_curr + step_cost
                if new_g < costs.get(nxt, float('inf')):
                    costs[nxt] = new_g
                    h = math.hypot(goal[0] - nxt[0], goal[1] - nxt[1])
                    heappush(queue, (new_g + h, new_g, nxt, path + [nxt]))
        return None

    def smooth_path(self, grid: OccupancyGrid, path: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        if len(path) <= 2: return path # nothing to smooth

        smoothed = [path[0]]
        i, n = 0, len(path)
        while i < n - 1:
            # try to connect as far ahead as possible
            j = n - 1
            while j > i and not self._line_of_sight(grid, path[i], path[j]):
                j -= 1
            if j == i:
                i += 1
                smoothed.append(path[i])
            else:
                smoothed.append(path[j])
                i = j
        return smoothed

    # Bresenham's line algorithm to check line of sight between two grid cells
    def _line_of_sight(self, grid: OccupancyGrid, p1: Tuple[int, int], p2: Tuple[int, int]) -> bool:
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