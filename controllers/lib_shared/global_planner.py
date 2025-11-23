import math
from heapq import heappush, heappop
from typing import List, Tuple, Optional

class OccupancyGrid:
    def __init__(self, width: int, height: int, cell_size: float, origin: Tuple[float, float]):
        self.width, self.height = width, height
        self.cell_size = cell_size
        self.origin = origin
        self.grid = [[0] * width for _ in range(height)]
        self.max_y = origin[1] + height * cell_size
    
    @classmethod
    def from_string(cls, map_str: str, cell_size: float, origin: Tuple[float, float]) -> 'OccupancyGrid':
        lines = [l.strip() for l in map_str.strip().split('\n') if l.strip()]
        h, w = len(lines), max(len(l) for l in lines)
        obj = cls(w, h, cell_size, origin)
        
        for r, line in enumerate(lines):
            for c, char in enumerate(line):
                if char == '#': obj.grid[r][c] = 1
        return obj
    
    # NOTE:  [-x, x] is left -> right, [+y, y] is bottom -> top
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        c = math.floor((x - self.origin[0]) / self.cell_size)
        r = math.floor((self.max_y - y) / self.cell_size)        
        return r, c

    # NOTE: returns center of cell
    def grid_to_world(self, r: int, c: int) -> Tuple[float, float]:
        x = c * self.cell_size + self.origin[0] + self.cell_size/2
        y = self.max_y - (r * self.cell_size) - self.cell_size/2
        return x, y
    
    # NOTE: Cconverting baxk and forth will lose precision due to flooring

    def is_valid(self, r: int, c: int) -> bool:
        return 0 <= r < self.height and 0 <= c < self.width

    def is_free(self, r: int, c: int) -> bool:
        if not self.is_valid(r, c): return False
        return self.grid[r][c] == 0

    def inflate(self, radius: int) -> 'OccupancyGrid':
        new_grid = OccupancyGrid(self.width, self.height, self.cell_size, self.origin)
        new_grid.grid = [row[:] for row in self.grid]
        obstacles = [(r,c) for r in range(self.height) for c in range(self.width) if not self.is_free(r,c)]
        
        # add 1s aeround each obstacle cell
        for r, c in obstacles:
            for dr in range(-radius, radius+1):
                for dc in range(-radius, radius+1):
                    if self.is_valid(r+dr, c+dc):
                        new_grid.grid[r+dr][c+dc] = 1
        return new_grid

class AStarPlanner:
    def __init__(self):
        self.moves = [(-1,0,1), (1,0,1), (0,-1,1), (0,1,1)] + [(d1, d2, 1.414) for d1 in (-1,1) for d2 in (-1,1)]
    
    def plan(self, grid: OccupancyGrid, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        if not (grid.is_free(*start) and grid.is_free(*goal)): return None

        queue = [(0, 0, start, [start])]
        seen = {start: 0}
        
        while queue:
            _, _, curr, path = heappop(queue)
            if curr == goal: return path
            
            for dr, dc, cost in self.moves:
                nxt = (curr[0]+dr, curr[1]+dc)
                if not grid.is_free(*nxt): continue
                if abs(dr) == 1 and abs(dc) == 1:
                    if not (grid.is_free(curr[0]+dr, curr[1]) and grid.is_free(curr[0], curr[1]+dc)):
                        continue

                new_g = seen[curr] + cost
                if nxt not in seen or new_g < seen[nxt]:
                    seen[nxt] = new_g
                    h = math.hypot(goal[0]-nxt[0], goal[1]-nxt[1])
                    heappush(queue, (new_g + h, 0, nxt, path + [nxt]))
        return None
    

    '''
    Use two-pointers to check if a direct line of sight exists between non-adjacent
    waypoints using Bresenham's line algorithm, removing unnecessary intermediate waypoints.
    '''
    # ============= Path Smoothing with Line of Sight =============
    def smooth_path(self, grid: OccupancyGrid, path: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        if len(path) <= 2: return path # nothing to smooth

        smoothed = [path[0]]
        i, n = 0, len(path)
        # two-pointer approach to skip as many waypoints as possible
        while i < n - 1:
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
    
    # ============= Bresenham's line algorithm for line of sight =============
    # -> separated for readability
    def _line_of_sight(self, grid: OccupancyGrid, p1: Tuple[int, int],  p2: Tuple[int, int]) -> bool:
        r0, c0 = p1
        r1, c1 = p2
        
        dr = abs(r1 - r0) # delta row
        dc = abs(c1 - c0) # delta column
        sr = 1 if r0 < r1 else -1 # step row
        sc = 1 if c0 < c1 else -1 # step column
        err = dr - dc
        
        r, c = r0, c0
        
        while True:
            if not grid.is_free(r, c):
                return False
            
            if r == r1 and c == c1:
                return True
            
            e2 = 2 * err
            if e2 > -dc:
                err -= dc
                r += sr
            if e2 < dr:
                err += dr
                c += sc