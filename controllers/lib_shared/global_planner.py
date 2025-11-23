import math
from heapq import heappush, heappop
from typing import List, Tuple, Optional

class OccupancyGrid:
    def __init__(self, width: int, height: int, cell_size: float, origin: Tuple[float, float]):
        self.width, self.height = width, height
        self.cell_size = cell_size
        self.origin = origin
        self.grid = [[0] * width for _ in range(height)]
        
        # Top-Down Logic: Row 0 corresponds to Max Y
        # Max Y = Origin Y (Bottom) + Height * CellSize
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
    
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        # X maps to Column (Left to Right)
        c = math.floor((x - self.origin[0]) / self.cell_size)
        
        # Y maps to Row (Top to Bottom)
        # Row 0 is at Max Y. Row increases as Y decreases.
        r = math.floor((self.max_y - y) / self.cell_size)
        
        return r, c

    def grid_to_world(self, r: int, c: int) -> Tuple[float, float]:
        x = c * self.cell_size + self.origin[0] + self.cell_size/2
        
        # y = Max_Y - (row * cell_size) - half_cell
        y = self.max_y - (r * self.cell_size) - self.cell_size/2
        return x, y
    
    def is_valid(self, r: int, c: int) -> bool:
        return 0 <= r < self.height and 0 <= c < self.width

    def is_free(self, r: int, c: int) -> bool:
        if not self.is_valid(r, c): return False
        return self.grid[r][c] == 0

    def inflate(self, radius: int) -> 'OccupancyGrid':
        new_grid = OccupancyGrid(self.width, self.height, self.cell_size, self.origin)
        new_grid.grid = [row[:] for row in self.grid]
        
        obstacles = [(r,c) for r in range(self.height) for c in range(self.width) if not self.is_free(r,c)]
        
        for r, c in obstacles:
            for dr in range(-radius, radius+1):
                for dc in range(-radius, radius+1):
                    if self.is_valid(r+dr, c+dc):
                        new_grid.grid[r+dr][c+dc] = 1
        return new_grid

class AStarPlanner:
    def __init__(self):
        self.moves = [(-1,0,1), (1,0,1), (0,-1,1), (0,1,1)] + \
                     [(d1, d2, 1.414) for d1 in (-1,1) for d2 in (-1,1)]
    
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