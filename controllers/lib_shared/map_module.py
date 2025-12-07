# ============================================= #
#  WORLD MAP & POS DEBUG     -> AUTHOR: ZAHIN   #
# ============================================= #
import math
from typing import Tuple

DEFAULT_CELL_SIZE = 8 / 24    # 0.333 meters
ORIGIN = (-4.0, -6.0)         # (Min X, Min Y) - Bottom-Left corner
# Easier to draw like this - Will TRANSPOSE so Width=X and Height=Y
CAFETERIA_MAP = """
####################################
#..................................#
#..................................#
#....######....######....######....#
#....######....######....######....#
#....######....######....######....#
#....######....######....######....#
#....######....######....######....#
#..................................#
#..................................#
#....######....######....######....#
#....######....######....######....#
#....######....######....######....#
#....######....######....######....#
#....######....######....######....#
#..................................#
#..................................#
#....######....######....######....#
#....######....######....######....#
#....######....######....######....#
#....######....######....######....#
#....######....######....######....#
#..................................#
#..................................#
####################################
"""

class OccupancyGrid:
    def __init__(self, width: int, height: int, cell_size: float, origin: Tuple[float, float]):
        self.width = width   # Corresponds to X
        self.height = height # Corresponds to Y
        self.cell_size = cell_size
        self.origin = origin
        # Grid accessed as grid[y_index][x_index] (Row=Y, Col=X)
        self.grid = [[0] * width for _ in range(height)]

    @classmethod
    def from_string(cls, map_str: str, cell_size: float, origin: Tuple[float, float]) -> 'OccupancyGrid':
        # 1. Parse raw lines (Original: Rows=X, Cols=Y)
        raw_lines = [l.rstrip('\n') for l in map_str.strip().splitlines() if l.strip()]
        
        # 2. Transpose Logic
        transposed_data = list(zip(*raw_lines))
        
        # 3. Create Grid
        h_new = len(transposed_data)    # New Height (Y range)
        w_new = len(transposed_data[0]) # New Width (X range)
        
        obj = cls(w_new, h_new, cell_size, origin)
        
        # 4. Fill Grid (1=Wall, 0=Free)
        for r in range(h_new):
            for c in range(w_new):
                if transposed_data[r][c] == '#':
                    obj.grid[r][c] = 1
        return obj

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        # Maps World (X, Y) to Grid (Col, Row)
        # Row (Y index)
        row = math.floor((y - self.origin[1]) / self.cell_size)
        # Col (X index)
        col = math.floor((x - self.origin[0]) / self.cell_size)
        return row, col

    def grid_to_world(self, r: int, c: int) -> Tuple[float, float]:
        # Maps Grid (Row, Col) to World (Y, X)
        y = (r + 0.5) * self.cell_size + self.origin[1]
        x = (c + 0.5) * self.cell_size + self.origin[0]
        return round(x, 2), round(y, 2)

    def is_valid(self, r: int, c: int) -> bool:
        return 0 <= r < self.height and 0 <= c < self.width

    def is_free(self, r: int, c: int) -> bool:
        return self.is_valid(r, c) and self.grid[r][c] == 0

    def inflate(self, radius: int) -> 'OccupancyGrid':
        new_grid = OccupancyGrid(self.width, self.height, self.cell_size, self.origin)
        new_grid.grid = [row[:] for row in self.grid]
        
        for r in range(self.height):
            for c in range(self.width):
                if self.grid[r][c] == 1:
                    r_min = max(0, r - radius)
                    r_max = min(self.height, r + radius + 1)
                    c_min = max(0, c - radius)
                    c_max = min(self.width, c + radius + 1)
                    
                    for nr in range(r_min, r_max):
                        for nc in range(c_min, c_max):
                            new_grid.grid[nr][nc] = 1
        return new_grid


def get_map(verbose=False) -> OccupancyGrid:
    return OccupancyGrid.from_string(CAFETERIA_MAP, DEFAULT_CELL_SIZE, ORIGIN)


def visualise_robot_on_map(grid: OccupancyGrid, rx: float, rz: float, yaw: float, robot_id=None, path=None):
    r_bot, c_bot = grid.world_to_grid(rx, rz)
    path_set = set(path) if path else set()

    sector = int((yaw + math.pi/8) / (math.pi/4)) % 8
    arrows = ['→', '↗', '↑', '↖', '←', '↙', '↓', '↘']
    arrow = arrows[sector]

    print(f"\nMap: Robot ({r_bot}, {c_bot}) {arrow} | World ({rx:.2f}, {rz:.2f})")
    
    # View Window (Centered on Robot)
    RADIUS_R, RADIUS_C = 5, 8
    r_start = max(0, r_bot - RADIUS_R)
    r_end = min(grid.height, r_bot + RADIUS_R + 1)
    c_start = max(0, c_bot - RADIUS_C)
    c_end = min(grid.width, c_bot + RADIUS_C + 1)
    
    # PRINT IN REVERSE ROW ORDER (Max Y at Top, Min Y at Bottom)
    for r in range(r_end - 1, r_start - 1, -1):
        line = []
        for c in range(c_start, c_end):
            if (r, c) == (r_bot, c_bot):
                line.append(arrow)
            elif (r, c) in path_set:
                line.append('+')
            elif grid.grid[r][c] == 1:
                line.append('#')
            else:
                line.append('.')
        print(f"{r:02d} {''.join(line)}")
 