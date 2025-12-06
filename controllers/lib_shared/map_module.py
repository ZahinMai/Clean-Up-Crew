# ============================================= #
#  WORLD MAP & POS DEBUG     -> AUTHOR: ZAHIN   #
# ============================================= #
import math
from typing import Tuple

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

# --- CONFIGURATION ---
GRID_W, GRID_H = 36, 24 # cells
DEFAULT_CELL_SIZE = 8/24 # meters
ORIGIN = -4, -6 # meters (bottom-left corner in world coordinates)


CAFETERIA_MAP = """
####################################
#..................................#
#..................................#
#......####.....####.....####......#
#......####.....####.....####......#
#......####.....####.....####......#
#......####.....####.....####......#
#......####.....####.....####......#
#..................................#
#..................................#
#......####.....####.....####......#
#......####.....####.....####......#
#......####.....####.....####......#
#......####.....####.....####......#
#......####.....####.....####......#
#..................................#
#..................................#
#......####.....####.....####......#
#......####.....####.....####......#
#......####.....####.....####......#
#......####.....####.....####......#
#......####.....####.....####......#
#..................................#
#..................................#
####################################
"""

def get_map(robot=None, gps=None, timestep=None, force_recalibrate=False, verbose=True) -> OccupancyGrid:
    return OccupancyGrid.from_string(CAFETERIA_MAP, DEFAULT_CELL_SIZE, ORIGIN)

def visualise_robot_on_map(grid: OccupancyGrid, rx: float, rz: float, yaw: float, path = None):
    r_grid = grid.world_to_grid(rx, rz)
    
    # Prepare Path and Heading
    path_set = set(path) if path else set()
    vis_angle = yaw - math.pi/2 
    angle = vis_angle % (2 * math.pi)
    sector = int((angle + math.pi/8) / (math.pi/4)) % 8
    arrows_ccw = ['→', '↗', '↑', '↖', '←', '↙', '↓', '↘']
    arrow = arrows_ccw[sector-2]

    print(f"\nMap: Robot {r_grid} {arrow} | World ({rx:.2f}, {rz:.2f})")
    
    # Ensure robot is within bounds
    if not grid.is_valid(r_grid[0], r_grid[1]):
        print(f"ROBOT OUT OF BOUNDS! Grid: {r_grid}")
        return

    # View Window -> Simple centering around bot for visualisation
    VIEW_RADIUS_R = 6  # Rows
    VIEW_RADIUS_C = 10  # Columns

    center_r, center_c = r_grid
    
    start_r = max(0, center_r - VIEW_RADIUS_R)
    end_r = min(grid.height, center_r + VIEW_RADIUS_R + 1)

    start_c = max(0, center_c - VIEW_RADIUS_C)
    end_c = min(grid.width, center_c + VIEW_RADIUS_C + 1)
    
    
    #  For each row print the cells as # . + →
    for r in range(start_r, end_r):
        line = []
        for c in range(start_c, end_c):
            if (r, c) == r_grid:
                line.append(arrow)  # Robot position
            elif (r, c) in path_set:
                line.append('+')    # Path node
            elif grid.grid[r][c] == 1:
                line.append('#')    # Obstacle
            else:
                line.append('.')    # Free space
        print(f"{r:02d} {''.join(line)}")


# ============================================= #
#                DEBUG FUNCTIONS                #
# ============================================= #
def debug_position(grid, robot_id, rx, ry, yaw):
    """Print real GPS position vs mapped grid position."""
    # Current mapped position (what code uses)
    mapped_x, mapped_z = rx, ry
    
    # Convert to grid coordinates
    grid_pos = grid.world_to_grid(mapped_x, mapped_z)
    
    print(f"\n{'='*60}")
    print(f"[{robot_id}] POSITION DEBUG")
    print(f"{'='*60}")
    print(f"Real GPS:    X={rx:.3f}, Y={ry:.3f}, ANGLE:{yaw:.3f}")
    print(f"Mapped:      rx={mapped_x:.3f}, rz={mapped_z:.3f}")
    print(f"Grid:        row={grid_pos[0]}, col={grid_pos[1]}")
    print(f"Grid origin: {grid.origin}")
    print(f"Cell size:   {grid.cell_size:.3f}")
    print(f"{'='*60}\n")


def check_map_alignment(grid):
    print("Corner Check:")
    corners = [(0,0), (0, grid.width-1), (grid.height-1, 0), (grid.height-1, grid.width-1)]
    names = ["Top-Left", "Top-Right", "Bottom-Left", "Bottom-Right"]
    for (r,c), name in zip(corners, names):
        wx, wy = grid.grid_to_world(r, c)
        state = "OCCUPIED" if not grid.is_free(r,c) else "FREE"
        print(f"  {name:12} grid ({r:2d},{c:2d}) = world ({wx:5.2f}, {wy:5.2f}) [{state}]")