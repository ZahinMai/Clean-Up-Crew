import math
from typing import Tuple, Optional, List
from .global_planner import OccupancyGrid

# --- CONFIGURATION ---
# 35 columns (Width), 21 rows (Height)
GRID_W, GRID_H = 35, 21

# World: 12.86m x 7.7m. 
DEFAULT_CELL_SIZE = 7.7/21 # meters

# Anchor: Top-Left World Coordinates
TOP_LEFT_X = -6.0
TOP_LEFT_Y = 4.7

# The 21x35 Map String
CAFETERIA_MAP = """
###################################
#...................###############
#...................###############
#.....########....................#
#.....########....................#
#.....########....................#
#.....##..........................#
#.....##..........................#
#.....##..........................#
#.....##..####..####..#...####....#
#.....##..####..####..#...####....#
#.....##..####..####..#...####....#
#.....##..####..####..#...####....#
#.....##..............#...........#
#.....##..####..####..#...####....#
#.....##..####..####..#...####....#
#.....##..####..####..#...####....#
#.....##..####..####..#...####....#
#.....##..............#...........#
#.....#################...........#
###################################
"""

class MapService:
    @staticmethod
    def calculate_origin(top_left_x: float, top_left_y: float, 
                         height_cells: int, cell_size: float) -> Tuple[float, float]:
        # Origin X (Min X) is the Left Edge
        origin_x = top_left_x
        
        # Origin Y (Min Y) is Top Edge minus Height
        origin_y = top_left_y - (height_cells * cell_size)
        
        return (origin_x, origin_y)

    @staticmethod
    def get_map() -> OccupancyGrid:
        # Calculate origin dynamically from the anchor point
        origin = MapService.calculate_origin(TOP_LEFT_X, TOP_LEFT_Y, GRID_H, DEFAULT_CELL_SIZE)
        print(f"Map Loaded. Anchor: ({TOP_LEFT_X}, {TOP_LEFT_Y}) -> Origin: {origin}")
        # Using positional args for robustness
        return OccupancyGrid.from_string(CAFETERIA_MAP, DEFAULT_CELL_SIZE, origin)

    @staticmethod
    def visualise(grid: OccupancyGrid, rx: float, rz: float, yaw: float, 
                  goals: Optional[List[Tuple[float, float]]] = None, path=None):
        r_grid = grid.world_to_grid(rx, rz)
        path_set = set(path) if path else set()
        
        angle = yaw % (2 * math.pi)
        sector = int((angle + math.pi/8) / (math.pi/4)) % 8
        arrows_ccw = ['→', '↗', '↑', '↖', '←', '↙', '↓', '↘']
        arrow = arrows_ccw[sector]

        print(f"\nMap: Robot {r_grid} {arrow} | World ({rx:.2f}, {rz:.2f})")
        
        # Simple clamping to keep the print window valid
        center_r = max(0, min(grid.height-1, r_grid[0]))
        center_c = max(0, min(grid.width-1, r_grid[1]))
        
        start_r = max(0, center_r - 10)
        end_r = min(grid.height, start_r + 20)
        start_r = max(0, end_r - 20)

        start_c = max(0, center_c - 17)
        end_c = min(grid.width, start_c + 34)
        start_c = max(0, end_c - 34)

        for r in range(start_r, end_r):
            line = []
            for c in range(start_c, end_c):
                if (r,c) == r_grid: line.append(arrow)
                elif (r,c) in path_set: line.append('+')
                elif grid.grid[r][c] == 1: line.append('#')
                else: line.append('.')
            print(f"{r:02d} {''.join(line)}")
        
        if not grid.is_valid(r_grid[0], r_grid[1]):
            print(f"⚠ ROBOT OUT OF BOUNDS! Grid: {r_grid}")
            
        print("-" * 40)

def get_map(robot=None, gps=None, timestep=None, force_recalibrate=False, verbose=True):
    return MapService.get_map()

def visualise_robot_on_map(grid, rx, rz, yaw, gx=None, gz=None, path=None):
    MapService.visualise(grid, rx, rz, yaw, path=path)

def check_map_alignment(grid):
    print("Corner Check:")
    corners = [(0,0), (0, grid.width-1), (grid.height-1, 0), (grid.height-1, grid.width-1)]
    names = ["Top-Left", "Top-Right", "Bottom-Left", "Bottom-Right"]
    for (r,c), name in zip(corners, names):
        wx, wy = grid.grid_to_world(r, c)
        state = "OCCUPIED" if not grid.is_free(r,c) else "FREE"
        print(f"  {name:12} grid ({r:2d},{c:2d}) = world ({wx:5.2f}, {wy:5.2f}) [{state}]")