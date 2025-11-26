import math
from typing import Tuple, Optional, List
from .global_planner import OccupancyGrid

# --- CONFIGURATION ---
GRID_W, GRID_H = 35, 21 # cells
DEFAULT_CELL_SIZE = 7.7/21 # meters
ORIGIN = -5.8, -3 # meters (bottom-left corner in world coordinates)
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

def get_map(robot=None, gps=None, timestep=None, force_recalibrate=False, verbose=True) -> OccupancyGrid:
    return OccupancyGrid.from_string(CAFETERIA_MAP, DEFAULT_CELL_SIZE, ORIGIN)

def visualise_robot_on_map(grid: OccupancyGrid, rx: float, rz: float, yaw: float, path = None):
    r_grid = grid.world_to_grid(rx, rz)
    
    # Prepare Path and Heading
    path_set = set(path) if path else set()
    angle = yaw % (2 * math.pi)
    sector = int((angle + math.pi/8) / (math.pi/4)) % 8
    arrows_ccw = ['→', '↗', '↑', '↖', '←', '↙', '↓', '↘']
    arrow = arrows_ccw[sector]

    print(f"\nMap: Robot {r_grid} {arrow} | World ({rx:.2f}, {rz:.2f})")
    
    # Ensure robot is within bounds
    if not grid.is_valid(r_grid[0], r_grid[1]):
        print(f"ROBOT OUT OF BOUNDS! Grid: {r_grid}")
        return

    # Define View Window (Simple centering around the robot)
    VIEW_RADIUS_R = 10  # Rows
    VIEW_RADIUS_C = 17  # Columns

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

def check_map_alignment(grid):
    print("Corner Check:")
    corners = [(0,0), (0, grid.width-1), (grid.height-1, 0), (grid.height-1, grid.width-1)]
    names = ["Top-Left", "Top-Right", "Bottom-Left", "Bottom-Right"]
    for (r,c), name in zip(corners, names):
        wx, wy = grid.grid_to_world(r, c)
        state = "OCCUPIED" if not grid.is_free(r,c) else "FREE"
        print(f"  {name:12} grid ({r:2d},{c:2d}) = world ({wx:5.2f}, {wy:5.2f}) [{state}]")