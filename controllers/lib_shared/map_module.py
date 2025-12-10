# ============================================= #
#  WORLD MAP & POS DEBUG     -> AUTHOR: ZAHIN   #
# ============================================= #
import math
from typing import Tuple, List, Optional
CELL_SIZE = 8/24  # meters per grid cell
WORLD_MIN_X = -4.0
WORLD_MIN_Y = -6.0
# Cafeteria layout (# = wall, . = free space)
CAFETERIA_LAYOUT = """
########################
#......................#
#......................#
#......................#
#..####...####...####..#
#..####...####...####..#
#..####...####...####..#
#..####...####...####..#
#..####...####...####..#
#..####...####...####..#
#......................#
#......................#
#......................#
#......................#
#......................#
#..####...####...####..#
#..####...####...####..#
#..####...####...####..#
#..####...####...####..#
#..####...####...####..#
#..####...####...####..#
#......................#
#......................#
#......................#
#......................#
#......................#
#..####...####...####..#
#..####...####...####..#
#..####...####...####..#
#..####...####...####..#
#..####...####...####..#
#..####...####...####..#
#......................#
#......................#
#......................#
########################
"""

class OccupancyGrid:
    """
    2D occupancy grid for navigation.
    Grid coordinates: grid[row][col] where row increases with Y, col increases with X
    World coordinates: (x, y) in meters
    """
    def __init__(self, width: int, height: int, cell_size: float, origin_x: float, origin_y: float):
        self.width = width
        self.height = height
        self.cell_size = cell_size
        self.origin_x = origin_x
        self.origin_y = origin_y
        
        # Grid: 0 = free, 1 = occupied
        self.grid = [[0] * width for _ in range(height)]

    @classmethod
    def from_layout(cls, layout: str, cell_size: float, origin_x: float, origin_y: float) -> 'OccupancyGrid':
        """Create grid from ASCII layout string."""
        lines = [line.rstrip() for line in layout.strip().splitlines() if line.strip()]
        
        height = len(lines)
        width = max(len(line) for line in lines)
        
        grid = cls(width, height, cell_size, origin_x, origin_y)
        
        # Fill grid (rows are stored bottom-to-top, so reverse the input)
        for row_idx, line in enumerate(reversed(lines)):
            for col_idx, char in enumerate(line):
                if char == '#':
                    grid.grid[row_idx][col_idx] = 1
        
        return grid

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates (x, y) to grid coordinates (row, col)."""
        col = int((x - self.origin_x) / self.cell_size)
        row = int((y - self.origin_y) / self.cell_size)
        return row, col

    def grid_to_world(self, row: int, col: int) -> Tuple[float, float]:
        """Convert grid coordinates (row, col) to world coordinates (x, y) at cell center."""
        x = self.origin_x + (col + 0.5) * self.cell_size
        y = self.origin_y + (row + 0.5) * self.cell_size
        return x, y

    def is_valid(self, row: int, col: int) -> bool:
        """Check if grid coordinates are within bounds."""
        return 0 <= row < self.height and 0 <= col < self.width

    def is_free(self, row: int, col: int) -> bool:
        """Check if grid cell is free (not occupied)."""
        return self.is_valid(row, col) and self.grid[row][col] == 0

    def inflate_obstacles(self, radius: int) -> 'OccupancyGrid':
        """ Create new grid with inflated obstacles for collision avoidance """
        inflated = OccupancyGrid(self.width, self.height, self.cell_size, 
                                 self.origin_x, self.origin_y)
        
        inflated.grid = [row[:] for row in self.grid] # copy og grid
        
        # Inflate each obstacle
        for row in range(self.height):
            for col in range(self.width):
                if self.grid[row][col] == 1:
                    # Mark cells within radius as occupied
                    for dr in range(-radius, radius + 1):
                        for dc in range(-radius, radius + 1):
                            new_row = row + dr
                            new_col = col + dc
                            if inflated.is_valid(new_row, new_col):
                                inflated.grid[new_row][new_col] = 1
        
        return inflated


def get_map() -> OccupancyGrid:
    return OccupancyGrid.from_layout(CAFETERIA_LAYOUT, CELL_SIZE, WORLD_MIN_X, WORLD_MIN_Y)


def visualise_map(grid: OccupancyGrid, robot_x: float, robot_y: float, robot_yaw: float, 
                  path: Optional[List[Tuple[int, int]]] = None, view_radius: int = 5):
    robot_row, robot_col = grid.world_to_grid(robot_x, robot_y)
    path_set = set(path) if path else set()
    
    # Determine arrow direction based on yaw
    arrows = ['→', '↗', '↑', '↖', '←', '↙', '↓', '↘']
    sector = int((robot_yaw + math.pi/8) / (math.pi/4)) % 8
    arrow = arrows[sector]
    
    print(f"\n=== Map View ===")
    print(f"Robot Position: World({robot_x:.2f}, {robot_y:.2f}) Grid({robot_row}, {robot_col}) {arrow}")
    
    # Calculate view window
    row_min = max(0, robot_row - view_radius)
    row_max = min(grid.height, robot_row + view_radius + 1)
    col_min = max(0, robot_col - view_radius * 2)
    col_max = min(grid.width, robot_col + view_radius * 2 + 1)
    
    # Print grid (top to bottom = high Y to low Y)
    for row in range(row_max - 1, row_min - 1, -1):
        line = []
        for col in range(col_min, col_max):
            if (row, col) == (robot_row, robot_col):
                line.append(arrow)
            elif (row, col) in path_set:
                line.append('*')
            elif grid.grid[row][col] == 1:
                line.append('#')
            else:
                line.append('.')
        print(f"{row:2d} | {''.join(line)}")
    
    # Print column indicators
    col_labels = ''.join(str(c % 10) for c in range(col_min, col_max))
    print(f"   | {col_labels}")