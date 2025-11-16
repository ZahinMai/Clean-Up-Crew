"""
Cafeteria Map Definition
Provides pre-defined maps and waypoints for navigation
"""
import os
import sys

# Add lib_shared to path if not already there
THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if THIS_DIR not in sys.path:
    sys.path.insert(0, THIS_DIR)

from .navigation import OccupancyGrid
from typing import Tuple

# Map configuration
CELL_SIZE = 0.5  # Size of each grid cell in meters
ORIGIN = (0.0, 0.0)  # World coordinates of grid origin (bottom-left)

# 30x30 Cafeteria Map (15m x 15m)
CAFETERIA_MAP = """
##############################
#                            #
#                            #
#                            #
#                            #
# ########################   #
# ########################   #
# ########################   #
# #                    ###   #
# #   ####   ####      ###   #
# #   ####   ####      ###   #
# #   ####   ####            #
# #   ####   ####            #
# #                          #
# #                          #
# #   ####   ####            #
# #   ####   ####            #
# #   ####   ####            #
# #   ####   ####            #
# #                          #
# #                          #
# ################           # 
#                            #
#                            #
#     ####   ####            #
#     ####   ####            #
#     ####   ####            #
#     ####   ####            #
#                            #
##############################
"""

# Smaller test map for quick testing
TEST_MAP = """
####################
#                  #
#   ####    ####   #
#   ####    ####   #
#                  #
#   ####    ####   #
#   ####    ####   #
#                  #
####################
"""


def get_map(cell_size: float = CELL_SIZE, 
            origin: Tuple[float, float] = ORIGIN, 
            use_test_map: bool = False) -> OccupancyGrid:
    """
    Get the cafeteria occupancy grid map.
    
    Args:
        cell_size: Size of each grid cell in meters (default: 0.5)
        origin: World coordinates of grid origin (default: (0.0, 0.0))
        use_test_map: If True, use smaller test map
        
    Returns:
        OccupancyGrid instance with the map loaded
    """
    map_string = TEST_MAP if use_test_map else CAFETERIA_MAP
    
    grid = OccupancyGrid.from_string(
        map_string=map_string,
        cell_size=cell_size,
        origin=origin
    )
    
    print(f"Map loaded: {grid.height}×{grid.width} cells, {cell_size}m/cell")
    print(f"World size: {grid.height * cell_size}m × {grid.width * cell_size}m")
    
    return grid


# Predefined waypoints (grid coordinates)
WAYPOINTS = {
    'entrance': (1, 15),
    'dining_area': (15, 15),
    'kitchen': (5, 25),
    'exit': (28, 28),
}


def get_waypoint(name: str) -> Tuple[int, int]:
    """
    Get predefined waypoint by name.
    
    Args:
        name: Waypoint name (e.g., 'entrance', 'dining_area')
        
    Returns:
        Grid coordinates (row, col)
    """
    if name not in WAYPOINTS:
        raise KeyError(f"Unknown waypoint: {name}")
    return WAYPOINTS[name]


# Example usage
if __name__ == "__main__":
    print("=" * 60)
    print("Map Module Test")
    print("=" * 60)
    
    grid = get_map()
    
    print("\nMap visualization:")
    grid.visualize()
    
    print("\nAvailable waypoints:")
    for name, coords in WAYPOINTS.items():
        world = grid.grid_to_world(coords[0], coords[1])
        print(f"  {name:15} Grid {coords} → World ({world[0]:.2f}, {world[1]:.2f})")