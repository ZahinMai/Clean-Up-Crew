import os
import sys
from typing import Tuple

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if THIS_DIR not in sys.path:
    sys.path.insert(0, THIS_DIR)

from .global_planner import OccupancyGrid

CELL_SIZE = 0.5
ORIGIN = (-13.5, -3.5)

CAFETERIA_MAP = """
####################################################
#                            ##################### #
#                            ##################### #
#                            ##################### #
#                                                  #
#          ############                            #
#          ############                            #
#          ############                            #
#          ###                                     #
#          ###                                     #
#          ###                                     #
#          ###                                     #
#          ###                     ##              #
#          ###   ######   ######   ##   ######     #
#          ###   ######   ######   ##   ######     #
#          ###   ######   ######   ##   ######     #
#          ###   ######   ######   ##   ######     #
#          ###                     ##              #
#          ###                     ##              #
#          ###                     ##              #
#          ###   ######   ######   ##   ######     #
#          ###   ######   ######   ##   ######     #
#          ###   ######   ######   ##   ######     #
#          ###   ######   ######   ##   ######     #
#          ###                     ##              #
#          ###                     ##              #
#          ###                     ##              #
#          ###                     ##              #
#          ##########################              #
####################################################
"""

def get_map(cell_size: float = CELL_SIZE, 
            origin: Tuple[float, float] = ORIGIN, 
            use_test_map: bool = False) -> OccupancyGrid:
    map_string = CAFETERIA_MAP
    
    grid = OccupancyGrid.from_string(
        map_string=map_string,
        cell_size=cell_size,
        origin=origin
    )
    
    return grid

WAYPOINTS = {
    'entrance': (1, 15),
    'dining_area': (15, 15),
    'kitchen': (5, 25),
    'exit': (28, 28),
}

def get_waypoint(name: str) -> Tuple[int, int]:
    if name not in WAYPOINTS:
        raise KeyError(f"Unknown waypoint: {name}")
    return WAYPOINTS[name]

if __name__ == "__main__":
    grid = get_map()
    grid.visualize()