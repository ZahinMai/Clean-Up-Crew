import math
from heapq import heappush, heappop
from typing import List, Tuple, Optional

''' 
OccupancyGrid class
- represents a 2D grid map w methods for coordinate conversion, occupancy checks, and obstacle inflation.

AStarPlanner class
- implements the A* pathfinding algorithm on an OccupancyGrid.
- uses a priority queue to explore most promising nodes first, based on cost & heuristic estimates.
- path smoothing method removes unnecessary waypoints while ensuring the path remains collision-free.
'''

class OccupancyGrid:
    # 0 = free, 1 = occupied
    def __init__(self, width: int, height: int, cell_size: float = 0.246, 
                 origin: Tuple[float, float] = (0.0, 0.0)):
        self.width = width
        self.height = height
        self.cell_size = cell_size
        self.origin = origin
        self.grid = [[0 for _ in range(width)] for _ in range(height)]
    
    @classmethod
    # Construct map from string representation (in map.py)
    def from_string(cls, map_string: str, cell_size: float = 0.246, 
                    origin: Tuple[float, float] = (0.0, 0.0)) -> 'OccupancyGrid':
        
        # Parse lines, ignoring empty ones
        lines = [line.rstrip() for line in map_string.strip().split('\n') if line.strip()]
        
        # Determine dimensions
        height = len(lines)
        width = max(len(line) for line in lines)
       
        grid = cls(width, height, cell_size, origin)  # Create empty grid
        
        # Fill grid based on characters in string
        for row, line in enumerate(lines):
            for col, char in enumerate(line):
                if char == '#':
                    grid.grid[row][col] = 1
                else:
                    grid.grid[row][col] = 0
        
        return grid
    
    # Coordinate conversion methods
    def world_to_grid(self, x: float, z: float) -> Tuple[int, int]:
        col = math.floor((x - self.origin[0]) / self.cell_size)
        row = math.floor((z - self.origin[1]) / self.cell_size)
        return row, col
    def grid_to_world(self, row: int, col: int) -> Tuple[float, float]:
        x = col * self.cell_size + self.origin[0] + self.cell_size / 2
        z = row * self.cell_size + self.origin[1] + self.cell_size / 2
        return x, z
    
    # Validity and occupancy checks
    def is_valid(self, row: int, col: int) -> bool:
        return 0 <= row < self.height and 0 <= col < self.width
    
    def is_free(self, row: int, col: int) -> bool:
        if not self.is_valid(row, col):
            return False
        return self.grid[row][col] == 0
    
    def set_obstacle(self, row: int, col: int):
        if self.is_valid(row, col):
            self.grid[row][col] = 1

    # this method inflates obstacles by a given radius (in cells) for safety 
    def inflate(self, radius: int) -> 'OccupancyGrid':
        inflated = OccupancyGrid(self.width, self.height, self.cell_size, self.origin)
        inflated.grid = [row[:] for row in self.grid]

        obstacles = []
        for r in range(self.height):
            for c in range(self.width):
                if not self.is_free(r, c):
                    obstacles.append((r, c))

        for r, c in obstacles:
            for dr in range(-radius, radius + 1):
                for dc in range(-radius, radius + 1):
                    nr, nc = r + dr, c + dc
                    if inflated.is_valid(nr, nc):
                        inflated.set_obstacle(nr, nc)
        return inflated
    
    # Simple text-based visualisation (using PIL was an overkill))
    def visualise(self):
        for row in self.grid:
            line = ''.join('#' if cell == 1 else '.' for cell in row)
            print(line)

class AStarPlanner:
    def __init__(self):
        # Possible movements: (dr, dc, cost)
        self.motions = [
            (-1, 0, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (0, 1, 1.0),
            (-1, -1, 1.414), # -> diagonal moves cost = sqrt(2)
            (-1, 1, 1.414),
            (1, -1, 1.414),
            (1, 1, 1.414),
        ]
    
    # Heuristic function: Euclidean distance
    def heuristic(self, pos1: Tuple[int, int], pos2: Tuple[int, int]) -> float:
        return math.hypot(pos2[0] - pos1[0], pos2[1] - pos1[1])
    
    def is_diagonal_blocked(self, grid: OccupancyGrid, current: Tuple[int, int], neighbor: Tuple[int, int]) -> bool:
        dr = neighbor[0] - current[0]
        dc = neighbor[1] - current[1]
        
        if abs(dr) == 1 and abs(dc) == 1:
            adj1 = (current[0] + dr, current[1])
            adj2 = (current[0], current[1] + dc)
            
            # If either adjacent cell is occupied, diagonal move is blocked
            if not (grid.is_free(*adj1) and grid.is_free(*adj2)): return True
        
        return False
    
    # Main A* planning method
    def plan(self, grid: OccupancyGrid, 
             start: Tuple[int, int], 
             goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        
        # Early exit if start or goal blocked
        if not (grid.is_free(*start) and grid.is_free(*goal)): return None

        open_set = [] # priority queue of (f_score, counter, position, path)
        counter = 0 # to ensure unique entries in priority queue
        heappush(open_set, (0, counter, start, [start]))
        
        g_score = {start: 0}
        closed_set = set()
        
        while open_set:
            _, _, current, path = heappop(open_set) # get node w lowest f_score (g + h)
            
            if current in closed_set: continue # skip already evaluated nodes
            closed_set.add(current)            # mark current as evaluated 
            if current == goal: return path    # path found
            
            # Go through neighbors, adding valid ones to open set with calculated scores
            for dr, dc, cost in self.motions:
                neighbor = (current[0] + dr, current[1] + dc)
                
                # Skip invalid, occupied, or diagonally blocked neighbors
                if (neighbor in closed_set or not grid.is_free(*neighbor)
                        or self.is_diagonal_blocked(grid, current, neighbor)):
                    continue
                
                # tentative since we may find a better path later
                tentative_g = g_score[current] + cost
                
                # If this path to neighbor is better, record it
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g 
                    f_score = tentative_g + self.heuristic(neighbor, goal)
                    counter += 1
                    heappush(open_set, (f_score, counter, neighbor, path + [neighbor]))
    
        return None
        
    # Path smoothing method to remove unnecessary waypoints
    def smooth_path(self, grid: OccupancyGrid, 
                    path: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        if len(path) <= 2:
            return path
        
        smoothed = [path[0]] # Add start point
        current_idx = 0 # index of last added point
        
        while current_idx < len(path) - 1:
            for i in range(len(path) - 1, current_idx, -1):
                if self._line_of_sight(grid, path[current_idx], path[i]):
                    smoothed.append(path[i])
                    current_idx = i
                    break
            else:
                current_idx += 1
                if current_idx < len(path):
                    smoothed.append(path[current_idx])
        
        return smoothed
    
    '''
    Bresenham's line algorithm:
    -> Determine which grid cells a straight line between two points passes through and check if they are free.
    
    Step through grid, at each step:
        decide which cell to move to based on acc error between ideal line & actual grid cells.
        if any cell along the line occupied -> no line of sight.
    
    -> This ensures that the smoothed path does not pass through obstacles.
    '''
    def _line_of_sight(self, grid: OccupancyGrid, 
                       p1: Tuple[int, int], 
                       p2: Tuple[int, int]) -> bool:
        r0, c0 = p1
        r1, c1 = p2
        
        dr = abs(r1 - r0)
        dc = abs(c1 - c0)
        sr = 1 if r0 < r1 else -1
        sc = 1 if c0 < c1 else -1
        err = dr - dc
        
        r, c = r0, c0
        
        # Bresenham's line algorithm -> check cells along the line
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