"""
Unified Navigation Library for TurtleBot3
Combines occupancy grid, A* planning, and DWA local control
"""

import math
from heapq import heappush, heappop
from typing import List, Tuple, Optional, Dict


# ============================================================================
# OCCUPANCY GRID
# ============================================================================

class OccupancyGrid:
    """Occupancy grid map for navigation"""
    
    def __init__(self, width: int, height: int, cell_size: float = 0.5, 
                 origin: Tuple[float, float] = (0.0, 0.0)):
        """
        Args:
            width: Grid width in cells
            height: Grid height in cells
            cell_size: Size of each cell in meters
            origin: World coordinates of grid origin (bottom-left)
        """
        self.width = width
        self.height = height
        self.cell_size = cell_size
        self.origin = origin
        self.grid = [[0 for _ in range(width)] for _ in range(height)]
    
    @classmethod
    def from_string(cls, map_string: str, cell_size: float = 0.5, 
                    origin: Tuple[float, float] = (0.0, 0.0)) -> 'OccupancyGrid':
        """
        Create occupancy grid from ASCII string
        # = obstacle, space = free, T/G = markers (free)
        """
        lines = [line.rstrip() for line in map_string.strip().split('\n') if line.strip()]
        height = len(lines)
        width = max(len(line) for line in lines)
        
        grid = cls(width, height, cell_size, origin)
        
        for row, line in enumerate(lines):
            for col, char in enumerate(line):
                if char == '#':
                    grid.grid[row][col] = 1  # Obstacle
                else:
                    grid.grid[row][col] = 0  # Free space
        
        return grid
    
    def world_to_grid(self, x: float, z: float) -> Tuple[int, int]:
        """Convert world coordinates to grid indices (row, col)"""
        col = int((x - self.origin[0]) / self.cell_size)
        row = int((z - self.origin[1]) / self.cell_size)
        return row, col
    
    def grid_to_world(self, row: int, col: int) -> Tuple[float, float]:
        """Convert grid indices to world coordinates (x, z) - cell center"""
        x = col * self.cell_size + self.origin[0] + self.cell_size / 2
        z = row * self.cell_size + self.origin[1] + self.cell_size / 2
        return x, z
    
    def is_valid(self, row: int, col: int) -> bool:
        """Check if grid cell is within bounds"""
        return 0 <= row < self.height and 0 <= col < self.width
    
    def is_free(self, row: int, col: int) -> bool:
        """Check if cell is free (not obstacle)"""
        if not self.is_valid(row, col):
            return False
        return self.grid[row][col] == 0
    
    def set_obstacle(self, row: int, col: int):
        """Mark cell as occupied"""
        if self.is_valid(row, col):
            self.grid[row][col] = 1
    
    def visualize(self):
        """Print ASCII visualization of the map"""
        for row in self.grid:
            line = ''.join('#' if cell == 1 else '.' for cell in row)
            print(line)


# ============================================================================
# A* GLOBAL PLANNER
# ============================================================================

class AStarPlanner:
    """A* path planning algorithm for grid-based navigation"""
    
    def __init__(self, allow_diagonal: bool = True):
        self.allow_diagonal = allow_diagonal
        
        if allow_diagonal:
            # 8-directional movement
            self.motions = [
                (-1, 0, 1.0),    # Up
                (1, 0, 1.0),     # Down
                (0, -1, 1.0),    # Left
                (0, 1, 1.0),     # Right
                (-1, -1, 1.414), # Diagonals
                (-1, 1, 1.414),
                (1, -1, 1.414),
                (1, 1, 1.414),
            ]
        else:
            # 4-directional movement
            self.motions = [
                (-1, 0, 1.0),
                (1, 0, 1.0),
                (0, -1, 1.0),
                (0, 1, 1.0),
            ]
    
    def heuristic(self, pos1: Tuple[int, int], pos2: Tuple[int, int]) -> float:
        """Euclidean distance heuristic"""
        return math.hypot(pos2[0] - pos1[0], pos2[1] - pos1[1])
    
    def is_diagonal_blocked(self, grid: OccupancyGrid, 
                           current: Tuple[int, int], 
                           neighbor: Tuple[int, int]) -> bool:
        """Check if diagonal movement is blocked by adjacent obstacles"""
        dr = neighbor[0] - current[0]
        dc = neighbor[1] - current[1]
        
        # Only for diagonal moves
        if abs(dr) == 1 and abs(dc) == 1:
            adj1 = (current[0] + dr, current[1])
            adj2 = (current[0], current[1] + dc)
            
            if not grid.is_free(*adj1) or not grid.is_free(*adj2):
                return True
        
        return False
    
    def plan(self, grid: OccupancyGrid, 
             start: Tuple[int, int], 
             goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        """
        Plan path from start to goal using A*
        Args:
            grid: OccupancyGrid object
            start: (row, col) start position
            goal: (row, col) goal position
        Returns:
            List of (row, col) waypoints, or None if no path found
        """
        # Validate inputs
        if not grid.is_free(*start):
            print(f"ERROR: Start {start} is not free")
            return None
        
        if not grid.is_free(*goal):
            print(f"ERROR: Goal {goal} is not free")
            return None
        
        # Priority queue: (f_score, counter, current, path)
        open_set = []
        counter = 0
        heappush(open_set, (0, counter, start, [start]))
        
        # Track best cost to reach each node
        g_score = {start: 0}
        closed_set = set()
        
        nodes_explored = 0
        
        while open_set:
            _, _, current, path = heappop(open_set)
            
            if current in closed_set:
                continue
            
            closed_set.add(current)
            nodes_explored += 1
            
            # Goal reached
            if current == goal:
                print(f"A* found path: {nodes_explored} nodes explored, {len(path)} steps")
                return path
            
            # Explore neighbors
            for dr, dc, cost in self.motions:
                neighbor = (current[0] + dr, current[1] + dc)
                
                if neighbor in closed_set:
                    continue
                
                if not grid.is_free(*neighbor):
                    continue
                
                # Check diagonal blocking
                if self.allow_diagonal and self.is_diagonal_blocked(grid, current, neighbor):
                    continue
                
                tentative_g = g_score[current] + cost
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self.heuristic(neighbor, goal)
                    counter += 1
                    heappush(open_set, (f_score, counter, neighbor, path + [neighbor]))
        
        print(f"A* failed: No path found after exploring {nodes_explored} nodes")
        return None
    
    def smooth_path(self, grid: OccupancyGrid, 
                    path: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        """
        Smooth path by removing unnecessary waypoints using line-of-sight
        """
        if len(path) <= 2:
            return path
        
        smoothed = [path[0]]
        current_idx = 0
        
        while current_idx < len(path) - 1:
            # Try to connect to farthest visible point
            for i in range(len(path) - 1, current_idx, -1):
                if self._line_of_sight(grid, path[current_idx], path[i]):
                    smoothed.append(path[i])
                    current_idx = i
                    break
            else:
                # No line of sight, move to next point
                current_idx += 1
                if current_idx < len(path):
                    smoothed.append(path[current_idx])
        
        return smoothed
    
    def _line_of_sight(self, grid: OccupancyGrid, 
                       p1: Tuple[int, int], 
                       p2: Tuple[int, int]) -> bool:
        """Check if there's a clear line of sight between two cells (Bresenham)"""
        r0, c0 = p1
        r1, c1 = p2
        
        dr = abs(r1 - r0)
        dc = abs(c1 - c0)
        sr = 1 if r0 < r1 else -1
        sc = 1 if c0 < c1 else -1
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


# ============================================================================
# DWA LOCAL PLANNER
# ============================================================================

def _clamp(x, a, b): 
    return a if x < a else b if x > b else x

def _normalize_angle(a: float) -> float:
    """Wrap angle to (-pi, pi]"""
    while a > math.pi: a -= 2*math.pi
    while a <= -math.pi: a += 2*math.pi
    return a


class DWAPlanner:
    """Dynamic Window Approach for local planning"""
    
    def __init__(self, params: Optional[Dict] = None):
        # Default parameters
        self.params = {
            'DT_CTRL': 0.08,      # Control timestep [s]
            'T_PRED': 2.0,        # Prediction horizon [s]
            'DT_SIM': 0.08,       # Simulation timestep [s]
            'NV': 10,             # Linear velocity samples
            'NW': 16,             # Angular velocity samples
            'V_MAX': 0.25,        # Max linear velocity [m/s]
            'W_MAX': 2.0,         # Max angular velocity [rad/s]
            'A_V': 1.0,           # Linear acceleration [m/s^2]
            'A_W': 2.5,           # Angular acceleration [rad/s^2]
            'RADIUS': 0.18,       # Robot radius [m]
            'SAFE': 0.10,         # Safety margin [m]
            'CLEAR_N': 1.00,      # Clearance normalization [m]
            'ALPHA': 0.30,        # Heading weight
            'BETA': 0.35,         # Progress weight
            'GAMMA': 0.50,        # Clearance weight
            'DELTA': 0.10,        # Speed weight
            'EPS': 0.10,          # Smoothness weight
            'D_NORM': 2.0         # Distance normalization [m]
        }
        
        if params:
            self.params.update(params)
        
        self.p = self.params
    
    def rollout(self, v: float, w: float) -> List[Tuple[float, float, float]]:
        """Simulate trajectory from origin with given velocities"""
        x = y = th = t = 0.0
        traj = []
        
        while t < self.p['T_PRED']:
            x += v * math.cos(th) * self.p['DT_SIM']
            y += v * math.sin(th) * self.p['DT_SIM']
            th = _normalize_angle(th + w * self.p['DT_SIM'])
            traj.append((x, y, th))
            t += self.p['DT_SIM']
        
        return traj
    
    def min_clearance(self, trajectory: List[Tuple[float, float]], 
                     obstacles: List[Tuple[float, float]]) -> float:
        """Calculate minimum distance from trajectory to obstacles"""
        if not obstacles:
            return float('inf')
        
        min_dist = float('inf')
        for tx, ty in trajectory:
            for ox, oy in obstacles:
                dist = math.hypot(ox - tx, oy - ty)
                min_dist = min(min_dist, dist)
        
        return min_dist
    
    def compute_velocity(self, goal_x: float, goal_y: float,
                        obstacles: List[Tuple[float, float]],
                        current_v: float = 0.0,
                        current_w: float = 0.0,
                        prev_v: float = 0.0,
                        prev_w: float = 0.0) -> Tuple[float, float]:
        """
        Compute optimal velocity command using DWA
        
        Args:
            goal_x, goal_y: Goal position in robot frame [m]
            obstacles: List of (x, y) obstacle points in robot frame [m]
            current_v, current_w: Current velocities for dynamic window
            prev_v, prev_w: Previous command for smoothness
        
        Returns:
            (v, w): Linear and angular velocities [m/s, rad/s]
        """
        p = self.p
        
        # Dynamic window bounds
        v_min = _clamp(current_v - p['A_V'] * p['DT_CTRL'], -p['V_MAX'], p['V_MAX'])
        v_max = _clamp(current_v + p['A_V'] * p['DT_CTRL'], -p['V_MAX'], p['V_MAX'])
        w_min = _clamp(current_w - p['A_W'] * p['DT_CTRL'], -p['W_MAX'], p['W_MAX'])
        w_max = _clamp(current_w + p['A_W'] * p['DT_CTRL'], -p['W_MAX'], p['W_MAX'])
        
        # Sample velocities
        samples = [(prev_v, prev_w), (0.0, 0.0)]  # Bias towards previous and stop
        
        for i in range(max(1, p['NV'])):
            v = v_min + (v_max - v_min) * (i / (p['NV'] - 1) if p['NV'] > 1 else 0)
            for j in range(max(1, p['NW'])):
                w = w_min + (w_max - w_min) * (j / (p['NW'] - 1) if p['NW'] > 1 else 0)
                samples.append((v, w))
        
        # Goal angle
        goal_angle = math.atan2(goal_y, goal_x) if math.hypot(goal_x, goal_y) > 1e-9 else 0.0
        
        best_v, best_w = 0.0, 0.0
        best_score = -1e9
        
        for v, w in samples:
            traj = self.rollout(v, w)
            points = [(x, y) for x, y, _ in traj]
            
            # Calculate clearance with robot inflation
            min_dist = self.min_clearance(points, obstacles)
            effective_clearance = max(0.0, min_dist - (p['RADIUS'] + 0.02))
            
            # Safety check
            if effective_clearance < p['SAFE']:
                continue
            
            # End state
            x_end, y_end, th_end = traj[-1]
            
            # Scoring components
            heading = 1.0 - abs(_normalize_angle(th_end - goal_angle)) / math.pi
            progress = 1.0 - min(math.hypot(goal_x - x_end, goal_y - y_end) / p['D_NORM'], 1.0)
            speed = max(0.0, v) / (p['V_MAX'] if p['V_MAX'] > 0 else 1.0)
            clearance = min(effective_clearance / p['CLEAR_N'], 1.0)
            smoothness = math.exp(-math.hypot(v - prev_v, w - prev_w) / 0.2)
            
            score = (p['ALPHA'] * heading + 
                    p['BETA'] * progress + 
                    p['GAMMA'] * clearance + 
                    p['DELTA'] * speed + 
                    p['EPS'] * smoothness)
            
            if score > best_score:
                best_score = score
                best_v, best_w = v, w
        
        # Fallback if all rejected
        if best_score < -1e8:
            if abs(goal_angle) > math.pi / 6:
                return (0.0, 0.3 * math.copysign(1.0, goal_angle))
            return (0.05, 0.0)
        
        return best_v, best_w


# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================

def world_to_robot_frame(dx: float, dz: float, yaw: float) -> Tuple[float, float]:
    """Transform world-frame offset to robot frame"""
    gx = dx * math.cos(-yaw) - dz * math.sin(-yaw)
    gy = dx * math.sin(-yaw) + dz * math.cos(-yaw)
    return gx, gy