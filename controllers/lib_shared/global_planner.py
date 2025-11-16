# A* Pathfinding Algorithm for 2D Grid Navigation
# Author: Zahin Maisa
"""
Usage:
    from global_planner import AStarPlanner
    from navigation import OccupancyGrid
    
    planner = AStarPlanner(allow_diagonal=True)
    path = planner.find_path(grid, start=(0, 0), goal=(10, 10))
"""

import heapq
import math
from typing import List, Tuple


class AStarPlanner:
    """A* pathfinding algorithm for grid-based navigation."""
    
    class Node:
        """Represents a node in the A* search."""
        
        def __init__(self, position: Tuple[int, int], parent=None):
            self.position = position
            self.parent = parent
            self.g = 0  # Cost from start
            self.h = 0  # Heuristic cost to goal
            self.f = 0  # Total cost (g + h)
        
        def __eq__(self, other):
            return self.position == other.position
        
        def __lt__(self, other):
            return self.f < other.f
        
        def __hash__(self):
            return hash(self.position)
    
    def __init__(self, allow_diagonal: bool = True):
        self.allow_diagonal = allow_diagonal
        
        if allow_diagonal:
            # 8-directional movement
            self.directions = [
                (-1, 0, 1.0),   # Up
                (1, 0, 1.0),    # Down
                (0, -1, 1.0),   # Left
                (0, 1, 1.0),    # Right
                (-1, -1, 1.414), # Up-Left (diagonal)
                (-1, 1, 1.414),  # Up-Right (diagonal)
                (1, -1, 1.414),  # Down-Left (diagonal)
                (1, 1, 1.414),   # Down-Right (diagonal)
            ]
        else:
            # 4-directional movement
            self.directions = [
                (-1, 0, 1.0),   # Up
                (1, 0, 1.0),    # Down
                (0, -1, 1.0),   # Left
                (0, 1, 1.0),    # Right
            ]
    
    #  Calculate heuristic distance (Euclidean or Manhattan).
    def heuristic(self, pos1: Tuple[int, int], pos2: Tuple[int, int]) -> float:
        if self.allow_diagonal:
            # Euclidean distance for diagonal movement
            return math.sqrt((pos2[0] - pos1[0])**2 + (pos2[1] - pos1[1])**2)
        else:
            # Manhattan distance for 4-directional movement
            return abs(pos2[0] - pos1[0]) + abs(pos2[1] - pos1[1])
    
    def is_valid_position(self, grid, position: Tuple[int, int]) -> bool:

        row, col = position
        
        # Handle both OccupancyGrid objects and raw 2D lists
        if hasattr(grid, 'is_free'):
            # OccupancyGrid object
            return grid.is_free(row, col)
        else:
            # Raw 2D list
            rows = len(grid)
            cols = len(grid[0]) if rows > 0 else 0
            
            # Check bounds
            if row < 0 or row >= rows or col < 0 or col >= cols:
                return False
            
            # Check obstacle (0 = free, 1 = occupied)
            return grid[row][col] == 0
    
    def is_diagonal_blocked(self, grid, current_pos: Tuple[int, int], 
                           next_pos: Tuple[int, int]) -> bool:
        curr_row, curr_col = current_pos
        next_row, next_col = next_pos
        
        row_diff = next_row - curr_row
        col_diff = next_col - curr_col
        
        # Only check for diagonal moves
        if abs(row_diff) == 1 and abs(col_diff) == 1:
            # Check the two adjacent cells
            adj1 = (curr_row + row_diff, curr_col)
            adj2 = (curr_row, curr_col + col_diff)
            
            # If either adjacent cell is blocked, diagonal is blocked
            if not self.is_valid_position(grid, adj1) or \
               not self.is_valid_position(grid, adj2):
                return True
        
        return False
    
    def get_neighbors(self, grid, current_pos: Tuple[int, int]) -> List[Tuple[Tuple[int, int], float]]:
        neighbors = []
        curr_row, curr_col = current_pos
        
        for dr, dc, cost in self.directions:
            neighbor_pos = (curr_row + dr, curr_col + dc)
            
            # Check if position is valid
            if not self.is_valid_position(grid, neighbor_pos):
                continue
            
            # For diagonal moves, check if blocked by adjacent obstacles
            if self.allow_diagonal and self.is_diagonal_blocked(grid, current_pos, neighbor_pos):
                continue
            
            neighbors.append((neighbor_pos, cost))
        
        return neighbors
    
    # Reconstruct path from start to goal by following parent pointers.
    def reconstruct_path(self, current_node) -> List[Tuple[int, int]]:
        path = []
        current = current_node
        
        while current is not None:
            path.append(current.position)
            current = current.parent
        
        # Reverse to get path from start to goal
        path.reverse()
        return path
    
    # The main A* pathfinding method
    def find_path(self, grid, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        # Validate input
        if not self.is_valid_position(grid, start):
            print(f"Error: Start position {start} is invalid or blocked")
            return []
        
        if not self.is_valid_position(grid, goal):
            print(f"Error: Goal position {goal} is invalid or blocked")
            return []
        
        # Initialise start and goal nodes
        start_node = self.Node(start)
        start_node.g = 0
        start_node.h = self.heuristic(start, goal)
        start_node.f = start_node.h
        
        # Open list (priority queue) and closed set
        open_list = []
        heapq.heappush(open_list, start_node)
        
        closed_set = set()
        open_dict = {start: start_node}  # For quick lookup
        
        # Statistics
        nodes_explored = 0
        
        # Main A* loop
        while open_list:
            # Get node with lowest f score
            current_node = heapq.heappop(open_list)
            current_pos = current_node.position
            
            # Remove from open dict
            if current_pos in open_dict:
                del open_dict[current_pos]
            
            # Add to closed set
            closed_set.add(current_pos)
            nodes_explored += 1
            
            # Check if we reached the goal
            if current_pos == goal:
                path = self.reconstruct_path(current_node)
                print(f"A* found path: {nodes_explored} nodes explored, {len(path)} steps")
                return path
            
            # Explore neighbors
            for neighbor_pos, move_cost in self.get_neighbors(grid, current_pos):
                # Skip if already in closed set
                if neighbor_pos in closed_set:
                    continue
                
                # Calculate new g score
                new_g = current_node.g + move_cost
                
                # Check if neighbor is in open list
                if neighbor_pos in open_dict:
                    neighbor_node = open_dict[neighbor_pos]
                    # Update if we found a better path
                    if new_g < neighbor_node.g:
                        neighbor_node.g = new_g
                        neighbor_node.f = neighbor_node.g + neighbor_node.h
                        neighbor_node.parent = current_node
                        # Re-heapify
                        heapq.heapify(open_list)
                else:
                    # Create new neighbor node
                    neighbor_node = self.Node(neighbor_pos, current_node)
                    neighbor_node.g = new_g
                    neighbor_node.h = self.heuristic(neighbor_pos, goal)
                    neighbor_node.f = neighbor_node.g + neighbor_node.h
                    
                    heapq.heappush(open_list, neighbor_node)
                    open_dict[neighbor_pos] = neighbor_node
        
        # No path found
        print(f"A* failed: No path found after exploring {nodes_explored} nodes")
        return []


# Example usage and testing
if __name__ == "__main__":
    print("Testing A* Planner")
    print("=" * 60)
    
    # Simple test grid (0 = free, 1 = obstacle)
    test_grid = [
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 1, 1, 1, 1, 1, 1, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
        [0, 1, 1, 1, 1, 1, 1, 0, 1, 0],
        [0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
        [0, 1, 1, 1, 1, 0, 1, 1, 1, 0],
        [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        [0, 1, 1, 0, 1, 1, 1, 1, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    ]
    
    planner = AStarPlanner(allow_diagonal=True)
    
    start = (0, 0)
    goal = (9, 9)
    
    print(f"Finding path from {start} to {goal}...")
    path = planner.find_path(test_grid, start, goal)
    
    if path:
        print(f"\n✓ Path found with {len(path)} steps")
        print(f"Path: {path[:5]}...{path[-5:]}")
        
        # Visualise the path
        print("\nVisualisation (S=Start, G=Goal, #=Obstacle, +=Path, .=Free):")
        for row_idx, row in enumerate(test_grid):
            line = []
            for col_idx, cell in enumerate(row):
                pos = (row_idx, col_idx)
                if pos == start:
                    line.append('S')
                elif pos == goal:
                    line.append('G')
                elif pos in path:
                    line.append('+')
                elif cell == 1:
                    line.append('#')
                else:
                    line.append('.')
            print(''.join(line))
    else:
        print("\n✗ No path found!")
    
    print("=" * 60)