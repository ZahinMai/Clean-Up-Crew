import math
from typing import Tuple, List, Optional


def visualise_robot_on_map(grid, robot_x: float, robot_z: float, robot_theta: float,
                           goal_x: Optional[float] = None, goal_z: Optional[float] = None,
                           path: Optional[List[Tuple[int, int]]] = None):
    robot_grid = grid.world_to_grid(robot_x, robot_z)
    
    # Convert goal if provided
    goal_grid = None
    if goal_x is not None and goal_z is not None:
        goal_grid = grid.world_to_grid(goal_x, goal_z)
    
    # Create visualisation
    print("\n" + "=" * 70)
    print("MAP VISUALIsatION")
    print("=" * 70)
    print(f"Robot: world ({robot_x:.2f}, {robot_z:.2f}) = grid {robot_grid}")
    print(f"       orientation: {math.degrees(robot_theta):.0f}°")
    
    if goal_grid:
        print(f"Goal:  world ({goal_x:.2f}, {goal_z:.2f}) = grid {goal_grid}")
    
    print(f"Map:   {grid.height}x{grid.width} cells, {grid.cell_size}m/cell")
    print(f"       origin at {grid.origin}")
    print()
    
    # Build path set for quick lookup
    path_set = set(path) if path else set()
    
    # Print map with annotations
    for row in range(grid.height):
        line = []
        for col in range(grid.width):
            pos = (row, col)
            
            # Check what to display
            if pos == robot_grid:
                # Show robot with direction indicator
                if -math.pi/8 < robot_theta <= math.pi/8:
                    line.append('→')  # East
                elif math.pi/8 < robot_theta <= 3*math.pi/8:
                    line.append('↗')  # Northeast
                elif 3*math.pi/8 < robot_theta <= 5*math.pi/8:
                    line.append('↑')  # North
                elif 5*math.pi/8 < robot_theta <= 7*math.pi/8:
                    line.append('↖')  # Northwest
                elif robot_theta > 7*math.pi/8 or robot_theta <= -7*math.pi/8:
                    line.append('←')  # West
                elif -7*math.pi/8 < robot_theta <= -5*math.pi/8:
                    line.append('↙')  # Southwest
                elif -5*math.pi/8 < robot_theta <= -3*math.pi/8:
                    line.append('↓')  # South
                else:  # -3*math.pi/8 < robot_theta <= -math.pi/8
                    line.append('↘')  # Southeast
            elif goal_grid and pos == goal_grid:
                line.append('G')  # Goal
            elif pos in path_set:
                line.append('+')  # Path
            elif grid.grid[row][col] == 1:
                line.append('#')  # Obstacle
            else:
                line.append('.')  # Free space
        
        # Add row numbers for reference
        print(f"{row:2d} {''.join(line)}")
    
    # Print column numbers
    print("   " + "".join(str(i % 10) for i in range(grid.width)))
    
    print("\nLegend: →=Robot, G=Goal, +=Path, #=Obstacle, .=Free")
    print("=" * 70 + "\n")


def print_coordinate_info(grid, robot_x: float, robot_z: float):
    """
    Print detailed coordinate transformation info
    """
    robot_grid = grid.world_to_grid(robot_x, robot_z)
    back_to_world = grid.grid_to_world(robot_grid[0], robot_grid[1])
    
    print("\n" + "=" * 70)
    print("COORDINATE SYSTEM DEBUG")
    print("=" * 70)
    print(f"Grid configuration:")
    print(f"  Size: {grid.height} rows × {grid.width} cols")
    print(f"  Cell size: {grid.cell_size}m")
    print(f"  Origin: {grid.origin}")
    print(f"  World bounds: X=[{grid.origin[0]}, {grid.origin[0] + grid.width * grid.cell_size}]")
    print(f"               Z=[{grid.origin[1]}, {grid.origin[1] + grid.height * grid.cell_size}]")
    print()
    print(f"Robot position:")
    print(f"  World: ({robot_x:.3f}, {robot_z:.3f})")
    print(f"  Grid:  {robot_grid}")
    print(f"  Back to world: ({back_to_world[0]:.3f}, {back_to_world[1]:.3f})")
    print()
    
    # Check if in bounds
    in_bounds = grid.is_valid(robot_grid[0], robot_grid[1])
    is_free = grid.is_free(robot_grid[0], robot_grid[1]) if in_bounds else False
    
    print(f"Validation:")
    print(f"  In grid bounds? {in_bounds}")
    if in_bounds:
        print(f"  Cell free? {is_free}")
        print(f"  Cell value: {grid.grid[robot_grid[0]][robot_grid[1]]}")
    else:
        print(f"  ⚠ WARNING: Robot is outside map bounds!")
    
    print("=" * 70 + "\n")


def check_map_alignment(grid):
    print("\n" + "=" * 70)
    print("MAP CONFIGURATION CHECK")
    print("=" * 70)
    
    # Count obstacles and free cells
    total_cells = grid.height * grid.width
    obstacle_cells = sum(sum(1 for cell in row if cell == 1) for row in grid.grid)
    free_cells = total_cells - obstacle_cells
    
    print(f"Map statistics:")
    print(f"  Total cells: {total_cells}")
    print(f"  Obstacle cells: {obstacle_cells} ({100*obstacle_cells/total_cells:.1f}%)")
    print(f"  Free cells: {free_cells} ({100*free_cells/total_cells:.1f}%)")
    print()
    
    # Check corners
    corners = [
        ((0, 0), "Top-left"),
        ((0, grid.width-1), "Top-right"),
        ((grid.height-1, 0), "Bottom-left"),
        ((grid.height-1, grid.width-1), "Bottom-right"),
    ]
    
    print("Corner cells:")
    for (r, c), name in corners:
        world = grid.grid_to_world(r, c)
        status = "FREE" if grid.is_free(r, c) else "OCCUPIED"
        print(f"  {name:15} grid ({r:2d},{c:2d}) = world ({world[0]:6.2f},{world[1]:6.2f}) [{status}]")
    
    print()
    
    # Suggest good test positions
    print("Suggested test positions (in free space):")
    test_positions = []
    for r in range(1, grid.height-1):
        for c in range(1, grid.width-1):
            if grid.is_free(r, c):
                # Check if surrounded by free space (good starting position)
                if all(grid.is_free(r+dr, c+dc) 
                      for dr in [-1, 0, 1] for dc in [-1, 0, 1]
                      if grid.is_valid(r+dr, c+dc)):
                    world = grid.grid_to_world(r, c)
                    test_positions.append((r, c, world[0], world[1]))
                    if len(test_positions) >= 5:
                        break
        if len(test_positions) >= 5:
            break
    
    for i, (r, c, wx, wz) in enumerate(test_positions, 1):
        print(f"  {i}. grid ({r:2d},{c:2d}) = world ({wx:6.2f},{wz:6.2f})")
    
    print("=" * 70 + "\n")


# Quick test function
def test_visualisation():
    from map import get_map
    
    grid = get_map()
    
    # Check map configuration
    check_map_alignment(grid)
    
    # Test with sample robot position
    robot_x, robot_z = 0.8, 0.2
    robot_theta = 0.5
    goal_x, goal_z = 5.0, 5.0
    

    print_coordinate_info(grid, robot_x, robot_z)
    visualise_robot_on_map(grid, robot_x, robot_z, robot_theta, goal_x, goal_z)


if __name__ == "__main__":
    test_visualisation()