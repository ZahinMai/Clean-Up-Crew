# Implemented by: Abdullateef Vahora

from typing import List, Tuple

class CoveragePlanner:
    """
    Coverage Planner for Spotter Robot
    Decomposes the occupancy grid into distinct rectangular zones, generates a 
    lawnmower path for each zone, and joins them into a complete coverage path.
    """
    def __init__(self, grid_obj, step: int = 16):
        self.grid = grid_obj
        self.step = step
        self.working_grid = [row[:] for row in self.grid.grid] # Copy of the grid (0=Free, 1=Obstacle, 2=Visited)
        self.zones = []
        self.corners = { # Config to help with joining zones
            'TL': {'is_top': True,  'is_right': False},
            'BL': {'is_top': False, 'is_right': False},
            'TR': {'is_top': True,  'is_right': True},
            'BR': {'is_top': False, 'is_right': True}
        }

    def generate_lawnmower_path(self) -> List[Tuple[float, float]]:
        print("Generating Coverage Path.")
        
        self._decompose_grid()
        
        if not self.zones:
            return []

        ordered_zones = self._order_zones_by_adjacency()
        
        # Final path generation
        final_waypoints = []
        
        for i, zone in enumerate(ordered_zones):
            if i == 0:
                # First corner reflects the Spotter bot's starting position
                chosen_corner = 'BL'
            else:
                # For following zones, find the closest corner to previous end point
                previous_point = final_waypoints[-1]
                chosen_corner = self._get_closest_corner(zone, previous_point)
            
            zone_path = self._generate_zone_lawnmower(zone, chosen_corner)
            final_waypoints.extend(zone_path)

        print(f"Mission: {len(final_waypoints)} points.")
        return final_waypoints
    
    def generate_hardcoded_waypoints(self):
        """
        Generates waypoints covering the grid from Top to Bottom with equally spaced waypoints for this specific map.
        Used as a fallback or simple coverage method.
        """
        # Define the grid lines
        x_cols = [3.6, 1.2, -1.2, -3.6] 
        y_rows = [5.1, 3.4, 1.7, 0.0, -1.7, -3.4, -5.1] 
        
        final_waypoints = []
        
        for i, y in enumerate(y_rows):
            if i % 2 == 0:
                row_points = [(x, y) for x in x_cols]
            else:
                row_points = [(x, y) for x in reversed(x_cols)]
                
            final_waypoints.extend(row_points)
            
        return final_waypoints

    def _decompose_grid(self):
        """Iteratively finds the largest valid rectangle of free space starting from top-left."""
        rows = self.grid.height
        cols = self.grid.width
        
        while True:
            # Find the first '0' (free space)
            found = False
            r_start, c_start = -1, -1
            
            for r in range(rows):
                for c in range(cols):
                    if self.working_grid[r][c] == 0:
                        r_start, c_start = r, c
                        found = True
                        break
                if found: break
            
            if not found:
                break # No more free space
            
            # Expand Width (Right)
            c_end = c_start
            while (c_end + 1 < cols) and (self.working_grid[r_start][c_end + 1] == 0):
                c_end += 1
            
            # Expand Height (Down)
            r_end = r_start
            while r_end + 1 < rows:
                next_r = r_end + 1
                is_row_free = True
                
                # For every new row we add, the entire width must be free (prevents L-shaped 'rectangles').
                for k in range(c_start, c_end + 1):
                    if self.working_grid[next_r][k] != 0:
                        is_row_free = False
                        break
                
                if is_row_free:
                    r_end += 1
                else:
                    break # Stop expanding down if we hit an obstacle
            
            # Save the valid rectangle
            zone = {
                'r_min': r_start, 'r_max': r_end,
                'c_min': c_start, 'c_max': c_end,
                'id': len(self.zones)
            }
            self.zones.append(zone)
            
            # Mark area as Visited (2)
            for r in range(r_start, r_end + 1):
                for c in range(c_start, c_end + 1):
                    self.working_grid[r][c] = 2

    def _are_zones_touching(self, z1, z2):
        """Returns True if zones share a border (corners are neighbours)."""
        # Vertical neighbours
        is_vert_touching = (z1['r_max'] + 1 == z2['r_min']) or (z2['r_max'] + 1 == z1['r_min'])
        is_horz_overlap = not (z1['c_max'] < z2['c_min'] or z2['c_max'] < z1['c_min'])
        
        if is_vert_touching and is_horz_overlap: return True

        # Horizontal neighbours
        is_horz_touching = (z1['c_max'] + 1 == z2['c_min']) or (z2['c_max'] + 1 == z1['c_min'])
        is_vert_overlap = not (z1['r_max'] < z2['r_min'] or z2['r_max'] < z1['r_min'])

        if is_horz_touching and is_vert_overlap: return True
        
        return False # Not touching

    def _order_zones_by_adjacency(self):
        """Greedy chain to pick next touching zone."""
        ordered = []
        visited_ids = set()
        
        current_idx = 0
        
        while current_idx is not None:
            ordered.append(self.zones[current_idx])
            visited_ids.add(current_idx)
            
            next_idx = None
            for i, zone in enumerate(self.zones):
                if i in visited_ids: continue
                
                # Prevents joining zones that are separated by obstacles
                if self._are_zones_touching(self.zones[current_idx], zone):
                    next_idx = i
                    break
            
            current_idx = next_idx 
            
        return ordered

    def _get_closest_corner(self, zone, current_bot_pos):
        """Finds the corner of the zone closest to the current bot's position."""
        zone_corners = {
            'TL': (zone['r_min'], zone['c_min']),
            'BL': (zone['r_max'], zone['c_min']),
            'TR': (zone['r_min'], zone['c_max']),
            'BR': (zone['r_max'], zone['c_max'])
        }
        
        best_key = 'TL'
        min_distance = float('inf')
        
        for key, (r, c) in zone_corners.items():
            wx, wy = self.grid.grid_to_world(r, c)

            distance = (wx - current_bot_pos[0])**2 + (wy - current_bot_pos[1])**2
            
            if distance < min_distance:
                min_distance = distance
                best_key = key
                
        return best_key

    def _generate_zone_lawnmower(self, zone, start_key):
        """Generates waypoints for a given zone starting from a specified corner."""
        waypoints = []
        r_min, r_max = zone['r_min'], zone['r_max']
        c_min, c_max = zone['c_min'], zone['c_max']
        step = self.step
        corner = self.corners[start_key]

        # Order columns and rows based on starting corner
        cols = range(c_min, c_max + 1, step)
        if corner['is_right']:
            cols = range(c_max, c_min - 1, -step)

        for i, c in enumerate(cols):
            is_going_down = (i % 2 == 0) if corner['is_top'] else (i % 2 != 0)
            if is_going_down:
                rows = range(r_min, r_max + 1, step)
            else:
                rows = range(r_max, r_min - 1, -step)
                
            for r in rows:
                waypoints.append(self.grid.grid_to_world(r, c))
                
        return waypoints
