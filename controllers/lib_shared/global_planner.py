# ============================================= #
#  A* PATH PLANNING        -> AUTHOR: ZAHIN     #
# ============================================= #
import math
from heapq import heappush, heappop
from typing import List, Tuple, Optional
from .map_module import OccupancyGrid

class AStarPlanner:
    def __init__(self):
        diag_cost = 1.414  # Precomputed sqrt(2)
        # (delta_row, delta_col, move_cost)
        self.moves = [
            (-1, 0, 1), (1, 0, 1), (0, -1, 1), (0, 1, 1),
            (-1, -1, diag_cost), (-1, 1, diag_cost),
            (1, -1, diag_cost), (1, 1, diag_cost)
        ]
        # Precompute adjacent cells for diagonal corner checks
        self.diag_adjacent = {
            (-1, -1): [(-1, 0), (0, -1)],
            (-1, 1): [(-1, 0), (0, 1)],
            (1, -1): [(1, 0), (0, -1)],
            (1, 1): [(1, 0), (0, 1)]
        }

    def plan(self, grid: OccupancyGrid, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        """Return grid cell path from start to goal or None if unreachable."""
        if not (grid.is_free(*start) and grid.is_free(*goal)):
            return None

        # Use closed set to avoid revisiting nodes
        closed = set()
        queue = [(0.0, start)]
        g_scores = {start: 0.0}
        came_from = {}

        # Precompute goal for heuristic
        gr, gc = goal

        while queue:
            _, curr = heappop(queue)

            if curr in closed:
                continue
            
            closed.add(curr)

            if curr == goal:
                return self._reconstruct_path(came_from, curr)

            curr_g = g_scores[curr]
            cr, cc = curr

            for dr, dc, step_cost in self.moves:
                nr, nc = cr + dr, cc + dc
                nxt = (nr, nc)

                if nxt in closed or not grid.is_free(nr, nc):
                    continue

                # Corner cutting check for diagonals
                if abs(dr) == 1 and abs(dc) == 1:
                    adj_cells = self.diag_adjacent[(dr, dc)]
                    if not all(grid.is_free(cr + ar, cc + ac) for ar, ac in adj_cells):
                        continue

                new_g = curr_g + step_cost

                if new_g < g_scores.get(nxt, float('inf')):
                    came_from[nxt] = curr
                    g_scores[nxt] = new_g
                    h = math.sqrt((gr - nr) ** 2 + (gc - nc) ** 2)
                    heappush(queue, (new_g + h, nxt))

        return None

    def _reconstruct_path(self, came_from: dict, current: Tuple[int, int]) -> List[Tuple[int, int]]:
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def smooth_path(self, grid: OccupancyGrid, path: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        if len(path) <= 2:
            return path

        smoothed = [path[0]]
        i = 0
        path_len = len(path)

        while i < path_len - 1:
            # Binary search for furthest visible waypoint
            left, right = i + 1, path_len - 1
            best = i + 1

            while left <= right:
                mid = (left + right) // 2
                if self._line_of_sight(grid, path[i], path[mid]):
                    best = mid
                    left = mid + 1
                else:
                    right = mid - 1

            smoothed.append(path[best])
            i = best

        return smoothed

    def _line_of_sight(self, grid: OccupancyGrid, p1: Tuple[int, int], p2: Tuple[int, int]) -> bool:
        """Optimised Bresenham's Line Algorithm."""
        r0, c0 = p1
        r1, c1 = p2

        dr = abs(r1 - r0)
        dc = abs(c1 - c0)
        sr = 1 if r0 < r1 else -1
        sc = 1 if c0 < c1 else -1

        if dr >= dc:
            err = dr >> 1  # Bit shift instead of division
            r, c = r0, c0
            while True:
                if not grid.is_free(r, c):
                    return False
                if r == r1 and c == c1:
                    return True
                r += sr
                err -= dc
                if err < 0:
                    c += sc
                    err += dr
        else:
            err = dc >> 1
            r, c = r0, c0
            while True:
                if not grid.is_free(r, c):
                    return False
                if r == r1 and c == c1:
                    return True
                c += sc
                err -= dr
                if err < 0:
                    r += sr
                    err += dc