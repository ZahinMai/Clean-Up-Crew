# controllers/lib_shared/local_planner.py
from __future__ import annotations
from typing import List, Tuple, Dict
import math

class DWA:
    def __init__(self, config: Dict = None):
        # 1. Defaults
        defaults = {
            'DT_SIM': 0.10,   
            'T_PRED': 1.5,    
            
            # Robot Limits
            'V_MAX': 0.22,    
            'W_MAX': 1.5,     
            'ACC_V': 1.0,     
            'ACC_W': 2.0,     
            
            # Safety
            'RADIUS': 0.18,   
            'SAFE': 0.05,     
            
            # SCORING PARAMETERS
            'CLEAR_MAX': 0.5, # Cap clearance score at 0.5m
            
            # Sampling
            'NV': 5,          
            'NW': 11,         
            'LIDAR_SKIP': 5,  
            
            # Weights
            'W_HEAD': 1.0,    # Increased to be dominant
            'W_CLEAR': 0.1,   
            'W_VEL': 0.2      
        }

        if config:
            for key, value in config.items():
                if key in defaults:
                    defaults[key] = value
                else:
                    print(f"[DWA] WARNING: Unknown param '{key}' ignored.")
        
        for key, value in defaults.items():
            setattr(self, key, value)

    def _ang(self, a: float) -> float:
        return (a + math.pi) % (2 * math.pi) - math.pi

    def rollout(self, v: float, w: float) -> List[Tuple[float, float]]:
        traj = []
        x, y, th = 0.0, 0.0, 0.0
        t = 0.0
        
        # Optimization: fast trig for curves, simple math for lines
        if abs(w) < 1e-4:
            dx = v * self.DT_SIM
            while t < self.T_PRED:
                x += dx * math.cos(th)
                y += dx * math.sin(th)
                traj.append((x, y))
                t += self.DT_SIM
        else:
            while t < self.T_PRED:
                x += v * math.cos(th) * self.DT_SIM
                y += v * math.sin(th) * self.DT_SIM
                th += w * self.DT_SIM
                traj.append((x, y))
                t += self.DT_SIM
        return traj

    def get_safe_velocities(
        self,
        lidar_points: List[Tuple[float, float]],
        goal_vec: Tuple[float, float],
        prev_cmd: Tuple[float, float] = (0.0, 0.0),
        cur: Tuple[float, float] = (0.0, 0.0)
    ) -> Tuple[float, float]:
        
        dt = 0.1 
        v_min = max(0.0, cur[0] - self.ACC_V * dt)
        v_max = min(self.V_MAX, cur[0] + self.ACC_V * dt)
        w_min = max(-self.W_MAX, cur[1] - self.ACC_W * dt)
        w_max = min(self.W_MAX, cur[1] + self.ACC_W * dt)

        obstacles = lidar_points[::int(self.LIDAR_SKIP)]

        best_v, best_w = 0.0, 0.0
        max_score = -1.0
        
        gx, gy = goal_vec
        
        nv, nw = int(self.NV), int(self.NW)
        v_steps = [v_min + (v_max - v_min) * i / (nv - 1) for i in range(nv)] if nv > 1 else [v_max]
        w_steps = [w_min + (w_max - w_min) * j / (nw - 1) for j in range(nw)] if nw > 1 else [0.0]

        samples = [(v, w) for v in v_steps for w in w_steps]
        samples.append((0.0, 0.0)) # Ensure stop is always an option

        for v, w in samples:
            traj = self.rollout(v, w)
            
            # --- CLEARANCE CHECK ---
            min_dist_sq = float('inf')
            for tx, ty in traj:
                for ox, oy in obstacles:
                    d2 = (ox - tx)**2 + (oy - ty)**2
                    if d2 < min_dist_sq:
                        min_dist_sq = d2
            
            min_dist = math.sqrt(min_dist_sq) if min_dist_sq != float('inf') else 99.0
            
            if min_dist < (self.RADIUS + self.SAFE):
                continue

            # --- SCORING ---
            
            # 1. Heading Score (0.0 to 1.0)
            # Calculate where the robot ends up and faces
            robot_head_change = w * self.T_PRED
            dx = v * math.cos(w * self.T_PRED/2) * self.T_PRED
            dy = v * math.sin(w * self.T_PRED/2) * self.T_PRED
            
            # Vector to goal from the NEW position
            gx_new = gx - dx
            gy_new = gy - dy
            goal_ang = math.atan2(gy_new, gx_new)
            
            # How far off is the heading?
            error_ang = abs(self._ang(goal_ang - robot_head_change))
            score_head = 1.0 - (error_ang / math.pi)

            # 2. Velocity Score (THE FIX IS HERE)
            # Only reward speed if score_head is high (>0.7).
            # If we are facing the wrong way, speed score drops to near zero.
            raw_vel_score = v / self.V_MAX
            
            if score_head < 0.5:
                # If facing >90 deg away, penalize movement
                score_vel = 0.0 
            else:
                # Scale speed reward by how aligned we are
                score_vel = raw_vel_score * score_head

            # 3. Clearance Score
            score_clear = min(1.0, min_dist / self.CLEAR_MAX)

            total_score = (self.W_HEAD * score_head) + \
                          (self.W_VEL  * score_vel) + \
                          (self.W_CLEAR * score_clear)

            if total_score > max_score:
                max_score = total_score
                best_v, best_w = v, w

        if max_score < 0:
            # Recovery: Rotate in place
            return 0.0, 0.5 

        return best_v, best_w