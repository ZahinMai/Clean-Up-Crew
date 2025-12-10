# ============================================== #
#  SIMPLE OBSTACLE AVOIDANCE -> AUTHOR: ZAHIN    #
# ============================================== #
# Simple reactive obstacle avoidance that dodges #
# oncoming obstacles while following A* path.    #
# ============================================== #

import math
from typing import List, Tuple

def _wrap(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while angle > math.pi: angle -= 2 * math.pi
    while angle < -math.pi: angle += 2 * math.pi
    return angle


class SimpleAvoidance:
    """Simple avoidance: check front, dodge if needed, otherwise go to goal."""
    def __init__(self):
        self.DANGER_DIST = 0.25      # Distance to start avoiding (m)
        self.CRITICAL_DIST = 0.15    # Distance for emergency stop (m)
        self.SAFE_DIST = 0.4         # Distance considered safe (m)
        self.FOV_ANGLE = math.pi / 3 # 60° forward field of view
        self.SIDE_BIAS = 1.2         # Prefer turning right (positive = right)
        
    def get_avoidance_velocities(
        self,
        lidar_pts: List[Tuple[float, float]],
        goal_robot_frame: Tuple[float, float],
        max_v: float = 0.35,
        max_w: float = 2.0
    ) -> Tuple[float, float]:
        gx, gy = goal_robot_frame
        goal_dist = math.hypot(gx, gy)
        goal_angle = _wrap(math.atan2(gy, gx))
        
        # Check for obstacles in front
        obstacle_info = self._check_front_obstacles(lidar_pts)
        
        # EMERGENCY STOP - something very close
        if obstacle_info['critical']: return 0.0, 0.0
        
        elif obstacle_info['danger']:
            # AVOID - obstacle in danger zone
            v = max_v * 0.3  # Slow down
            
            # Determine dodge direction based on obstacle side
            obs_angle = obstacle_info['angle']
            
            if abs(obs_angle) < 0.2:  # Nearly straight ahead
                # Choose side based on more free space or bias
                left_clear = obstacle_info['left_clear']
                right_clear = obstacle_info['right_clear']
                
                if right_clear > left_clear * self.SIDE_BIAS:
                    w = -max_w * 0.7  # Turn right
                else:
                    w = max_w * 0.7   # Turn left
            else:
                # Obstacle on a side - turn away from it
                w = -math.copysign(max_w * 0.8, obs_angle)
            
            return v, w
        
        else:
            # CLEAR PATH - normal navigation to goal
            # Proportional control to goal
            v = max_v * min(1.0, goal_dist / 1.0)
            w = -2.5 * goal_angle
            
            # Slow down for sharp turns
            if abs(goal_angle) > 0.5:
                v *= 0.4
            
            # Clamp velocities
            v = max(-max_v, min(max_v, v))
            w = max(-max_w, min(max_w, w))
            
            return v, w
    
    def _check_front_obstacles(self, lidar_pts: List[Tuple[float, float]]) -> dict:
        if not lidar_pts:
            return {
                'critical': False,
                'danger': False,
                'distance': float('inf'),
                'angle': 0.0,
                'left_clear': float('inf'),
                'right_clear': float('inf')
            }
        
        min_dist = float('inf')
        min_angle = 0.0
        left_min = float('inf')
        right_min = float('inf')
        
        for x, y in lidar_pts:
            dist = math.hypot(x, y)
            angle = math.atan2(y, x)
            
            # Only consider points in forward field of view
            if abs(angle) < self.FOV_ANGLE and x > 0:
                if dist < min_dist:
                    min_dist = dist
                    min_angle = angle
                
                # Track left/right clearance
                if angle > 0:  # Left side
                    left_min = min(left_min, dist)
                else:  # Right side
                    right_min = min(right_min, dist)
        
        return {
            'critical': min_dist < self.CRITICAL_DIST,
            'danger': min_dist < self.DANGER_DIST,
            'distance': min_dist,
            'angle': min_angle,
            'left_clear': left_min,
            'right_clear': right_min
        }