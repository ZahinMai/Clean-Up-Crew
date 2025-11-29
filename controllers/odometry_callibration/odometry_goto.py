# CHANGES TO CONTROL VALUES
# speed_min = 0.2 from 1.0 -> smoother, slower movements (not choppy & buggy now)
# k_beta = 0.0 from -0.35 -> orientation correction wasn't needed
# v_adapt = 200/0.13 from 2000/0.13 -> reduced max linear for controlled movement

import math
PI = math.pi
class OdometryGotoConfiguration:
    def __init__(self):
        self.speed_min = 1

class OdometryGotoState:
    def __init__(self):
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_theta = 0.0

class OdometryGotoResult:
    def __init__(self):
        self.speed_left = 0
        self.speed_right = 0
        self.atgoal = 1


class sOdometryGoto:
    def __init__(self):
        self.track = None
        self.configuration = OdometryGotoConfiguration()
        self.state = OdometryGotoState()
        self.result = OdometryGotoResult()


def odometry_goto_init():
    pass


def odometry_goto_start(og, ot):
    og.track = ot
    og.configuration.speed_min = .5
    og.state.goal_x = 0.0
    og.state.goal_y = 0.0
    og.state.goal_theta = 0.0
    og.result.speed_left = 0
    og.result.speed_right = 0
    og.result.atgoal = 1


def odometry_goto_set_goal(og, goal_x, goal_y, goal_theta):
    og.state.goal_x = goal_x
    og.state.goal_y = goal_y
    og.state.goal_theta = goal_theta
    og.result.atgoal = 0


def odometry_goto_step(og):
    # Get current position
    curPos = [og.track.result.x, og.track.result.y, og.track.result.theta]
    goalPos = [og.state.goal_x, og.state.goal_y, og.state.goal_theta]
    
    dx = goalPos[0] - curPos[0]
    dy = goalPos[1] - curPos[1]
    
    # Controller parameters, an initial choice for the values is given but might be changed
    k_rho = 1.2 
    k_alpha = 1.5  
    k_beta = -0.35     # Reduced from -0.35
       
    # "v_c" is the robot's velocity in its longitudinal direction
    # the values range from -1000 to +1000
    # which corresponds approx. to max. 130mm/s
    v_adapt = 2000 / 0.13  # conversion-factor for speed in [m/s] to e-Puck speed units
    
    # "omega_c" is the robot's rotational speed around the vertical axis
    # (positive for turns in counter-clockwise direction)
    # the value is defined to range from -2000 to 2000
    # representing turn rates of max. 270°/s
    omega_adapt = 5000 / (270 * PI / 180)  # conversion-factor for turn rate in [rad/s] to e-Puck speed units

    # Calculate current distance and angles to goal position
    rho_c = math.sqrt(dx * dx + dy * dy)
    
    alpha_c = math.atan2(dy, dx) - curPos[2]
    # To prevent alpha from getting too big
    while alpha_c > PI:
        alpha_c = alpha_c - 2 * PI
    while alpha_c < -PI:
        alpha_c = alpha_c + 2 * PI
    
    
    beta_c = -curPos[2] - alpha_c
    # To prevent beta from getting too big
    while beta_c > PI:
        beta_c = beta_c - 2 * PI
    while beta_c < -PI:
        beta_c = beta_c + 2 * PI
   
    
    v_c = k_rho * rho_c # Linear velocity component
    omega_c = k_alpha * alpha_c # Angular velocity component
    
    # Adapt SI values to e-Puck units
    v_e = v_c * v_adapt
    omega_e = omega_c * omega_adapt
    
    # Finally record motor speed
    og.result.speed_left = int(v_e - omega_e / 2)
    og.result.speed_right = int(v_e + omega_e / 2)
    
    # Don't set speeds < MIN_SPEED (for accuracy reasons)
    if abs(og.result.speed_left) < og.configuration.speed_min:
        og.result.speed_left = 0
    if abs(og.result.speed_right) < og.configuration.speed_min:
        og.result.speed_right = 0
    
    # Termination condition
    if ((og.result.speed_left == 0 and og.result.speed_right == 0) or 
        rho_c < 0.002):
        og.result.atgoal = 1