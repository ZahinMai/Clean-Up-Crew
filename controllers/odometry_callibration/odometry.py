import math
PI = math.pi

increments_per_tour = 1000.0 
axis_wheel_ratio = 0.16
wheel_diameter_left = 0.033
wheel_diameter_right = 0.033

scaling_factor = 1

class OdometryTrackResult:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

class OdometryTrackState:
    def __init__(self):
        self.pos_left_prev = 0
        self.pos_right_prev = 0


class OdometryTrackConfiguration:
    def __init__(self):
        self.wheel_distance = 0.0
        self.wheel_conversion_left = 0.0
        self.wheel_conversion_right = 0.0


class sOdometryTrack:
    def __init__(self):
        self.result = OdometryTrackResult()
        self.state = OdometryTrackState()
        self.configuration = OdometryTrackConfiguration()


def odometry_track_init():
    pass


def odometry_track_start_pos(ot, pos_left, pos_right):
    ot.result.x = 0
    ot.result.y = 0
    ot.result.theta = 0
    
    ot.state.pos_left_prev = pos_left
    ot.state.pos_right_prev = pos_right
    
    # Calculate odometry configuration values
    ot.configuration.wheel_distance = (axis_wheel_ratio * scaling_factor * 
                                       (wheel_diameter_left + wheel_diameter_right) / 2)
    ot.configuration.wheel_conversion_left = (wheel_diameter_left * scaling_factor * 
                                             PI / increments_per_tour)
    ot.configuration.wheel_conversion_right = (wheel_diameter_right * scaling_factor * 
                                              PI / increments_per_tour)
    return 1


def odometry_track_step_pos(ot, pos_left, pos_right):
    # Calculate encoder deltas
    delta_pos_left = pos_left - ot.state.pos_left_prev
    delta_pos_right = pos_right - ot.state.pos_right_prev
    
    # Convert to distance traveled
    delta_left = delta_pos_left * ot.configuration.wheel_conversion_left
    delta_right = delta_pos_right * ot.configuration.wheel_conversion_right
    
    # Calculate change in orientation and position
    delta_theta = (delta_right - delta_left) / ot.configuration.wheel_distance
    theta2 = ot.result.theta + delta_theta * 0.5
    
    delta_x = (delta_left + delta_right) * 0.5 * math.cos(theta2)
    delta_y = (delta_left + delta_right) * 0.5 * math.sin(theta2)
    
    # Update position
    ot.result.x += delta_x
    ot.result.y += delta_y
    ot.result.theta += delta_theta
    
    # Normalize theta to [-PI, PI]
    if ot.result.theta > PI:
        ot.result.theta -= 2 * PI
    if ot.result.theta < -PI:
        ot.result.theta += 2 * PI
    
    # Update previous positions
    ot.state.pos_left_prev = pos_left
    ot.state.pos_right_prev = pos_right