"""
Collector Controller
"""

from controller import Robot
import math
import json
from lib_shared.communication import Communication
from global_planner import AStarPlanner, OccupancyGrid
from local_planner import DWA


class Collector(Robot):

    def __init__(self):
        super().__init__()

        self.timestep = int(self.getBasicTimeStep())

        # DEVICES
        
        self.gps = self.getDevice("gps")
        self.gps.enable(self.timestep)

        self.imu = self.getDevice("inertial unit")
        self.imu.enable(self.timestep)

        self.lidar = self.getDevice("lidar")
        self.lidar.enable(self.timestep)
        self.lidar.enablePointCloud()

        self.left = self.getDevice("left wheel")
        self.right = self.getDevice("right wheel")
        self.left.setPosition(float("inf"))
        self.right.setPosition(float("inf"))

        
        # COMMUNICATION
       
        self.comm = Communication(self, channel=1)
        self.robot_id = "collector_" + str(int(self.getTime()*1000) % 9999)

        # PLANNERS
        
        self.astar = AStarPlanner()
        self.dwa = DWA()

        # Load map (must exist as MAP_STR inside grid_map.py or similar)
        from grid_map import GRID_MAP_STR, ORIGIN, CELL_SIZE
        self.map = OccupancyGrid.from_string(GRID_MAP_STR, CELL_SIZE, ORIGIN)

        # inflate obstacles so robot does not clip corners
        self.map = self.map.inflate(radius=1)

        # FSM
        
        self.state = "IDLE"
        self.current_task = None
        self.path = []
        self.path_i = 0
        self.prev_cmd = (0.0, 0.0)

        # BIN LOCATION (MEASURED FROM WORLD)
        self.bin_pos = (5.4, 3.9)

    -
    # UTILITY: Convert world - grid and grid - world
    
    def world_pos(self):
        pos = self.gps.getValues()
        imu = self.imu.getRollPitchYaw()
        return pos[0], pos[2], imu[2]

    def lidar_points(self):
        """Convert raw lidar point cloud to list of (x,y) in robot frame."""
        cloud = self.lidar.getPointCloud()
        pts = []
        for p in cloud:
            pts.append((p.x, p.z))  # Webots lidar: x forward, z sideways
        return pts

 
    # FSM PROCESSING
   
    def run(self):
        while self.step(self.timestep) != -1:
            msg = self.comm.receive()
            if msg:
                print("Got msg:", msg)

            if self.state == "IDLE":
                self.handle_idle(msg)

            elif self.state == "GETTING_PATH":
                self.handle_getting_path()

            elif self.state == "GOING_TO_TRASH":
                self.handle_going_to_trash()

            elif self.state == "GOING_TO_BIN":
                self.handle_going_to_bin()

    
    # STATE: IDLE
    def handle_idle(self, msg):
        # Broadcast idle so dispatcher can assign task
        self.comm.send({
            "event": "idle",
            "collector_id": self.robot_id
        })

        # Check if dispatcher assigned us a task
        if msg and msg.get("event") == "assign_task":
            if msg["collector_id"] == self.robot_id:
                self.current_task = msg
                self.state = "GETTING_PATH"
                print(f"[{self.robot_id}] Received task:", msg)

    
    # STATE: GETTING_PATH
    def handle_getting_path(self):
        x, y, _ = self.world_pos()

        if self.current_task["target"] == "trash":
            goal = self.current_task["pos"]
        else:
            goal = self.bin_pos

        start = self.map.world_to_grid(x, y)
        end = self.map.world_to_grid(goal[0], goal[1])

        path_cells = self.astar.plan(self.map, start, end)

        if path_cells is None:
            print("PATH NOT FOUND")
            self.state = "IDLE"
            return

        # Convert cell path - world coords
        self.path = [self.map.grid_to_world(r, c) for (r, c) in path_cells]
        self.path_i = 0

        # After path is ready, start moving
        if self.current_task["target"] == "trash":
            self.state = "GOING_TO_TRASH"
        else:
            self.state = "GOING_TO_BIN"

    
    # PATH FOLLOWING (DWA)
    def follow_waypoint(self, target):
        x, y, th = self.world_pos()
        tx, ty = target

        dx = tx - x
        dy = ty - y
        dist = math.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx) - th

        # goal vector in robot frame:
        gx = dist * math.cos(angle_to_goal)
        gy = dist * math.sin(angle_to_goal)

        # obstacle list:
        obs = self.lidar_points()

        v, w = self.dwa.get_safe_velocities(
            lidar_points=obs,
            goal_vec=(gx, gy),
            prev_cmd=self.prev_cmd
        )

        self.prev_cmd = (v, w)

        # convert differential
        left = (v - 0.5 * w * 0.33) / 0.0975
        right = (v + 0.5 * w * 0.33) / 0.0975

        self.left.setVelocity(left)
        self.right.setVelocity(right)

        return dist < 0.25

    # STATE: GOING_TO_TRASH
    def handle_going_to_trash(self):

        if self.path_i >= len(self.path):
            print("Arrived at trash.")
            # notify supervisor
            self.comm.send({
                "event": "collected",
                "collector_id": self.robot_id,
                "task_id": self.current_task["task_id"]
            })
            # now go to bin
            self.current_task = {"target": "bin"}
            self.state = "GETTING_PATH"
            return

        wp = self.path[self.path_i]
        reached = self.follow_waypoint(wp)

        if reached:
            self.path_i += 1
            
    # STATE: GOING_TO_BIN
    def handle_going_to_bin(self):

        if self.path_i >= len(self.path):
            print("Delivered to bin.")
            self.state = "IDLE"
            return

        wp = self.path[self.path_i]
        reached = self.follow_waypoint(wp)

        if reached:
            self.path_i += 1

# Run Controller
collector = Collector()
collector.run()

