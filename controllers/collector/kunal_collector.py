# COLLECTOR CONTROLLER 

from controller import Robot
import math, os, sys, json

# PATH SETUP-
BASE = os.path.dirname(os.path.abspath(__file__))
CTRL_DIR = os.path.dirname(BASE)
if CTRL_DIR not in sys.path:
    sys.path.append(CTRL_DIR)

from lib_shared.global_planner import AStarPlanner
from lib_shared.local_planner import DWA
from lib_shared.map_module import get_map
from communication import CommunicationManager


# ROBOT CONSTANTS
LEFT = "left wheel motor"
RIGHT = "right wheel motor"
WHEEL_RADIUS = 0.033
AXLE_LENGTH = 0.160
WAYPOINT_TOL = 0.18

# COLLECTOR CLASS

    def __init__(self, robot_id="Collector_1"):
        self.robot = Robot()
        self.ts = int(self.robot.getBasicTimeStep())
        self.id = robot_id

        # MOTORS 
        self.lm = self.robot.getDevice(LEFT); self.lm.setPosition(float('inf'))
        self.rm = self.robot.getDevice(RIGHT); self.rm.setPosition(float('inf'))
        self.lm.setVelocity(0); self.rm.setVelocity(0)

        # SENSORS 
        self.gps = self.robot.getDevice("gps"); self.gps.enable(self.ts)
        self.imu = self.robot.getDevice("inertial unit"); self.imu.enable(self.ts)
        self.lidar = self.robot.getDevice("LDS-01"); self.lidar.enable(self.ts)

        # COMMUNICATION 
        self.comm = CommunicationManager(self.robot, self.id)

        # PATH PLANNING 
        self.map = get_map()
        self.astar = AStarPlanner()
        self.dwa = DWA()
        self.path = []
        self.current_wp = None

        # FSM 
        self.state = "IDLE"
        self.current_task = None
        self.last_idle_broadcast = 0.0

        print(f"[Collector {self.id}] READY.")

   
    # HELPERS
    
    def get_yaw(self):
        _, _, y = self.imu.getRollPitchYaw()
        return y

    def world_to_robot(self, dx, dz, yaw):
        rx = dx * math.cos(-yaw) - dz * math.sin(-yaw)
        rz = dx * math.sin(-yaw) + dz * math.cos(-yaw)
        return rx, rz

    def set_wheel_speeds(self, v, w):
        wl = (v - 0.5 * w * AXLE_LENGTH) / WHEEL_RADIUS
        wr = (v + 0.5 * w * AXLE_LENGTH) / WHEEL_RADIUS
        limit = min(self.lm.getMaxVelocity(), self.rm.getMaxVelocity())
        wl = max(-limit, min(limit, wl))
        wr = max(-limit, min(limit, wr))
        self.lm.setVelocity(wl)
        self.rm.setVelocity(wr)

    def at_waypoint(self, tx, tz):
        x, _, z = self.gps.getValues()
        return math.hypot(tx - x, tz - z) < WAYPOINT_TOL

    
    # FSM STATES
     
    # IDLE -
    def state_idle(self):
        now = self.robot.getTime()

        if now - self.last_idle_broadcast > 5:
            self.comm.send_status("idle")
            self.last_idle_broadcast = now
            print(f"[{self.id}] Broadcasting IDLE...")

        msg = self.comm.receive()
        if msg and "task" in msg:
            print(f"[{self.id}] Received task → {msg}")
            self.current_task = msg
            self.state = "GETTING_PATH"

    # GETTING PATH 
    def state_getting_path(self):
        x, _, z = self.gps.getValues()
        tx, tz = self.current_task["pos"]

        start = self.map.world_to_grid(x, z)
        goal = self.map.world_to_grid(tx, tz)

        path_cells = self.astar.plan(self.map, start, goal)
        if not path_cells:
            print(f"[{self.id}] No path found!")
            self.state = "IDLE"
            return

        self.path = [self.map.grid_to_world(r, c) for r, c in path_cells]
        self.current_wp = self.path.pop(0)

        print(f"[{self.id}] Path planned → {len(self.path)} waypoints.")

        self.state = "GOING_TO_TRASH"

    #  GOING TO TRASH 
    def state_going_to_trash(self):
        x, _, z = self.gps.getValues()
        yaw = self.get_yaw()

        tx, tz = self.current_wp

        dx, dz = (tx - x), (tz - z)
        gx, gy = self.world_to_robot(dx, dz, yaw)

        # LIDAR PROCESSING
        ranges = self.lidar.getRangeImage()
        hfov = self.lidar.getFov()
        n = len(ranges)

        obs = []
        for i, r in enumerate(ranges):
            if r == float("inf") or r <= 0.03:
                continue
            a = -hfov/2 + hfov * (i/(n-1))
            obs.append((r*math.cos(a), r*math.sin(a)))

        v, w = self.dwa.get_safe_velocities(obs, (gx, gy))
        self.set_wheel_speeds(v, w)

        # Reached waypoint
        if self.at_waypoint(tx, tz):
            if self.path:
                self.current_wp = self.path.pop(0)
            else:
                print(f"[{self.id}] Trash reached!")
                self.comm.send_event("collected", self.current_task["id"])

                # Now plan to go to BIN
                self.current_task = {"id": "BIN", "pos": (5.5, -2.5)}
                self.state = "GETTING_PATH"

    # GOING TO BIN 
    def state_going_to_bin(self):
        self.state_going_to_trash()   # identical motion logic

        # BIN handled by supervisor touch sensor

   
    # MAIN LOOP
    
    def run(self):
        while self.robot.step(self.ts) != -1:
            if self.state == "IDLE":
                self.state_idle()

            elif self.state == "GETTING_PATH":
                self.state_getting_path()

            elif self.state == "GOING_TO_TRASH":
                self.state_going_to_trash()

            elif self.state == "GOING_TO_BIN":
                self.state_going_to_bin()



# RUN
if __name__ == "__main__":
    bot = CollectorBot("Collector_1")
    bot.run()

