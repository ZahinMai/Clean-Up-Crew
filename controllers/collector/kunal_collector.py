"""
Collector Robot Controller
IDLE → GETTING_PATH → GOING_TO_TRASH → GOING_TO_BIN → IDLE
"""

from controller import Robot
import math, sys, os

ROOT = os.path.dirname(os.path.dirname(__file__))
sys.path.append(ROOT)

from lib_shared.communication import Communication
from lib_shared.global_planner import AStarPlanner
from lib_shared.local_planner import DWA
from lib_shared.map_module import get_map


class Collector(Robot):
    
    def __init__(self):
        super().__init__()
        self.ts = int(self.getBasicTimeStep())

        # Devices 
        self.left_motor = self.getDevice("left wheel motor")
        self.right_motor = self.getDevice("right wheel motor")
        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)

        self.gps = self.getDevice("gps"); self.gps.enable(self.ts)
        self.imu = self.getDevice("inertial unit"); self.imu.enable(self.ts)
        self.lidar = self.getDevice("LDS-01"); self.lidar.enable(self.ts)

        # IDs 
        self.robot_id = "collector_1"

        # Communication 
        self.comm = Communication(self, channel=1)

        # FSM 
        self.state = "IDLE"
        self.current_task = None
        self.path = []
        self.waypoint_idx = 0

        # Navigation 
        self.map = get_map()
        self.astar = AStarPlanner()
        self.dwa = DWA()

        # For smooth DWA behaviour
        self.prev_cmd = (0.0, 0.0)

        print(f"[Collector] READY | ID = {self.robot_id}")


    # helpers 

    def yaw(self):
        _, _, y = self.imu.getRollPitchYaw()
        return y

    def get_obstacles(self):
        ranges = self.lidar.getRangeImage()
        obs = []
        hfov = self.lidar.getFov()
        res = self.lidar.getHorizontalResolution()
        for i, r in enumerate(ranges):
            if r == float("inf") or r <= 0.03:
                continue
            a = -hfov/2 + hfov * (i / (res - 1))
            obs.append((r*math.cos(a), r*math.sin(a)))
        return obs
    
    def drive(self, v, w):
        R = 0.033
        L = 0.160
        wl = (v - 0.5*w*L) / R
        wr = (v + 0.5*w*L) / R
        max_w = min(self.left_motor.getMaxVelocity(),
                    self.right_motor.getMaxVelocity())
        wl = max(-max_w, min(max_w, wl))
        wr = max(-max_w, min(max_w, wr))
        self.left_motor.setVelocity(wl)
        self.right_motor.setVelocity(wr)

    
    #  FSM STATES
    
    def run(self):

        broadcast_timer = 0

        while self.step(self.ts) != -1:

            # STATE: IDLE 
            if self.state == "IDLE":
                
                # Broadcast idle every 4 seconds
                if self.getTime() - broadcast_timer > 4:
                    self.comm.send({
                        "event": "idle",
                        "collector_id": self.robot_id
                    })
                    broadcast_timer = self.getTime()
                    print("[Collector] Broadcasting idle…")
                msg = self.comm.receive()

                if msg and msg.get("event") == "assign_task":
                    if msg["collector_id"] == self.robot_id:
                        print("[Collector] Task received from dispatcher:", msg)
                        self.current_task = msg
                        self.state = "GETTING_PATH"


            # STATE: GETTING_PATH 
            elif self.state == "GETTING_PATH":

                cx, _, cz = self.gps.getValues()
                tx, tz = self.current_task["task_pos"]

                start = self.map.world_to_grid(cx, cz)
                goal = self.map.world_to_grid(tx, tz)

                path_idxs = self.astar.plan(self.map, start, goal)

                if not path_idxs:
                    print("[Collector] ERROR: No path found!")
                    self.state = "IDLE"
                    continue

                self.path = [self.map.grid_to_world(r, c) for r, c in path_idxs]
                self.waypoint_idx = 0

                print("[Collector] Path planned with", len(self.path), "waypoints")
                self.state = "GOING_TO_TRASH"


            # STATE: GOING_TO_TRASH 
            elif self.state == "GOING_TO_TRASH":

                if self.waypoint_idx >= len(self.path):
                    print("[Collector] Arrived at trash!")
                    
                    # Notify supervisor to despawn trash
                    self.comm.send({
                        "event": "collected",
                        "task_id": self.current_task["task_id"]
                    })

                    # Now go to bin
                    self.current_task = {
                        "task_pos": (0.0, 0.0)  # CHANGE BIN POSITION HERE
                    }
                    self.state = "GETTING_PATH"
                    continue

                # Navigation
                gx, gz = self.path[self.waypoint_idx]
                cx, _, cz = self.gps.getValues()

                dx = gx - cx
                dz = gz - cz
                yaw = self.yaw()

                # Transform goal to robot frame
                rx = dx * math.cos(-yaw) - dz * math.sin(-yaw)
                ry = dx * math.sin(-yaw) + dz * math.cos(-yaw)

                if math.hypot(dx, dz) < 0.15:
                    self.waypoint_idx += 1
                    continue

                obs = self.get_obstacles()
                v, w = self.dwa.get_safe_velocities(obs, (rx, ry),
                                                     prev_cmd=self.prev_cmd,
                                                     cur=self.prev_cmd)
                self.prev_cmd = (v, w)
                self.drive(v, w)


            # STATE: GOING_TO_BIN -
            elif self.state == "GOING_TO_BIN":
                pass



# MAIN
if __name__ == "__main__":
    bot = Collector()
    bot.run()
