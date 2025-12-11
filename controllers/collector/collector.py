#  COLLECTOR FSM              -> AUTHOR: ZAHIN  #
# ============================================= #
# Controller for Collector bots in task auction #
# config. Bid on tasks & navigate with A*       #


from controller import Robot
import math
import os
import sys

#  To Import Shared Libraries
CTRL_DIR = os.path.dirname(os.path.dirname(__file__))
if CTRL_DIR not in sys.path:
    sys.path.insert(0, CTRL_DIR)

from lib_shared.communication import Communication
from lib_shared.navigation import Navigator
from lib_shared.dual_logger import Logger


# CONFIGURATION

WHEEL_RADIUS = 0.033
AXLE_LENGTH = 0.16
CHANNEL = 1


def get_lidar_points(lidar, max_r=3.5, min_r=0.1):
    """Return filtered (x, y) lidar points in robot frame."""
    if not lidar:
        return []
    try:
        ranges = lidar.getRangeImage()
    except Exception:
        return []
    if not ranges:
        return []

    fov = lidar.getFov()
    n = len(ranges)
    pts = []
    for i, r in enumerate(ranges):
        if not (min_r <= r <= max_r) or not math.isfinite(r):
            continue
        a = -0.5 * fov + (fov * i) / (n - 1)
        x = r * math.cos(a)
        y = r * math.sin(a)
        pts.append((x, y))
    return pts



# COLLECTOR ROBOT

class Collector(Robot):
    """Auction-based collector robot with A* + local navigation."""

    def __init__(self):
        super().__init__()

        # Basic setup
        self.timestep = int(self.getBasicTimeStep())
        self.robot_id = self.getName()
        self.state = "IDLE"  # "IDLE" or "NAVIGATING"

        # Logging
        self.logger = Logger(prefix=self.robot_id, enabled=True)
        self.logger.start()

        # Communication
        self.comm = Communication(self, CHANNEL)

        # Sensors 
        self.gps = self.getDevice("gps")
        self.gps.enable(self.timestep)

        self.imu = self.getDevice("inertial unit")
        self.imu.enable(self.timestep)

        self.lidar = self.getDevice("LDS-01")
        self.lidar.enable(self.timestep)
        self.lidar.enablePointCloud()

        #  Actuators 
        self.lm = self.getDevice("left wheel motor")
        self.rm = self.getDevice("right wheel motor")
        self.lm.setPosition(float("inf"))
        self.rm.setPosition(float("inf"))
        self.lm.setVelocity(0.0)
        self.rm.setVelocity(0.0)

        # Navigation 
        # Navigator handles map, A* planning and local control
        self.nav = Navigator(map_inflation=1)

        self.current_task_id = None

        # Pose cache (world frame)
        self.rx = 0.0
        self.ry = 0.0
        self.yaw = 0.0

        # Idle heartbeat timing
        self.last_idle_broadcast = 0.0
        self.idle_interval = 1.0  # seconds

        print(f"{self.robot_id}: Initialised in IDLE state")

        # Send an initial IDLE immediately so the auctioneer knows we exist
        # (position will be updated after first sensor step, that's fine)
        self.send_idle_status()
        self.last_idle_broadcast = self.getTime()

    
    # Pose & messaging                                                   #
    
    def update_pose(self):
        """Update robot pose from sensors (x,y on ground plane)."""
        if self.gps.getSamplingPeriod() > 0:
            x, y, _ = self.gps.getValues()
            self.rx, self.ry = x, y

        if self.imu.getSamplingPeriod() > 0:
            self.yaw = self.imu.getRollPitchYaw()[2]

    def send_idle_status(self):
        """Broadcast idle state and current location."""
        success = self.comm.send({
            "event": "idle",
            "collector_id": self.robot_id,
            "pos": [self.rx, self.ry],
        })
        return success

    # ---------------------------------------------------------------------- #
    # Auction handling                                                       #
    # ---------------------------------------------------------------------- #
    def handle_auction_start(self, msg):
        """Submit bid if idle."""
        if self.state != "IDLE":
            return

        task_id = msg.get("task_id")
        pos = msg.get("pos")
        if task_id is None or not pos:
            return

        tx, ty = pos
        cost = math.hypot(tx - self.rx, ty - self.ry)

        self.comm.send(
            {
                "event": "bid",
                "collector_id": self.robot_id,
                "task_id": task_id,
                "cost": cost,
            }
        )
        print(f"{self.robot_id}: BID {cost:.2f} for task {task_id}")

    def handle_task_assignment(self, msg: dict):
        """Accept task and transition to NAVIGATING if this robot is winner."""
        if msg.get("collector_id") != self.robot_id:
            return

        task_id = msg.get("task_id")
        x = msg.get("target_x")
        z = msg.get("target_z")

        if None in (task_id, x, z):
            print(f"{self.robot_id}: ERROR – invalid task assignment {msg}")
            return

        print(f"{self.robot_id}: Assigned task {task_id} → ({x:.2f}, {z:.2f})")
        self.current_task_id = task_id

        # Plan a new path; Navigator stores it internally
        ok = self.nav.plan_path((self.rx, self.ry), (x, z))
        if ok:
            self.state = "NAVIGATING"
            print(f"{self.robot_id}: Path planned, switching to NAVIGATING")
        else:
            print(f"{self.robot_id}: Planning failed, remaining IDLE")
            self.current_task_id = None

    def handle_messages(self):
        """Process all pending messages from the auctioneer."""
        for msg in self.comm.receive_all():
            event = msg.get("event")
            if event == "auction_start":
                self.handle_auction_start(msg)
            elif event == "assign_task":
                self.handle_task_assignment(msg)

   
    # Main loop                                                          
    
    def run(self):
        print(f"\nCollector Started | Mode: Auction-Based\n")

        try:
            while self.step(self.timestep) != -1:
                self.update_pose()
                self.handle_messages()

                if self.state == "IDLE":
                    # Stop and periodically announce that we are free
                    self.lm.setVelocity(0.0)
                    self.rm.setVelocity(0.0)

                    now = self.getTime()
                    if now - self.last_idle_broadcast >= self.idle_interval:
                        self.send_idle_status()
                        self.last_idle_broadcast = now

                elif self.state == "NAVIGATING":
                    now = self.getTime()
                    lidar_pts = get_lidar_points(self.lidar)

                    # Navigator computes (v, w) and whether we are still moving
                    v, w, moving = self.nav.update(
                        pose=(self.rx, self.ry, self.yaw),
                        lidar_pts=lidar_pts,
                        now=now,
                    )

                    wl = (v + 0.5 * w * AXLE_LENGTH) / WHEEL_RADIUS
                    wr = (v - 0.5 * w * AXLE_LENGTH) / WHEEL_RADIUS
                    vmax = min(self.lm.getMaxVelocity(), self.rm.getMaxVelocity())

                    wl = max(-vmax, min(vmax, wl))
                    wr = max(-vmax, min(vmax, wr))

                    self.lm.setVelocity(wl)
                    self.rm.setVelocity(wr)

                    # Goal reached: Navigator reports moving == False
                    if not moving:
                        print(f"{self.robot_id}: Task {self.current_task_id} COMPLETE")

                        # 1. Stop motors
                        self.lm.setVelocity(0.0)
                        self.rm.setVelocity(0.0)

                        # 2. Notify auctioneer / supervisor
                        self.comm.send(
                            {
                                "event": "collected",
                                "collector_id": self.robot_id,
                                "task_id": self.current_task_id,
                                "x": self.rx,
                                "y": self.ry,
                            }
                        )

                        # 3. Reset FSM & allow new auctions
                        self.current_task_id = None
                        self.state = "IDLE"

                        # 4. Immediate idle heartbeat to trigger next auction
                        self.send_idle_status()
                        self.last_idle_broadcast = self.getTime()

        finally:
            self.logger.stop()


if __name__ == "__main__":
    Collector().run()
