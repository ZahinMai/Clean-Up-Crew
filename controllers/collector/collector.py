# ============================================= #
#  COLLECTOR FSM              -> AUTHOR: ZAHIN  #
# ============================================= #
# Controller for Collector bots in task auction #
# config. Bid on tasks & navigate with A*       #
# ============================================= #
from controller import Robot
import math, os, sys

# ---- To Import Shared Libraries ---- #
if os.path.dirname(os.path.dirname(__file__)) not in sys.path:
    sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from lib_shared.communication import Communication
from lib_shared.navigation import Navigator
from lib_shared.dual_logger import Logger

# =============================================================================
# CONFIGURATION
# =============================================================================
WHEEL_RADIUS = 0.033
AXLE_LENGTH = 0.16
CHANNEL = 1

def get_lidar_points(lidar, max_r=3.5, min_r=0.1):
    """Return filtered (x, y) lidar points in robot frame."""
    if not lidar: return []
    try: ranges = lidar.getRangeImage()
    except Exception: return []
    if not ranges: return []

    fov, n = lidar.getFov(), len(ranges)
    return [
        (r * math.cos(-fov / 2 + i / (n - 1) * fov),
         r * math.sin(-fov / 2 + i / (n - 1) * fov))
        for i, r in enumerate(ranges)
        if min_r <= r <= max_r and math.isfinite(r)
    ]

# =============================================================================
# COLLECTOR ROBOT
# =============================================================================
class Collector(Robot):
    """Auction-based collector robot with A* navigation."""
    def __init__(self):
        super().__init__()

        # Basic setup
        self.timestep = int(self.getBasicTimeStep())
        self.robot_id = self.getName()
        self.state = "IDLE"

        # Setup Position Logging & Comm
        self.logger = Logger(prefix=self.robot_id, enabled=True)
        self.logger.start()
        self.comm = Communication(self, CHANNEL)

        # ------ Sensor & Actuator Setup ------ #
        self.gps = self.getDevice('gps')
        self.gps.enable(self.timestep)

        self.imu = self.getDevice('inertial unit')
        self.imu.enable(self.timestep)

        self.lidar = self.getDevice('LDS-01')
        self.lidar.enable(self.timestep)
        self.lidar.enablePointCloud()

        self.lm = self.getDevice('left wheel motor')
        self.lm.setPosition(float('inf'))
        self.lm.setVelocity(0)

        self.rm = self.getDevice('right wheel motor')
        self.rm.setPosition(float('inf'))
        self.rm.setVelocity(0)

        # ---------- Navigation Setup ---------- #
        self.nav = Navigator(map_inflation = 1)
        self.current_task_id = None
        self.last_idle_broadcast = 0.0

        # Pose cache
        self.rx = 0.0
        self.ry = 0.0
        self.yaw = 0.0

        print(f"{self.robot_id}: Initialised in IDLE state")

    # ---------------------------------------------------------------------- #
    # Pose & messaging                                                       #
    # ---------------------------------------------------------------------- #
    def update_pose(self):
        """Update robot pose from sensors (x,z on ground plane)."""
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
        
        # Send standardized BID
        self.comm.send({
            "event": "bid",
            "collector_id": self.robot_id,
            "task_id": task_id,
            "cost": cost,
        })
        print(f"Bid {cost:.2f} for task_{task_id}")

    def handle_task_assignment(self, msg):
        """Accept task and transition to NAVIGATING."""
        if msg.get("collector_id") != self.robot_id:
            return
            
        task_id = msg.get("task_id")
        # Standardized keys: target_x, target_z
        x = msg.get("target_x")
        z = msg.get("target_z")
        
        if None in [task_id, x, z]:
            print("ERROR: Invalid task assignment")
            return

        print(f"{self.robot_id}: Assigned {task_id} → ({x:.2f}, {z:.2f})")
        self.current_task_id = task_id

        # Plan path
        if self.nav.plan_path((self.rx, self.ry), (x, z)):
            self.state = "NAVIGATING"
        else:
            print("Planning failed, staying IDLE")
            self.current_task_id = None

    def handle_messages(self):
        """Process incoming messages."""
        msg = self.comm.receive()
        if not msg: return
        event = msg.get("event")
        if event == "auction_start":
            self.handle_auction_start(msg)
        elif event == "assign_task":
            self.handle_task_assignment(msg)

    # ---------------------------------------------------------------------- #
    # Main loop                                                              #
    # ---------------------------------------------------------------------- #
    def run(self):
        print(f"\nCollector Started | Mode: Auction-Based\n")

        try:
            while self.step(self.timestep) != -1:
                self.update_pose()
                self.handle_messages()

                if self.state == "IDLE":
                    self.lm.setVelocity(0)
                    self.rm.setVelocity(0)
                    now = self.getTime()
                    # Heartbeat: broadcast IDLE every 2s
                    if now - self.last_idle_broadcast >= 2.0:
                        self.send_idle_status()
                        self.last_idle_broadcast = now

                elif self.state == "NAVIGATING":
                    now = self.getTime()
                    lidar_pts = get_lidar_points(self.lidar)

                    v, w, moving = self.nav.update(
                        pose=(self.rx, self.ry, self.yaw),
                        lidar_pts=lidar_pts,
                        now=now,
                    )

                    wl = (v + w * AXLE_LENGTH * 0.5) / WHEEL_RADIUS
                    wr = (v - w * AXLE_LENGTH * 0.5) / WHEEL_RADIUS
                    limit = min(self.lm.getMaxVelocity(), self.rm.getMaxVelocity())

                    self.lm.setVelocity(max(-limit, min(limit, wl)))
                    self.rm.setVelocity(max(-limit, min(limit, wr)))

                    # Goal reached
                    if not moving:
                        print(f"Task {self.current_task_id} COMPLETE")
                        
                        # 1. Stop motors
                        self.lm.setVelocity(0)
                        self.rm.setVelocity(0)
                        
                        # 2. Send collected event
                        self.comm.send({
                            "event": "collected",
                            "collector_id": self.robot_id,
                            "task_id": self.current_task_id,
                            "x": self.rx,
                            "y": self.ry,
                        })
                        
                        # 3. Reset State
                        self.current_task_id = None
                        self.state = "IDLE"
                        self.nav.clear_path()
                        
                        # 4. CRITICAL: Trigger next auction immediately
                        self.send_idle_status()
                        self.last_idle_broadcast = self.getTime()

        finally:
            self.logger.stop()

if __name__ == "__main__":
    Collector().run()