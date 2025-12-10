# =============================================================================
# COLLECTOR FSM - FOR COLLECTOR BOTS                          -> Author: ZAHIN
# =============================================================================
# Multi-agent collector that participates in auction system, uses A* for
# global planning, optional DWA for local avoidance. FSM: IDLE ↔ NAVIGATING
# =============================================================================

from controller import Robot
import math, os, sys

# ---- To Import Shared Libraries ---- #
if os.path.dirname(os.path.dirname(__file__)) not in sys.path:
    sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from lib_shared.communication import Communication
from lib_shared.navigation import Navigator
from lib_shared.dual_logger import Logger

# =============================================================================
# CONFIGURATION & UTILITIES
# =============================================================================

WHEEL_RADIUS = 0.0325
AXLE_LENGTH = 0.16
DWA_ENABLED = False  # TO TOGGLE DWA AS IT IS A POINT OF FAILURE!!!
CHANNEL = 1
DWA_CONFIG = {
    'DT_SIM': 0.25, 'T_PRED': 1.0, 'NV': 7, 'NW': 10, 'V_MAX': 0.35, 'W_MAX': 2.0,
    'A_V': 1.0, 'A_W': 2.0, 'RADIUS': 0.09, 'SAFE': 0.15, 'ALPHA': 0.5, 'BETA': 0.3,
    'GAMMA': 0.1, 'DELTA': 0.1, 'EPS': 0.1, 'LIDAR_SKIP': 5,
}


def get_lidar_points(lidar, max_r=3.5, min_r=0.1):
    """Return filtered (x, y) lidar points in robot frame."""
    if not lidar:return []
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

        # Setup logging
        self.logger = Logger(prefix=self.robot_id, enabled=True)
        self.logger.start()

        # Communication
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
        self.nav = Navigator(
            dwa_enabled=DWA_ENABLED,
            dwa_config=DWA_CONFIG,
            map_inflation=0,
            vis_period=2.0,
            safety_radius=0.09,
            simple_v_max=0.35,
            simple_w_max=2.0,
        )

        self.current_task_id = None
        self.last_idle_broadcast = 0.0

        # Pose cache (for auction & status messages, not nav)
        self.rx = 0.0
        self.ry = 0.0
        self.yaw = 0.0

        print(f"Initialised in IDLE state")

    # ---------------------------------------------------------------------- #
    # Pose & messaging                                                       #
    # ---------------------------------------------------------------------- #

    def update_pose(self):
        """Update robot pose from sensors."""
        if self.gps.getSamplingPeriod() > 0:
            self.rx, self.ry = self.gps.getValues()[:2]
        if self.imu.getSamplingPeriod() > 0:
            self.yaw = self.imu.getRollPitchYaw()[2]

    def send_idle_status(self):
        """Broadcast idle state and current location."""
        success = self.comm.send({
            "event": "idle",
            "collector_id": self.robot_id,
            "pos": [self.rx, self.ry],
        })
        if success:
            print(f"Broadcast IDLE at ({self.rx:.2f}, {self.ry:.2f})")
        return success

    # ---------------------------------------------------------------------- #
    # Auction handling                                                       #
    # ---------------------------------------------------------------------- #

    def handle_auction_start(self, msg):
        """Submit bid if idle."""
        if self.state != "IDLE":
            return
        task_id, pos = msg.get("task_id"), msg.get("pos")
        if not task_id or not pos:
            print("ERROR: Invalid auction message")
            return

        tx, ty = pos
        cost = math.hypot(tx - self.rx, ty - self.ry)
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
        x = msg.get("target_x")
        z = msg.get("target_z")
        if None in [task_id, x, z]:
            print("ERROR: Invalid task assignment")
            return

        print(f"Assigned {task_id} → ({x:.2f}, {z:.2f})")
        self.current_task_id = task_id

        # Plan path using shared navigator
        if self.nav.plan_path(
            start_world=(self.rx, self.ry),
            goal_world=(x, z),
        ):
            self.state = "NAVIGATING"
        else:
            print("Planning failed, staying IDLE")
            self.current_task_id = None

    def handle_messages(self):
        """Process incoming messages."""
        msg = self.comm.receive()
        if not msg:
            return

        event = msg.get("event")
        if event == "auction_start":
            self.handle_auction_start(msg)
        elif event == "assign_task":
            self.handle_task_assignment(msg)

    # ---------------------------------------------------------------------- #
    # Main loop                                                              #
    # ---------------------------------------------------------------------- #

    def run(self):
        """Main control loop with finite state machine."""
        print(
            f"\n{'='*60}\nCollector Started\n"
            f"Mode: Auction-Based Task Assignment\n{'='*60}\n"
        )

        try:
            while self.step(self.timestep) != -1:
                # Update sensors and messages
                self.update_pose()
                self.handle_messages()

                if self.state == "IDLE":
                    # Stop and broadcast availability
                    self.lm.setVelocity(0)
                    self.rm.setVelocity(0)
                    now = self.getTime()
                    if now - self.last_idle_broadcast >= 2.0:
                        self.send_idle_status()
                        self.last_idle_broadcast = now

                elif self.state == "NAVIGATING":
                    # Lidar -> nav -> (v, w) -> wheels
                    now = self.getTime()
                    lidar_pts = get_lidar_points(self.lidar)

                    v, w, moving = self.nav.update(
                        pose=(self.rx, self.ry, self.yaw),
                        lidar_pts=lidar_pts,
                        now=now,
                    )

                    # Convert (v, w) to wheel angular velocities
                    wl = (v + w * AXLE_LENGTH * 0.5) / WHEEL_RADIUS
                    wr = (v - w * AXLE_LENGTH * 0.5) / WHEEL_RADIUS
                    limit = min(self.lm.getMaxVelocity(), self.rm.getMaxVelocity())

                    self.lm.setVelocity(max(-limit, min(limit, wl)))
                    self.rm.setVelocity(max(-limit, min(limit, wr)))

                    # Goal reached
                    if not moving:
                        print(f"Task {self.current_task_id} COMPLETE")
                        self.comm.send({
                            "event": "collected",
                            "collector_id": self.robot_id,
                            "task_id": self.current_task_id,
                            "x": self.rx,
                            "y": self.ry,
                        })
                        self.current_task_id = None
                        self.state = "IDLE"
                        self.nav.clear_path()
                        # Ensure motors are fully stopped
                        self.lm.setVelocity(0)
                        self.rm.setVelocity(0)

        finally:
            self.logger.stop()


if __name__ == "__main__":
    Collector().run()
