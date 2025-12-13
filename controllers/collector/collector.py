# ============================================= #
#  COLLECTOR FSM              -> AUTHOR: ZAHIN  #
# ============================================= #
# Collector robot: bids while moving, executes  #
# tasks FIFO, reports idle + position for       #
# auctioneer's allocation strategies.         #
# ============================================= #

from controller import Robot
import math, os, sys, datetime

# ==== Shared Library Import ==== #
if os.path.dirname(os.path.dirname(__file__)) not in sys.path:
    sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from lib_shared.communication import Communication
from lib_shared.navigation import Navigator
from lib_shared.dual_logger import Logger
from lib_shared.CONFIG import AUCTION_STRATEGY, CAUTION_ZONE, TRASH_LAYOUT
from collector_coverage import run_coverage_setup
# ======================================================================
# Config
# ======================================================================
WHEEL_RADIUS = 0.033
AXLE_LENGTH  = 0.16
CHANNEL      = 1

def get_lidar_points(lidar, max_r=3.5, min_r=0.1):
    if not lidar:
        return []
    try:
        ranges = lidar.getRangeImage()
    except Exception:
        return []
    if not ranges:
        return []
    fov, n = lidar.getFov(), len(ranges)
    return [
        (r * math.cos(-fov/2 + i/(n-1)*fov),
         r * math.sin(-fov/2 + i/(n-1)*fov))
        for i, r in enumerate(ranges)
        if min_r <= r <= max_r and math.isfinite(r)
    ]

# =============================================================================
# COLLECTOR ROBOT
# =============================================================================
class Collector(Robot):
    """Auction-based collector robot with A* navigation and FIFO task queue."""

    def __init__(self):
        super().__init__()

        self.timestep    = int(self.getBasicTimeStep())
        self.robot_id    = self.getName()
        self.state       = "IDLE"      # or "NAVIGATING"
        self.current_task_id = None
        self.pending_tasks   = []      # FIFO queue of (task_id, x, z)

        # Logging + communication
        self.logger = Logger(prefix=self.robot_id, enabled=True)
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.logger.start(f"nav_output_{self.robot_id[-1]}_{timestamp}.md")
        self.comm = Communication(self, CHANNEL)

        # Sensors
        self.gps  = self.getDevice('gps'); self.gps.enable(self.timestep)
        self.imu  = self.getDevice('inertial unit'); self.imu.enable(self.timestep)
        self.lidar = self.getDevice('LDS-01')
        self.lidar.enable(self.timestep); self.lidar.enablePointCloud()

        # Motors
        self.lm = self.getDevice('left wheel motor')
        self.rm = self.getDevice('right wheel motor')
        for m in (self.lm, self.rm):
            m.setPosition(float('inf'))
            m.setVelocity(0.0)

        # Navigation
        self.nav = Navigator(map_inflation=1)

        # Pose cache
        self.rx = self.ry = self.yaw = 0.0

        # Idle heartbeat
        self.last_idle_broadcast = 0.0
        self.idle_interval       = 2.0

        self.logger.write(f"{self.robot_id}: Initialised and Waiting for Setup")

        # Inform auctioneer immediately (position updates next timestep)
        self.send_idle_status()
        self.last_idle_broadcast = self.getTime()


        # For logging total dist coverage & time
        self.start_time = None
        self.dist_covered = 0.0
    # ================================================================== #
    # Pose update & idle broadcast
    # ================================================================== #
    def update_pose(self):
        if not self.start_time:
            self.start_time = self.getTime()

        if self.gps.getSamplingPeriod() > 0:
            x, y, _ = self.gps.getValues()
            self.rx, self.ry = x, y
        if self.imu.getSamplingPeriod() > 0:
            self.yaw = self.imu.getRollPitchYaw()[2]

    def send_idle_status(self):
        """Broadcast idle state + position so auctioneer can pick next task."""
        sent = self.comm.send({
            "event": "idle",
            "collector_id": self.robot_id,
            "pos": [self.rx, self.ry],
        })
        if sent:
            self.logger.write(f"{self.robot_id}: broadcast IDLE ({self.rx:.2f}, {self.ry:.2f})\n")
        return sent

    # Handshake / Setup Logic
    def wait_for_setup(self):
        """Blocks until the Supervisor broadcasts the setup configuration."""
        self.logger.write("Waiting for configuration from Supervisor...")
        while self.step(self.timestep) != -1:

            while True:
                msg = self.comm.receive()
                if not msg: break

                if msg.get("event") == "configure":
                    setup = msg.get("setup", "AUCTION")
                    rubbish = msg.get("rubbish_list", [])
                    self.logger.write(f"Received Configuration: {setup}")
                    return setup, rubbish

        return "AUCTION", [] # Fallback
    
    # ================================================================== #
    # Auction Handling
    # ================================================================== #
    def handle_auction_start(self, msg):
        """Robots always bid; auctioneer decides assignment."""
        task_id = msg.get("task_id")
        pos     = msg.get("pos")
        if task_id is None or not pos:
            return

        tx, ty = pos

        cost = math.hypot(tx - self.rx, ty - self.ry)

        self.comm.send({
            "event": "bid",
            "collector_id": self.robot_id,
            "task_id": task_id,
            "cost": cost,
        })
        self.logger.write(f"{self.robot_id}: BID {cost:.2f} for task {task_id}\n")

    def handle_task_assignment(self, msg):
        """Start immediately if idle; else queue for later."""
        if msg.get("collector_id") != self.robot_id:
            return

        task_id = msg.get("task_id")
        x = msg.get("target_x")
        z = msg.get("target_z")

        if None in (task_id, x, z):
            self.logger.write(f"{self.robot_id}: Invalid assignment message\n")
            return

        # Start immediately
        if self.state == "IDLE" and self.current_task_id is None:
            self.logger.write(f"{self.robot_id}: Assigned task {task_id} (start now)\n")
            self.current_task_id = task_id

            if self.nav.plan_path((self.rx, self.ry), (x, z)):
                self.state = "NAVIGATING"
                self.logger.write(f"{self.robot_id}: Path planned -> NAVIGATING\n")
            else:
                self.logger.write(f"{self.robot_id}: Failed to plan path for task {task_id}\n")
                self.current_task_id = None
            return

        # Else queue it
        self.logger.write(f"{self.robot_id}: Queued task {task_id}\n")
        self.pending_tasks.append((task_id, x, z))

    def handle_messages(self):
        for msg in self.comm.receive_all():
            event = msg.get("event")
            if event == "auction_start":
                self.handle_auction_start(msg)
            elif event == "assign_task":
                self.handle_task_assignment(msg)

    # ================================================================== #
    # Task Chaining
    # ================================================================== #
    def _start_next_task_if_any(self):
        if not self.pending_tasks:
            # No more work
            self.current_task_id = None
            self.state = "IDLE"
            self.nav.clear_path()
            self.send_idle_status()
            self.last_idle_broadcast = self.getTime()
            self.logger.write(f"{self.robot_id}: Queue empty. IDLE")
            return

        task_id, x, z = self.pending_tasks.pop(0)
        self.logger.write(f"{self.robot_id}: Starting queued task {task_id}\n")

        self.current_task_id = task_id
        if self.nav.plan_path((self.rx, self.ry), (x, z)):
            self.state = "NAVIGATING"
        else:
            self.logger.write(f"{self.robot_id}: Failed to plan queued task {task_id}\n")
            self._start_next_task_if_any()  # try next one

    # ================================================================== #
    # Main Loop
    # ================================================================== #
    def run(self):
        # == PERFORMANCE METRICS (dist covered, IDLE time, near misses) ==
        self.start_time = self.getTime()
        self.prev_rx = None
        self.prev_ry = None
        dt_seconds = self.timestep / 1000.0 # ms -> s for easier calc
        self.total_idle_time = 0.0
        self.collision_count = 0
        self.is_in_collision = False # To debounce collision counting
        self.dist_covered = 0.0

        
        setup_mode, rubbish_list = self.wait_for_setup()

        self.logger.write(f"\nCollector Started | Mode: {setup_mode}\n")

        try:
            if setup_mode == "SWARM" or setup_mode == "BASELINE":
                run_coverage_setup(self, rubbish_list, setup_mode)
            else:
                # DEFAULT: AUCTION SETUP
                dt_seconds = self.timestep / 1000.0

                while self.step(self.timestep) != -1:
                    self.update_pose()
                    self.handle_messages()
                    
                    # Metric: Distance 
                    if self.prev_rx is not None and self.prev_ry is not None:
                        step_dist = math.hypot(self.rx - self.prev_rx, self.ry - self.prev_ry)
                        self.dist_covered += step_dist
                    self.prev_rx = self.rx
                    self.prev_ry = self.ry

                    if self.state == "IDLE":
                        self.total_idle_time += dt_seconds
                        self.lm.setVelocity(0.0)
                        self.rm.setVelocity(0.0)

                        now = self.getTime()
                        if now - self.last_idle_broadcast >= self.idle_interval:
                            self.send_idle_status()
                            self.last_idle_broadcast = now

                    elif self.state == "NAVIGATING":
                        now = self.getTime()
                        lidar_pts = get_lidar_points(self.lidar)
                        
                        # Metric: Collision 
                        min_dist = float('inf')
                        if lidar_pts:
                            min_dist = min(math.hypot(x, y) for x, y in lidar_pts)
                        
                        if min_dist < CAUTION_ZONE:
                            if not self.is_in_collision:
                                self.collision_count += 1
                                self.is_in_collision = True
                        else:
                            self.is_in_collision = False

                        v, w, moving = self.nav.update(
                            pose=(self.rx, self.ry, self.yaw),
                            lidar_pts=lidar_pts,
                            now=now,
                        )

                        wl = (v + 0.5*w*AXLE_LENGTH) / WHEEL_RADIUS
                        wr = (v - 0.5*w*AXLE_LENGTH) / WHEEL_RADIUS

                        vmax = min(self.lm.getMaxVelocity(), self.rm.getMaxVelocity())
                        wl = max(-vmax, min(vmax, wl))
                        wr = max(-vmax, min(vmax, wr))

                        self.lm.setVelocity(wl)
                        self.rm.setVelocity(wr)

                        if not moving:
                            self.logger.write(f"{self.robot_id}: Task {self.current_task_id} COMPLETE\n")
                            self.lm.setVelocity(0.0)
                            self.rm.setVelocity(0.0)
                            self.comm.send({
                                "event": "collected",
                                "collector_id": self.robot_id,
                                "task_id": self.current_task_id,
                                "x": self.rx,
                                "y": self.ry,
                            })
                            self.nav.clear_path()
                            self._start_next_task_if_any()

        finally:
            # LOGGING PERFORMANCE METRICS
            elapsed_time = self.getTime() - self.start_time
            
            self.logger.write("\n" + "="*40 + "\n")
            self.logger.write(f"PERFORMANCE METRICS FOR {setup_mode} | {AUCTION_STRATEGY.upper()} on {TRASH_LAYOUT}\n")
            self.logger.write("="*40 + "\n")
            self.logger.write(f"Total Idle Time:   {self.total_idle_time:.3f} s\n")
            self.logger.write(f"Collision Count:   {self.collision_count}\n")
            self.logger.write(f"Distance covered:   {self.dist_covered:.3f}\n")
            self.logger.write(f"Time Elapsed:   {elapsed_time:.3f} s\n")
            self.logger.write("="*40 + "\n")
            self.logger.stop()


if __name__ == "__main__":
    Collector().run()