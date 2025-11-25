"""
Responsibilities:
- Scan environment and detect trash using vision.py
- Emit trash_found(x, y) to communication system
- Listen for im_idle messages from collectors
- Allocate tasks to idle collectors
- Send go_to_task(x, y, collector_id)
"""

from controller import Robot
from shared_libraries.communication import Communication
from shared_libraries.navigation import NavigationLibrary
from shared_libraries.vision import Vision

class Spotter(Robot):
    def __init__(self):
        super().__init__()
        self.timestep = int(self.getBasicTimeStep())

        # Libraries
        self.comms = Communication(self)
        self.nav = NavigationLibrary(self, self.timestep)
        self.vision = Vision(self)

        # Task Management
        self.trash_queue = []        # list of (x, y)
        self.idle_collectors = []    # list of collector IDs

        # Internal scanning pattern
        self.scan_angle = 0.0
        self.scan_direction = 1

    # ------------------------------------------------------

    def allocate_tasks(self):
        """
        Assigns trash tasks to idle collectors.
        One collector gets one task at a time.
        """
        while self.trash_queue and self.idle_collectors:
            trash_x, trash_y = self.trash_queue.pop(0)
            collector_id = self.idle_collectors.pop(0)

            print(f"[Spotter] Assigning trash ({trash_x}, {trash_y}) to Collector {collector_id}")
            self.comms.send_go_to_task(collector_id, trash_x, trash_y)

    # ------------------------------------------------------

    def run(self):
        print("[Spotter] Spotter controller running...")

        while self.step(self.timestep) != -1:

            # 1. Update Odometry 
            self.nav.update_odometry()

            # 2. Vision: Detect trash 
            trash = self.vision.detect_trash()

            if trash:
                (x, y) = trash
                print(f"[Spotter] TRASH FOUND at ({x:.2f}, {y:.2f})")

                # Add to queue
                self.trash_queue.append((x, y))

                # Emit trash_found message
                self.comms.send_trash_found(x, y)

            # === 3. Communication: Receive messages ===
            msg = self.comms.receive()
            if msg:

                if msg["type"] == "im_idle":
                    collector_id = msg["collector_id"]
                    print(f"[Spotter] Collector {collector_id} is idle")

                    self.idle_collectors.append(collector_id)

                elif msg["type"] == "trash_found":
                    # Ignore — collectors should never emit this
                    pass

            # 4. Allocate tasks if possible 
            self.allocate_tasks()

            # 5. Continue scanning pattern 
            self.perform_scan_motion()

    # ------------------------------------------------------

    def perform_scan_motion(self):
        """Simple back-and-forth rotation for scanning."""
        rotate_speed = 0.5

        # Change direction when limits reached
        if self.scan_angle > 1.5:
            self.scan_direction = -1
        elif self.scan_angle < -1.5:
            self.scan_direction = 1

        self.scan_angle += 0.01 * self.scan_direction

        self.nav.rotate_to_angle(self.scan_angle)


# Run controller
spotter = Spotter()
spotter.run()
