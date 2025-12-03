"""
Spotter controller
"""

from controller import Robot
import math
import time
import vision           
from lib_shared.communication import Communication


class Spotter(Robot):
    def __init__(self):
        super().__init__()
        self.timestep = int(self.getBasicTimeStep())

        # Devices
        self.camera = self.getDevice("camera")
        self.camera.enable(self.timestep)

        self.gps = self.getDevice("gps")
        self.gps.enable(self.timestep)

        self.imu = self.getDevice("inertial unit")
        self.imu.enable(self.timestep)

        # Wheels 
        self.left = self.getDevice("left wheel motor")
        self.right = self.getDevice("right wheel motor")
        self.left.setPosition(float("inf"))
        self.right.setPosition(float("inf"))
        self.left.setVelocity(0.0)
        self.right.setVelocity(0.0)

        # Communication 
        self.comm = Communication(self, channel=1, verbose=True)
        self.spotter_id = f"spotter_{int(self.getTime() * 1000) % 9999}"

        # Simple patrol waypoints (in world coordinates x,z) (adjusting)
        self.waypoints = [
            (0.5, 0.0),
            (0.5, 1.0),
            (-0.8, 1.0),
            (-0.8, 0.0)
        ]
        self.wp_index = 0

        # Trash detection 
        self.task_counter = 0
        self.cooldown = 2.0  # seconds between reports
        self.last_report_time = -10.0
        

    def get_pose(self):
        gx, _, gz = self.gps.getValues()
        r, p, y = self.imu.getRollPitchYaw()
        return gx, gz, y

    def go_to_waypoint(self, tx, tz) -> bool:
        """Very simple P-controller to steer towards (tx, tz). Returns True if close enough."""
        x, z, th = self.get_pose()
        dx, dz = (tx - x), (tz - z)
        dist = math.hypot(dx, dz)

        if dist < 0.15:
            self.left.setVelocity(0.0)
            self.right.setVelocity(0.0)
            return True

        # heading error
        target_th = math.atan2(dz, dx)
        e = (target_th - th + math.pi) % (2 * math.pi) - math.pi

        v = 4.0           # base wheel speed (rad/s in motor units)
        w = 6.0 * e       # steer gain

        wl = v - w
        wr = v + w

        max_vel = min(self.left.getMaxVelocity(), self.right.getMaxVelocity())
        wl = max(-max_vel, min(max_vel, wl))
        wr = max(-max_vel, min(max_vel, wr))

        self.left.setVelocity(wl)
        self.right.setVelocity(wr)
        return False

   
    def detect_and_report(self):
        now = self.getTime()
        if now - self.last_report_time < self.cooldown:
            return

        if vision.is_trash_visible(self.camera):
            x, z, _ = self.get_pose()
            self.task_counter += 1
            task_id = f"t_{self.task_counter}"

            msg = {
                "event": "trash_spotted",
                "task_id": task_id,
                "pos": [x, z],
                "spotter_id": self.spotter_id
            }

            print(f"[SPOTTER] Trash detected at ({x:.2f}, {z:.2f}) -> task {task_id}")
            self.comm.send(msg)
            self.last_report_time = now


    def run(self):
        print("[SPOTTER] Controller started.")
        while self.step(self.timestep) != -1:
            # Move along patrol waypoints
            tx, tz = self.waypoints[self.wp_index]
            reached = self.go_to_waypoint(tx, tz)
            if reached:
                self.wp_index = (self.wp_index + 1) % len(self.waypoints)

            # Check for trash and report if visible
            self.detect_and_report()


if __name__ == "__main__":
    spotter = Spotter()
    spotter.run()

            

