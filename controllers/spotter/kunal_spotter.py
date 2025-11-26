from controller import Robot
from shared_libraries.communication import Communication
from shared_libraries.navigation import NavigationLibrary
from shared_libraries.vision import Vision

class Spotter(Robot):
    def __init__(self):
        super().__init__()

        self.comm = Communication(self, channel=1)
        self.nav = NavigationLibrary(self)
        self.vision = Vision(self)

        self.tasks_to_assign = []
        self.idle_collectors = []

    def dispatch_tasks(self):

        if not self.tasks_to_assign or not self.idle_collectors:
            return  # Nothing to assign
        
        # Pop one task + nearest collector
        task = self.tasks_to_assign.pop(0)
        robot_id = self.idle_collectors.pop(0)

        # Send assignment
        self.comm.send({
            "event": "assign_task",
            "collector_id": robot_id,
            "task_pos": task["pos"],
            "task_id": task["id"]
        })

        print(f"[DISPATCH] Assigned {task['id']} to {robot_id}")

    def run(self):
        timestep = int(self.getBasicTimeStep())

        while self.step(timestep) != -1:

            # 1. Receive messages
            msg = self.comm.receive()
            if msg:

                # A collector reports it is idle
                if msg.get("event") == "idle":
                    self.idle_collectors.append(msg["collector_id"])

                # A collector reports a completed task
                if msg.get("event") == "collected":
                    print("[SPOTTER] Task completed:", msg)

            # 2. Camera check
            trash = self.vision.detect_trash()
            if trash:
                trash_id, pos = trash
                self.tasks_to_assign.append({"id": trash_id, "pos": pos})
                self.comm.send({
                    "event": "trash_found",
                    "trash_id": trash_id,
                    "pos": pos
                })
                print("[SPOTTER] Found trash:", trash_id)

            # 3. Dispatch tasks
            self.dispatch_tasks()

            # 4. Continue movement
            self.nav.update_odometry()
            

