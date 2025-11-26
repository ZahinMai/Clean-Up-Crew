from shared_libraries.communication import CommDevice
import math


class SpotterBot:
    def __init__(self, robot):
        self.robot = robot
        self.comm = CommDevice(robot)

        # Dispatcher state
        self.tasks_to_assign = []       # tasks from vision
        self.idle_collectors = []       # collector ids
        self.collector_positions = {}   # last GPS positions

    # ----------------------------------------------------------------------
    # Called every control step
    def dispatcher_update(self):

        # 1. Read all messages for this timestep
        messages = self.comm.receive_all()

        for msg in messages:

            # Collector broadcasting idle status
            if "status" in msg and msg["status"] == "idle":
                robot_id = msg["robot_id"]

                if robot_id not in self.idle_collectors:
                    self.idle_collectors.append(robot_id)

                # Position sent by Collector FSM
                if "pos" in msg:
                    self.collector_positions[robot_id] = msg["pos"]

            # Spotter vision detected trash
            if "event" in msg and msg["event"] == "trash_detected":
                self.tasks_to_assign.append(
                    {"id": msg["id"], "pos": msg["pos"]}
                )

        # 2. Run task allocation
        self.assign_tasks()

    # ----------------------------------------------------------------------
    def assign_tasks(self):
        """Checks if a task + idle robot exists, then assigns."""

        if not self.tasks_to_assign or not self.idle_collectors:
            return  # Nothing to do this cycle

        # Pop first unassigned task
        task = self.tasks_to_assign.pop(0)

        # Find closest idle robot
        best_robot = None
        best_dist = float("inf")

        for robot_id in self.idle_collectors:
            if robot_id not in self.collector_positions:
                continue  # No position info yet

            rx, rz = self.collector_positions[robot_id]
            tx, tz = task["pos"]

            d = math.sqrt((rx - tx)**2 + (rz - tz)**2)

            if d < best_dist:
                best_dist = d
                best_robot = robot_id

        if best_robot is None:
            # No collectors with known GPS location
            return

        # Remove from idle robots
        self.idle_collectors.remove(best_robot)

        # Send task assignment
        assignment = {
            "task": "GOTO_TRASH",
            "id": task["id"],
            "pos": task["pos"],
            "robot_id": best_robot
        }

        print(f"[Dispatcher] Assigned {best_robot} → trash {task['id']}")
        self.comm.send(assignment)
