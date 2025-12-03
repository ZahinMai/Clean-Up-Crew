"""
Dispatcher Controller 
"""

from controller import Robot
from communication import Communication

class Dispatcher(Robot):
    def __init__(self):
        super().__init__()
        self.timestep = int(self.getBasicTimeStep())

        # Communication (your library)
        self.comm = Communication(self, channel=1, verbose=True)

        # Two queues the dispatcher maintains:
        self.pending_tasks = []      
        self.idle_collectors = []    

        print("[DISPATCH] Dispatcher initialized.")

    
    # Handle incoming messages
    
    def handle_message(self, msg):
        event = msg.get("event", None)

        if event == "trash_spotted":
            # Spotter sends info about new trash
            print(f"[DISPATCH] Trash spotted → Task {msg['task_id']}")
            self.pending_tasks.append({
                "task_id": msg["task_id"],
                "pos": msg["pos"],
                "spotter_id": msg.get("spotter_id", "unknown")
            })

        elif event == "idle":
            # Collector tells it is available
            cid = msg.get("collector_id")
            if cid and cid not in self.idle_collectors:
                self.idle_collectors.append(cid)
                print(f"[DISPATCH] Collector '{cid}' is idle.")

        elif event == "collected":
            # Optionally: a collector finished a task
            print(f"[DISPATCH] Task completed: {msg}")

        else:
            print("[DISPATCH] Unknown message:", msg)

   
    # Perform matching between idle collectors & tasks
  
    def assign_tasks(self):
        """
        Assigns tasks ONLY when both:
        At least one pending task exists
        At least one collector is idle
        """
        while self.pending_tasks and self.idle_collectors:
            task = self.pending_tasks.pop(0)
            collector_id = self.idle_collectors.pop(0)

            assignment = {
                "event": "assign_task",
                "collector_id": collector_id,
                "task_id": task["task_id"],
                "pos": task["pos"],
                "target": "trash"
            }

            print(f"[DISPATCH] Assigning Task {task['task_id']} → Collector {collector_id}")
            self.comm.send(assignment)

    
    # Main loop
    def run(self):
        print("[DISPATCH] Dispatcher running..")

        while self.step(self.timestep) != -1:

            # Receive and process messages
            msg = self.comm.receive()
            if msg:
                self.handle_message(msg)

            # Match tasks whenever possible
            self.assign_tasks()


# Standalone launch
if __name__ == "__main__":
    dispatcher = Dispatcher()
    dispatcher.run()

