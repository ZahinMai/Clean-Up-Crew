from shared_libraries.communication import Communication

class Collector(Robot):
    def __init__(self):
        super().__init__()
        self.comm = Communication(self, channel=1)
        self.state = "IDLE"
        self.robot_id = "collector_1"

    def send_idle_status(self):
        self.comm.send({
            "event": "idle",
            "collector_id": self.robot_id
        })

    def run(self):
        timestep = int(self.getBasicTimeStep())

        while self.step(timestep) != -1:

            # Receive messages
            msg = self.comm.receive()
            if msg:

                if msg.get("event") == "assign_task" and msg["collector_id"] == self.robot_id:
                    self.current_task = msg
                    self.state = "GO_TO_TRASH"

            # FSM behaviour here...
