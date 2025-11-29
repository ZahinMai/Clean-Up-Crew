from controller import Robot
import os, sys
# ---- To Import Shared Libraries ---- #
if os.path.dirname(os.path.dirname(__file__)) not in sys.path:
    sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
from lib_shared.communication import Communication

class TestSender(Robot):
    def __init__(self):
        super().__init__()
        self.comm = Communication(self, channel=1)
        
    def run(self):
        timestep = int(self.getBasicTimeStep())
        sent = False
        
        while self.step(timestep) != -1:
            # Send message once after 1 second
            if self.getTime() > 2.0 and not sent:
                print("TestSender: Broadcasting task...")
                self.comm.send({
                    "event": "assign_task", 
                    "collector_id": "collector_1",
                    "target_x": 5.5, 
                    "target_z": 1.6
                })
                sent = True

if __name__ == "__main__":
    sender = TestSender()
    sender.run()