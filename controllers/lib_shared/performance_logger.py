from controller import Supervisor
from shared_libraries.communication import Communication
import time

class PerformanceLogger(Supervisor):
    def __init__(self):
        super().__init__()
        self.comm = Communication(self, channel=1)
        self.start_time = time.time()
        self.score = 0

    def run(self):
        timestep = int(self.getBasicTimeStep())

        while self.step(timestep) != -1:

            msg = self.comm.receive()
            if msg:

                if msg.get("event") == "collected":
                    self.score += 1
                    elapsed = time.time() - self.start_time
                    print(f"[LOGGER] Score {self.score}  Time {elapsed:.2f}s")

                    if self.score == 10:
                        print("*** EXPERIMENT COMPLETE ***")
                        print(f"Final time: {elapsed:.2f}s")
                        break
