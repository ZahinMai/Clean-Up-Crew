from controller import Robot
import os, sys, time, random

# Path hack to find lib_shared
if os.path.dirname(os.path.dirname(__file__)) not in sys.path:
    sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from lib_shared.communication import Communication

class TestSender(Robot):
    def __init__(self):
        super().__init__()
        self.timestep = int(self.getBasicTimeStep())
        self.comm = Communication(self, channel=1)
        
        self.last_auction_time = 0
        self.active_auction = None
        self.AUCTION_DURATION = 2.0 # Wait 2 seconds for bids
        
        # Hardcoded 'fake' trash locations for testing
        self.fake_tasks = [
            (3.5, 2.0),
            (-2.0, -1.0),
            (0.0, 3.0),
            (4.0, -2.0)
        ]

    def run(self):
        print("Dummy Spotter (Auctioneer) Running.")
        
        while self.step(self.timestep) != -1:
            current_time = self.getTime()
            
            # --- 1. Start New Auction (Every 10s if idle) ---
            if not self.active_auction and (current_time - self.last_auction_time > 10.0):
                
                # Pick a random task
                target = random.choice(self.fake_tasks)
                task_id = f"task_{int(current_time)}"
                
                print(f"\n[AUCTION] Opening Auction for Task {task_id} at {target}")
                
                self.active_auction = {
                    "id": task_id,
                    "pos": target,
                    "start_time": current_time,
                    "bids": []
                }
                
                self.comm.send({
                    "event": "auction_start",
                    "task_id": task_id,
                    "pos": target
                })
                self.last_auction_time = current_time

            # --- 2. Process Auction ---
            if self.active_auction:
                
                # Listen for Bids
                msg = self.comm.receive()
                if msg and msg.get("event") == "bid":
                    if msg["task_id"] == self.active_auction["id"]:
                        print(f"   + Bid received from {msg['collector_id']}: {msg['cost']:.2f}")
                        self.active_auction["bids"].append(msg)

                # Close Auction after duration
                if current_time - self.active_auction["start_time"] > self.AUCTION_DURATION:
                    bids = self.active_auction["bids"]
                    
                    if bids:
                        # Sort by cost (lowest wins)
                        winner = sorted(bids, key=lambda x: x["cost"])[0]
                        print(f"[AUCTION] Winner: {winner['collector_id']} (Cost: {winner['cost']:.2f})")
                        
                        self.comm.send({
                            "event": "assign_task",
                            "collector_id": winner['collector_id'],
                            "task_id": self.active_auction["id"],
                            "target_x": self.active_auction["pos"][0],
                            "target_z": self.active_auction["pos"][1]
                        })
                    else:
                        print("[AUCTION] No bids received. Cancelled.")
                    
                    # Reset
                    self.active_auction = None

if __name__ == "__main__":
    DummySpotter().run()