# =============================================================================
# SPOTTER AUCTION TEST                                        -> Author: ZAHIN
# =============================================================================
# Tests auction-based task assignment for multiple collector robots:
# - Bidding system (robots bid based on path cost)
# - Winner selection (lowest cost wins)
# =============================================================================

from controller import Robot
import json, sys, os

# Add shared library to path
if os.path.dirname(os.path.dirname(__file__)) not in sys.path:
    sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from lib_shared.dual_logger import Logger

# =============================================================================
# SPOTTER TESTER - Automated test controller for auction system
# =============================================================================
class SpotterTester(Robot):
    """
    Main test controller that:
    1. Sends auction requests to collector robots
    2. Receives and evaluates bids
    3. Assigns tasks to lowest bidder
    4. Tracks completion and success rate
    """
    def __init__(self):
        super().__init__()

        self.logger = Logger()
        self.logger.start()

        self.timestep = int(self.getBasicTimeStep())
        self.emitter = self.getDevice('emitter')
        self.receiver = self.getDevice('receiver')
        self.receiver.enable(self.timestep)

        self.current_task_id = 0
        self.active_task_id = None
        self.auction_active = False
        self.auction_start_time = 0.0
        self.pending_idle_events = 0
        self.bids_received = {}  # {task_id: [(collector_id, cost), ...]}
        self.tasks_completed = 0

        # Simple round-robin over test locations
        self.test_locations = [
            (1.5, 2.0, "Sanity Check (Near)"),
            (-4.0, 3.0, "Navigation (Behind Tables)"),
            (2.5, -2.5, "Human Avoidance Zone"),
            (-2.5, 3.5, "Multi-Agent Task A"),
            (2.5, 3.5, "Multi-Agent Task B"),
        ]
        self.location_index = 0

        print("\n" + "="*70)
        print("SPOTTER AUCTION TEST (AUTO)")
        print("="*70 + "\n")

    # -------------------------------------------------------------------------
    # COMMUNICATION
    # -------------------------------------------------------------------------
    def send_message(self, msg):
        self.emitter.send(json.dumps(msg))

    def receive_messages(self):
        messages = []
        while self.receiver.getQueueLength() > 0:
            try:
                messages.append(json.loads(self.receiver.getString()))
            except Exception as e:
                print(f"Parse error: {e}")
            finally:
                self.receiver.nextPacket()
        return messages

    # -------------------------------------------------------------------------
    # MESSAGE PROCESSING
    # -------------------------------------------------------------------------
    def handle_idle(self, msg):
        # Each idle event should result in exactly one auction.
        if self.auction_active:
            self.pending_idle_events += 1
        else:
            self.start_auction()

    def handle_bid(self, msg):
        collector_id = msg.get("collector_id")
        task_id = msg.get("task_id")
        cost = msg.get("cost")
        if task_id is None or task_id != self.active_task_id:
            return
        self.bids_received.setdefault(task_id, []).append((collector_id, cost))
        print(f"→ BID {collector_id}: {cost:.2f} for task {task_id}")

    def handle_collected(self, msg):
        collector_id = msg.get("collector_id")
        print(f"✓ {collector_id} COMPLETED task")
        self.tasks_completed += 1

    # -------------------------------------------------------------------------
    # AUCTION LOGIC
    # -------------------------------------------------------------------------
    def start_auction(self):
        x, z, name = self.test_locations[self.location_index]
        self.location_index = (self.location_index + 1) % len(self.test_locations)

        self.current_task_id += 1
        task_id = self.current_task_id
        self.active_task_id = task_id
        self.bids_received[task_id] = []

        print(f"\n{'─'*70}")
        print(f"**AUCTION #{task_id}**: {name} at ({x:.2f}, {z:.2f})")
        print(f"{'─'*70}")

        self.send_message({
            "event": "auction_start",
            "task_id": task_id,
            "pos": [x, z],
        })

        self.auction_active = True
        self.auction_start_time = self.getTime()

    def finish_auction_if_ready(self, wait_time=2.0):
        if not self.auction_active or self.active_task_id is None:
            return
        if self.getTime() - self.auction_start_time < wait_time:
            return

        task_id = self.active_task_id
        bids = self.bids_received.get(task_id, [])
        if not bids:
            print("No bids received :(")
        else:
            winner_id, winner_cost = min(bids, key=lambda x: x[1])
            print(f"Winner: {winner_id} (cost: {winner_cost:.2f})")
            print("All bids:")
            for sid, cost in sorted(bids, key=lambda x: x[1]):
                marker = "→" if sid == winner_id else " "
                print(f"  {marker} {sid}: {cost:.2f}")

            x, z, _ = self.test_locations[(self.location_index - 1) % len(self.test_locations)]
            self.send_message({
                "event": "assign_task",
                "collector_id": winner_id,
                "task_id": task_id,
                "target_x": x,
                "target_z": z,
            })

        self.auction_active = False
        self.active_task_id = None
        self.auction_start_time = 0.0

        # Handle queued idle events: one auction per queued idle
        if self.pending_idle_events > 0:
            self.pending_idle_events -= 1
            self.start_auction()

    # -------------------------------------------------------------------------
    # MAIN LOOP
    # -------------------------------------------------------------------------
    def run(self):
        while self.step(self.timestep) != -1:
            for msg in self.receive_messages():
                event = msg.get("event")
                if event == "idle":
                    self.handle_idle(msg)
                elif event == "bid":
                    self.handle_bid(msg)
                elif event == "collected":
                    self.handle_collected(msg)

            self.finish_auction_if_ready()

if __name__ == "__main__":
    SpotterTester().run()
