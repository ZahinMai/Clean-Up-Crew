# ============================================= #
# TASK MANAGER / AUCTIONEER           -> ZAHIN  #
# ============================================= #
# TASK MANAGER/ AUCTIONEER   -> ABDUL & ZAHIN   #
# ==============================================#
# Spawns rubbish & assigns collection           #
# to Collector bots via a simple auction        #
# ============================================= #

from controller import Supervisor
import json, sys, os
from random import uniform, sample

# ---- To Import Shared Libraries ---- #
if os.path.dirname(os.path.dirname(__file__)) not in sys.path:
    sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from lib_shared.dual_logger import Logger
from lib_shared.map_module import get_map
from lib_shared.CONFIG import auction_strategy

# =========================================================
# Auction strategy:
# =========================================================
# "sequential"   -> lowest unused task_id first
# "nearest_task" -> task closest to any known collector pos
# "random"       -> random unassigned, uncompleted task
# =========================================================
AUCTION_STRATEGY = auction_strategy

# How long to wait for bids before closing an auction
AUCTION_WAIT_TIME = 2.0 # in seconds

TRASH_TEMPLATE = """
DEF TRASH_%d Solid {

# --- rubbish DEFINITION ---
RUBBISH_TEMPLATE = """
DEF RUBBISH_%d Solid {
  translation %f %f 0.1
  children [
    Shape {
      appearance PBRAppearance {
        baseColor 0 0 1
        roughness 0.5
        metalness 0
      }
      geometry Sphere {
        radius 0.05
      }
    }
  ]
  boundingObject Sphere {
    radius 0.05
  }
  physics Physics {
    density -1
    mass 1
  }
}
"""

class SpotterTester(Supervisor):
    """Supervisor that manages trash tasks and auctions them to collectors."""

    def __init__(self):
        super().__init__()

        self.timestep = int(self.getBasicTimeStep())

        # Logging
        self.logger = Logger(prefix="auctioneer", enabled=True)
        self.logger.start()

        # Map (if needed for spawn logic / visualisation)
        self.occupancy_grid = get_map()

        # Communication devices
        self.receiver = self.getDevice("receiver")
        self.receiver.enable(self.timestep)
        self.emitter = self.getDevice("emitter")

        # Get root node for spawning trash
        self.root_children = self.getRoot().getField("children")

        # Task management
        self.trash_locations =  []
        self.completed_task_ids = set()
        self.assigned_task_ids = set()  # tasks auctioned and assigned but not yet completed

        # Auction state
        self.auction_active = False
        self.auction_queued = False
        self.active_task_id = None
        self.auction_start_time = 0.0
        self.bids_received = {}  # task_id -> list[(collector_id, cost)]

        # Collector state (for nearest-task strategy)
        # collector_id -> (x, z)
        self.collector_positions = {}

        print("Auctioneer initialised")
        
        # CRITICAL FIX: Spawn rubbish during initialization
        self.auction_queued = False
        
        self.bids_received = {} 
        self.completed_task_ids = set()

        # Test Locations (X, Y/Z, Name)
        self.rubbish_locations = []
        self.location_index = 0

        self.setup = "AUCTION" # Default
        self_node = self.getFromDef("auctioneer")
        if self_node:
            data = self_node.getField("customData").getSFString()
            if "SETUP:COVERAGE" in data:
                self.setup = "COVERAGE"
            elif "SETUP:AUCTION" in data:
                self.setup = "AUCTION"

        print(f"INITIALIZING IN {self.setup} SETUP")

        # --- SPAWN rubbish ---
        self.root_children = self.getRoot().getField("children")
        self.spawn_rubbish()

        # Broadcast Setup immediately after spawn
        self.broadcast_setup()

        print("SUPERVISOR STARTED")

    # -------------------------------------------------------------------------
    # CONFIGURATION BROADCAST
    # -------------------------------------------------------------------------
    def broadcast_setup(self):
        """Sends the current setup and full rubbish list to all collectors for the coverage setup."""
        clean_list = []
        for i, (x, y, _) in enumerate(self.rubbish_locations):
            clean_list.append({"id": i, "x": x, "y": y})

        msg = {
            "event": "configure",
            "setup": self.setup,
            "rubbish_list": clean_list
        }

        # Send multiple times to ensure collectors wake up and receive it
        for _ in range(5):
            self.send_message(msg)
            self.step(self.timestep)

    # -------------------------------------------------------------------------
    # TRASH INITIALISATION / CLEANUP (stubbed hooks)
    # -------------------------------------------------------------------------
    def spawn_rubbish(self):
        """Spawns rubbish only on free cells in the occupancy grid."""
        print("Spawning rubbish...")

        # Collect all free cells from the grid
        free_cells = [
            (r, c)
            for r in range(self.occupancy_grid.height)
            for c in range(self.occupancy_grid.width)
            if self.occupancy_grid.is_free(r, c)
        ]

        num_rubbish = min(10, len(free_cells))

        # Pick random distinct free cells
        chosen_cells = sample(free_cells, num_rubbish)

        for i, (row, col) in enumerate(chosen_cells):
            # Convert grid cell to world coordinates (cell center)
            x, y = self.occupancy_grid.grid_to_world(row, col)

            # Track this rubbish location
            self.rubbish_locations.append((x, y, f"rubbish at ({x}, {y})"))

            # Remove any existing object with this DEF, just like before
            existing_node = self.getFromDef(f"RUBBISH_{i}")
            if existing_node:
                existing_node.remove()

            # Spawn the rubbish at the chosen free cell
            rubbish_str = RUBBISH_TEMPLATE % (i, x, y)
            self.root_children.importMFNodeFromString(-1, rubbish_str)
            trash_str = TRASH_TEMPLATE % (i, x, y)
            self.root_children.importMFNodeFromString(-1, trash_str)
        
        print(f"Spawned {num_rubbish} rubbish items")
            
    def remove_rubbish(self, task_id):
        """Removes the rubbish associated with the given task ID."""
        node_def = f"RUBBISH_{task_id}"
        rubbish_node = self.getFromDef(node_def)
        
        if rubbish_node:
            rubbish_node.remove()
            print(f"Deleted object: {node_def}")
        else:
            print(f"Warning: Could not find {node_def} to delete.")

    # -------------------------------------------------------------------------
    # COMMUNICATION
    # -------------------------------------------------------------------------
    def send_message(self, msg):
        self.emitter.send(json.dumps(msg))

    def receive_messages(self):
        """Yield all messages received since last step."""
        msgs = []
        while self.receiver.getQueueLength() > 0:
            try:
                msgs.append(json.loads(self.receiver.getString()))
            except Exception as e:
                print(f"[AUCTIONEER] Failed to decode message: {e}")
            self.receiver.nextPacket()
        return msgs

    # -------------------------------------------------------------------------
    # AUCTION STRATEGY: NEXT TASK SELECTION
    # -------------------------------------------------------------------------

    def select_next_task(self):
        """ Decide which task_id to auction next, based on AUCTION_STRATEGY """
        n_tasks = len(self.trash_locations)
        if len(self.completed_task_ids) >= n_tasks:
            return None

        # Tasks that are not completed and not currently assigned
        available_ids = [
            tid for tid in range(n_tasks)
            if tid not in self.completed_task_ids and tid not in self.assigned_task_ids
        ]
        if not available_ids:
            return None

        # Strategy: sequential – smallest task_id first
        if AUCTION_STRATEGY == "sequential":
            return min(available_ids)

        # Strategy: nearest_task – task closest to any known collector position
        if AUCTION_STRATEGY == "nearest_task" and self.collector_positions:
            best_id, best_dist2 = None, float("inf")
            for tid in available_ids:
                tx, tz, _ = self.trash_locations[tid]
                for (cx, cz) in self.collector_positions.values():
                    dx = tx - cx
                    dz = tz - cz
                    d2 = dx * dx + dz * dz
                    if d2 < best_dist2:
                        best_dist2 = d2
                        best_id = tid
            # Fallback if something went wrong
            return best_id if best_id is not None else min(available_ids)

        # Strategy: random – pick random available task
        if AUCTION_STRATEGY == "random":
            from random import choice
            return choice(available_ids)

        # Default fallback
        return min(available_ids)

    # -------------------------------------------------------------------------
    # MESSAGE HANDLERS
    # -------------------------------------------------------------------------
    def handle_idle(self, msg):
        """ Handle 'idle' messages from collectors.
        - Updates collector position cache.
        - Triggers an auction if needed and tasks remain.
        """
        collector_id = msg.get("collector_id")
        pos = msg.get("pos")

        # Track collector positions for nearest-task strategy
        if collector_id is not None and pos and len(pos) >= 2:
            self.collector_positions[collector_id] = (pos[0], pos[1])

        # Stop if all tasks done
        if len(self.completed_task_ids) >= len(self.trash_locations):
            return
        if len(self.completed_task_ids) >= len(self.rubbish_locations): return

        # In COVERAGE setup, collectors are running their coverage pattern.
        if self.setup == "COVERAGE": return

        # Queue logic: if an auction is ongoing, mark that we should
        # start another once it finishes; otherwise start immediately.
        if self.auction_active:
            self.auction_queued = True
        else:
            self.start_auction()

    def handle_bid(self, msg):
        if self.setup == "COVERAGE": return # Ignore bids in coverage mode

        collector_id = msg.get("collector_id")
        task_id = msg.get("task_id")
        cost = msg.get("cost")

        if task_id is None or task_id != self.active_task_id:
            return

        if collector_id is None or cost is None:
            return

        self.bids_received.setdefault(task_id, []).append((collector_id, cost))
        msg_str = f"-> BID {collector_id}: {cost:.2f} for task {task_id}"
        print(msg_str)

    def handle_collected(self, msg):
        collector_id = msg.get("collector_id")
        task_id = msg.get("task_id")

        if task_id is None:
            return

        # Idempotency check
        if task_id in self.completed_task_ids:
            return

        msg_str = f"OK {collector_id} COMPLETED task {task_id}"
        print(msg_str)

        self.completed_task_ids.add(task_id)
        # Once completed, it's no longer "assigned"
        if task_id in self.assigned_task_ids:
            self.assigned_task_ids.remove(task_id)

        self.remove_rubbish(task_id)

        # Check for termination
        if len(self.completed_task_ids) >= len(self.trash_locations):
            print("=" * 70)
        # Check for Termination
        if len(self.completed_task_ids) >= len(self.rubbish_locations):
            print("="*70)
            print("ALL TASKS COMPLETED. SAVING LOGS & EXITING.")
            print("=" * 70)
            self.logger.stop()
            sys.exit(0)

    # -------------------------------------------------------------------------
    # AUCTION LIFECYCLE
    # -------------------------------------------------------------------------

    def start_auction(self):
        """
        Start a new auction for a selected task according to AUCTION_STRATEGY.
        """
        task_id = self.select_next_task()
        if task_id is None:
        # If no more rubbish to collect, exit
        if self.location_index >= len(self.rubbish_locations):
            return

        x, z, name = self.trash_locations[task_id]
        
        # Else get next rubbish location
        x, z, name = self.rubbish_locations[self.location_index]
        task_id = self.location_index
        
        self.location_index += 1
        self.bids_received[task_id] = []

        print("-" * 50)
        print(f"**AUCTION #{task_id}**: {name} at ({x:.2f}, {z:.2f})")

        self.send_message({
            "event": "auction_start",
            "task_id": task_id,
            "pos": [x, z],
        })

        self.auction_active = True
        self.active_task_id = task_id
        self.auction_start_time = self.getTime()
        # Mark as assigned once we actually give it to someone in finish_auction_if_ready

    def finish_auction_if_ready(self, wait_time=AUCTION_WAIT_TIME):
        """ Close auction if wait_time elapsed and assign the task to the best bidder """
        if not self.auction_active or self.active_task_id is None:
            return

        if self.getTime() - self.auction_start_time < wait_time:
            return

        task_id = self.active_task_id
        bids = self.bids_received.get(task_id, [])

        if not bids:
            print(f"Auction #{task_id} FAILED: No bids.")
            # On failure, just leave task unassigned; it will be picked up
            # by a future call to start_auction() when a collector is free
        else:
            winner_id, winner_cost = min(bids, key=lambda x: x[1])
            print(f"Winner: {winner_id} (cost: {winner_cost:.2f})")

            target_x, target_z, _ = self.trash_locations[task_id]
            
            target_x, target_z, _ = self.rubbish_locations[task_id]

            self.send_message({
                "event": "assign_task",
                "collector_id": winner_id,
                "task_id": task_id,
                "target_x": target_x,
                "target_z": target_z,
            })

            # Mark this task as assigned (until collected)
            self.assigned_task_ids.add(task_id)

        # Reset auction state
        self.auction_active = False
        self.active_task_id = None
        self.auction_start_time = 0.0

        # Start a queued auction if any
        if self.auction_queued:
            self.auction_queued = False
            self.start_auction()

    # -------------------------------------------------------------------------
    # MAIN LOOP
    # -------------------------------------------------------------------------

    def run(self):
        print("Auctioneer running...")
        try:
            while self.step(self.timestep) != -1:
                for msg in self.receive_messages():
                    event = msg.get("event")
                    if event == "idle":
                        self.handle_idle(msg)
                    elif event == "bid":
                        self.handle_bid(msg)
                    elif event == "collected":
                        self.handle_collected(msg)
        while self.step(self.timestep) != -1:
            # Re-broadcast setup periodically 
            if self.getTime() % 5.0 < (self.timestep / 1000.0):
                 self.broadcast_setup()

            for msg in self.receive_messages():
                event = msg.get("event")
                if event == "idle":
                    self.handle_idle(msg)
                elif event == "bid":
                    self.handle_bid(msg)
                elif event == "collected":
                    self.handle_collected(msg)

                self.finish_auction_if_ready()

        finally:
            self.logger.stop()


if __name__ == "__main__":
    SpotterTester().run()