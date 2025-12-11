# ============================================= #
# TASK MANAGER/ AUCTIIONEER   -> ABDUL & ZAHIN  #
# ==============================================#
# Spawns rubbish & assigns collection           #
# to Collector bots by which one is closest     #
# ============================================= #

from controller import Supervisor
import json, sys, os
from random import uniform, sample

if os.path.dirname(os.path.dirname(__file__)) not in sys.path:
    sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from lib_shared.dual_logger import Logger
from lib_shared.map_module import get_map

# --- rubbish DEFINITION ---
TRASH_TEMPLATE = """
DEF TRASH_%d Solid {
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
    def __init__(self):
        super().__init__()

        # Enable logging (writes to file in background)
        self.logger = Logger(prefix='spotter', enabled=True)
        self.logger.start()
        self.occupancy_grid = get_map()
        self.timestep = int(self.getBasicTimeStep())
        
        # Comm Setup (Channel 1)
        self.emitter = self.getDevice('emitter')
        self.emitter.setChannel(1)
        self.receiver = self.getDevice('receiver')
        self.receiver.setChannel(1)
        self.receiver.enable(self.timestep)

        # State Variables
        self.active_task_id = None
        self.auction_active = False
        self.auction_start_time = 0.0
        self.auction_queued = False
        
        self.bids_received = {} 
        self.completed_task_ids = set()

        # Test Locations (X, Y/Z, Name)
        self.trash_locations = []
        self.location_index = 0

        # --- SPAWN rubbish ---
        self.root_children = self.getRoot().getField("children")
        self.spawn_rubbish()

        print("SPOTTER AUCTION TEST (AUTO) STARTED")

    # -------------------------------------------------------------------------
    # SUPERVISOR METHODS (Spawn/Delete)
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
            self.trash_locations.append((x, y, f"rubbish at ({x}, {y})"))

            # Remove any existing object with this DEF, just like before
            existing_node = self.getFromDef(f"TRASH_{i}")
            if existing_node:
                existing_node.remove()

            # Spawn the rubbish at the chosen free cell
            trash_str = TRASH_TEMPLATE % (i, x, y)
            self.root_children.importMFNodeFromString(-1, trash_str)
            
    def remove_trash(self, task_id):
        """Removes the rubbish associated with the given task ID."""
        node_def = f"TRASH_{task_id}"
        trash_node = self.getFromDef(node_def)
        
        if trash_node:
            trash_node.remove()
            print(f"Deleted object: {node_def}")
        else:
            print(f"Warning: Could not find {node_def} to delete.")

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
    # HANDLERS
    # -------------------------------------------------------------------------
    def handle_idle(self, msg):
        # Stop if all tasks done
        if len(self.completed_task_ids) >= len(self.trash_locations): return

        # Queue logic
        if self.auction_active:
            self.auction_queued = True
        else:
            self.start_auction()

    def handle_bid(self, msg):
        collector_id = msg.get("collector_id")
        task_id = msg.get("task_id")
        cost = msg.get("cost")
        
        if task_id is None or task_id != self.active_task_id:
            return
            
        self.bids_received.setdefault(task_id, []).append((collector_id, cost))
        msg_str = f"-> BID {collector_id}: {cost:.2f} for task {task_id}"
        print(msg_str)

    def handle_collected(self, msg):
        collector_id = msg.get("collector_id")
        task_id = msg.get("task_id")
        
        # Idempotency check
        if task_id in self.completed_task_ids:
            return
            
        msg_str = f"OK {collector_id} COMPLETED task {task_id}"
        print(msg_str)
        
        self.completed_task_ids.add(task_id)

        self.remove_trash(task_id)

        # Check for Termination
        if len(self.completed_task_ids) >= len(self.trash_locations):
            print("="*70)
            print("ALL TASKS COMPLETED. SAVING LOGS & EXITING.")
            print("="*70)
            self.logger.stop()
            sys.exit(0)

    # -------------------------------------------------------------------------
    # AUCTION LOGIC
    # -------------------------------------------------------------------------
    def start_auction(self):
        # If no more rubbish to collect, exit
        if self.location_index >= len(self.trash_locations):
            return
        
        # Else get next rubbish location
        x, z, name = self.trash_locations[self.location_index]
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

    def finish_auction_if_ready(self, wait_time=2.0):
        if not self.auction_active or self.active_task_id is None:
            return
        
        if self.getTime() - self.auction_start_time < wait_time:
            return

        task_id = self.active_task_id
        bids = self.bids_received.get(task_id, [])
        
        if not bids:
            print(f"Auction #{task_id} FAILED: No bids. Rewinding index.")
            # Retry this task next time
            self.location_index -= 1
        else:
            winner_id, winner_cost = min(bids, key=lambda x: x[1])
            print(f"Winner: {winner_id} (cost: {winner_cost:.2f})")
            
            target_x, target_z, _ = self.trash_locations[task_id]

            self.send_message({
                "event": "assign_task",
                "collector_id": winner_id,
                "task_id": task_id,
                "target_x": target_x,
                "target_z": target_z,
            })

        self.auction_active = False
        self.active_task_id = None
        self.auction_start_time = 0.0

        if self.auction_queued:
            self.auction_queued = False
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