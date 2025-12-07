# =============================================================================
# SPOTTER AUCTION TEST                                        -> Author: ZAHIN
# =============================================================================
# Tests auction-based task assignment for multiple collector robots:
# - Bidding system (robots bid based on path cost)
# - Winner selection (lowest cost wins)
# - Navigation verification (A* pathfinding around obstacles)
# - Dynamic obstacle avoidance (human detection)
# - Multi-agent coordination (different bots win sequential tasks)
# =============================================================================

from controller import Robot, Keyboard
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
        
        # Setup logging
        self.logger = Logger()
        self.logger.start()
        
        # Robot setup
        self.timestep = int(self.getBasicTimeStep())
        self.robot_id = self.getName()
        
        # Communication devices
        self.emitter = self.getDevice('emitter')
        self.receiver = self.getDevice('receiver')
        self.receiver.enable(self.timestep)
        
        # GPS for position tracking
        self.gps = self.getDevice('gps')
        self.gps.enable(self.timestep)
        
        # Keyboard for manual control
        self.keyboard = Keyboard()
        self.keyboard.enable(self.timestep)
        
        # Configuration
        self.manual = False  # Set True for manual testing
        
        # State tracking
        self.test_phase = 0
        self.start_time = 0
        self.spotters_idle = {}      # {collector_id: (x, z)}
        self.spotters_busy = set()   # {collector_id, ...}
        self.bids_received = {}      # {task_id: [(collector_id, cost), ...]}
        self.current_task_id = 0
        self.tasks_completed = 0
        self.pending_assignment = None  # (task_id, location_idx)
        
        # Test locations: (x, z, description)
        self.test_locations = [
            # Group 1: Basic communication and pickup
            (1.5, 2.0, "Sanity Check (Near)"),
            
            # Group 2: Navigation around static obstacles
            (-4.0, 3.0, "Navigation (Behind Tables)"),

            # Group 3: Dynamic obstacle avoidance (human traffic)
            (2.5, -2.5, "Human Avoidance Zone"),
            
            # Group 4: Multi-agent bidding competition
            (-2.5, 3.5, "Multi-Agent Task A"),
            (2.5, 3.5, "Multi-Agent Task B"),
        ]
        
        self.print_header()
    
    # -------------------------------------------------------------------------
    # COMMUNICATION
    # -------------------------------------------------------------------------
    def send_message(self, msg):
        """Broadcast message to all robots."""
        self.emitter.send(json.dumps(msg))
    
    def receive_messages(self):
        """Collect all pending messages from robots."""
        messages = []
        while self.receiver.getQueueLength() > 0:
            try:
                msg_str = self.receiver.getString()
                messages.append(json.loads(msg_str))
            except Exception as e:
                print(f"⚠ Parse error: {e}")
            finally:
                self.receiver.nextPacket()
        return messages
    
    # -------------------------------------------------------------------------
    # MESSAGE PROCESSING
    # -------------------------------------------------------------------------
    def process_idle(self, msg):
        """Handle robot reporting idle status."""
        collector_id = msg.get("collector_id")
        pos = msg.get("pos")
        
        if collector_id and pos:
            # Move from busy to idle
            if collector_id in self.spotters_busy:
                self.spotters_busy.remove(collector_id)
            self.spotters_idle[collector_id] = tuple(pos)
            print(f"✓ {collector_id} IDLE at ({pos[0]:.2f}, {pos[1]:.2f})")
    
    def process_bid(self, msg):
        """Handle robot bid for auction."""
        collector_id = msg.get("collector_id")
        task_id = msg.get("task_id")
        cost = msg.get("cost")
        
        if task_id is not None:
            if task_id not in self.bids_received:
                self.bids_received[task_id] = []
            self.bids_received[task_id].append((collector_id, cost))
            print(f"→ BID {collector_id}: {cost:.2f} for task {task_id}")
    
    def process_complete(self, msg):
        """Handle task completion notification."""
        collector_id = msg.get("collector_id")
        print(f"✓ {collector_id} COMPLETED task")
        
        self.tasks_completed += 1
        if collector_id in self.spotters_busy:
            self.spotters_busy.remove(collector_id)
    
    # -------------------------------------------------------------------------
    # AUCTION LOGIC
    # -------------------------------------------------------------------------
    def start_auction(self, idx=None):
        """Start new auction for target location."""
        if idx is None:
            idx = self.current_task_id % len(self.test_locations)
        
        x, z, name = self.test_locations[idx]
        self.current_task_id += 1
        task_id = self.current_task_id
        
        print(f"\n{'─'*70}")
        print(f"**AUCTION #{task_id}**: {name} at ({x:.2f}, {z:.2f})")
        print(f"{'─'*70}")
        
        # Broadcast auction to all robots
        self.send_message({
            "event": "auction_start",
            "task_id": task_id,
            "pos": [x, z]
        })
        
        self.bids_received[task_id] = []
        self.pending_assignment = (task_id, idx)
    
    def assign_task(self, task_id):
        """Assign task to lowest bidder."""
        if task_id not in self.bids_received or not self.bids_received[task_id]:
            print("No bids received :(")
            return None
        
        # Find winner (lowest cost)
        bids = self.bids_received[task_id]
        winner_id, winner_cost = min(bids, key=lambda x: x[1])
        
        # Display results
        print(f"\Winner: {winner_id} (cost: {winner_cost:.2f})")
        print("All bids:")
        for sid, cost in sorted(bids, key=lambda x: x[1]):
            marker = "→" if sid == winner_id else " "
            print(f"  {marker} {sid}: {cost:.2f}")
        
        # Send assignment
        _, idx = self.pending_assignment
        x, z, _ = self.test_locations[idx]
        
        self.send_message({
            "event": "assign_task",
            "collector_id": winner_id,
            "task_id": task_id,
            "target_x": x,
            "target_z": z
        })
        
        # Update state
        self.spotters_busy.add(winner_id)
        if winner_id in self.spotters_idle:
            del self.spotters_idle[winner_id]
        self.pending_assignment = None
        
        return winner_id
    
    # -------------------------------------------------------------------------
    # DISPLAY
    # -------------------------------------------------------------------------
    def print_header(self):
        """Display test header."""
        print("\n" + "="*70)
        print("SPOTTER AUCTION TEST")
        if self.manual:
            print("Controls: [A] Auction [S] Status [R] Reset [Q] Quit")
        print("="*70 + "\n")
    
    def print_status(self):
        """Display current system status."""
        print(f"\n{'='*70}")
        print(f"STATUS (t={self.getTime():.1f}s)")
        print(f"{'='*70}")
        print(f"Idle robots: {len(self.spotters_idle)}")
        print(f"Busy robots: {len(self.spotters_busy)}")
        print(f"Auctions run: {self.current_task_id}")
        print(f"Tasks completed: {self.tasks_completed}")
        
        if self.current_task_id > 0:
            rate = self.tasks_completed / self.current_task_id * 100
            print(f"Success rate: {rate:.1f}%")
        print(f"{'='*70}\n")
    
    # -------------------------------------------------------------------------
    # MANUAL CONTROL
    # -------------------------------------------------------------------------
    def handle_keyboard(self):
        """Process keyboard input for manual testing."""
        key = self.keyboard.getKey()
        if key == ord('A'):
            self.start_auction()
        elif key == ord('S'):
            self.print_status()
        elif key == ord('R'):
            print("Reset")
            self.current_task_id = 0
            self.tasks_completed = 0
            self.bids_received = {}
            self.pending_assignment = None
        elif key == ord('Q'):
            print("Quit")
            self.logger.stop()
            self.simulationQuit(0)
    
    # -------------------------------------------------------------------------
    # AUTOMATED TEST SEQUENCE
    # -------------------------------------------------------------------------
    def run_auto(self):
        """Run automated test sequence (12s per test)."""
        now = self.getTime()
        
        # Phase 0: Wait for robots to initialize (5s)
        if self.test_phase == 0:
            if now < 5.0:
                return
            if len(self.spotters_idle) < 1:
                return
            
            self.print_status()
            print("▶ Starting automated tests\n")
            self.test_phase = 1
            self.start_time = now
        
        # Phases 1-N: Run each test location
        elif 1 <= self.test_phase <= len(self.test_locations):
            phase_start = self.start_time + (self.test_phase - 1) * 12.0
            
            # Start auction at phase start
            if now >= phase_start and now < phase_start + 0.1:
                self.start_auction(self.test_phase - 1)
            
            # Assign task 2s after auction
            elif now >= phase_start + 2.0 and now < phase_start + 2.1:
                if self.pending_assignment:
                    task_id, _ = self.pending_assignment
                    self.assign_task(task_id)
            
            # Move to next phase after 12s
            elif now >= phase_start + 12.0:
                self.test_phase += 1
                if self.test_phase > len(self.test_locations):
                    self.test_phase = 99  # Complete
        
        # Phase 99: Test complete, save report
        elif self.test_phase == 99:
            print("\n" + "="*70)
            print("✓ TEST COMPLETE")
            print(f"Auctions: {self.current_task_id}")
            print(f"Completed: {self.tasks_completed}")
            print("="*70 + "\n")
            
            self.logger.stop()
            self.test_phase = 100

    def run(self):
        """Main control loop."""
        while self.step(self.timestep) != -1:
            # Process incoming messages
            messages = self.receive_messages()
            for msg in messages:
                event = msg.get("event")
                if event == "idle":
                    self.process_idle(msg)
                elif event == "bid":
                    self.process_bid(msg)
                elif event == "collected":
                    self.process_complete(msg)
            
            # Run control logic
            if self.manual:
                self.handle_keyboard()
            else:
                self.run_auto()

if __name__ == "__main__":
    SpotterTester().run()