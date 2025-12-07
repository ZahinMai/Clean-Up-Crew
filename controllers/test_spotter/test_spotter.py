# ============================================= #
#  temporary task dispatch -> AUTHOR: ZAHIN     #
# ============================================= #
from controller import Robot, Keyboard
import json, sys, datetime, os

class DualLogger:
    def __init__(self):
        self.terminal = sys.stdout
        self.log = []

    def write(self, message):
        self.terminal.write(message)  # Print to console
        self.log.append(message)      # Save to memory

    def flush(self):
        self.terminal.flush()

    def save_to_file(self):
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"report_{timestamp}.md"
        save_directory = "logs"
        # Ensure the directory exists
        os.makedirs(save_directory, exist_ok=True)
        full_path = os.path.join(save_directory, filename)
    
        with open(filename, "w", encoding='utf-8') as f:
            f.write("".join(self.log))
        print(f"File saved successfully to: {full_path}")

class SpotterTester(Robot):
    def __init__(self):        
        super().__init__()

        self.logger = DualLogger()
        sys.stdout = self.logger

        self.timestep = int(self.getBasicTimeStep())
        self.robot_id = self.getName()
        
        self.emitter = self.getDevice('emitter')
        self.receiver = self.getDevice('receiver')
        self.receiver.enable(self.timestep)
        
        self.gps = self.getDevice('gps')
        self.gps.enable(self.timestep)
        
        self.keyboard = Keyboard()
        self.keyboard.enable(self.timestep)
        
        # Configuration
        self.manual = False
        
        # State
        self.test_phase = 0
        self.start_time = 0
        self.spotters_idle = {}
        self.spotters_busy = set()
        self.bids_received = {}
        self.current_task_id = 0
        self.tasks_completed = 0
        self.pending_assignment = None
        
        # Test locations
        self.test_locations = [
            # --- TEST GROUP 1: BASIC COMMS & PICKUP ---
            # Easy target near start to verify bidding works & bot stops exactly at target.
            (1.5, 2.0, "1. Sanity Check (Near)"),

            # --- TEST GROUP 2: NAVIGATION & A* ---
            # Target behind obstacles (Tables) to verify A* finds path around static furniture, not through it.
            (-4.0, 3.0, "2. Navigation (Behind Tables)"),

            # --- TEST GROUP 3: DYNAMIC OBSTACLE AVOIDANCE ---
            # Target across center (where humans usually walk). Watch for pausing/re-routing.
            (2.5, -2.5, "3. Human Avoidance Zone"),

            # --- TEST GROUP 4: BIDDING COMPETITION ---
            # 2 tasks in rapid succession (handled by loop timing).
            # Ensure closest bot wins #4, and a DIFFERENT bot wins #5.
            (-2.5, 3.5, "4. Multi-Agent Task A"), 
            (2.5, 3.5,  "5. Multi-Agent Task B"), 
        ]
        self.print_header()

    def print_header(self):
        print("\n" + "="*70)
        print("SPOTTER AUCTION TEST")
        if self.manual:
            print("Controls: [A] Auction [S] Status [R] Reset [Q] Quit")
        print("="*70 + "\n")

    def send_message(self, msg):
        data = json.dumps(msg)
        self.emitter.send(data)

    def receive_messages(self):
        messages = []
        while self.receiver.getQueueLength() > 0:
            try:
                msg_str = self.receiver.getString()
                messages.append(json.loads(msg_str))
            except Exception as e:
                print(f"Parse error: {e}")
            finally:
                self.receiver.nextPacket()
        return messages

    def process_idle(self, msg):
        collector_id = msg.get("collector_id")
        pos = msg.get("pos")
        if collector_id and pos:
            if collector_id in self.spotters_busy:
                self.spotters_busy.remove(collector_id)
            self.spotters_idle[collector_id] = tuple(pos)
            print(f"  ✓ {collector_id} IDLE at ({pos[0]:.2f}, {pos[1]:.2f})")

    def process_bid(self, msg):
        collector_id = msg.get("collector_id")
        task_id = msg.get("task_id")
        cost = msg.get("cost")
        if task_id is not None:
            if task_id not in self.bids_received:
                self.bids_received[task_id] = []
            self.bids_received[task_id].append((collector_id, cost))
            print(f"   -> BID {collector_id}: {cost:.2f} for task {task_id}")

    def process_complete(self, msg):
        collector_id = msg.get("collector_id")
        print(f"{collector_id} COMPLETED task")
        self.tasks_completed += 1
        if collector_id in self.spotters_busy:
            self.spotters_busy.remove(collector_id)

    def start_auction(self, idx=None):
        if idx is None:
            idx = self.current_task_id % len(self.test_locations)
        
        x, z, name = self.test_locations[idx]
        self.current_task_id += 1
        task_id = self.current_task_id
        
        print(f"\n{'─'*70}")
        print(f"**AUCTION**: #{task_id}: {name} at ({x:.2f}, {z:.2f})")
        print(f"{'─'*70}")
        
        self.send_message({
            "event": "auction_start",
            "task_id": task_id,
            "pos": [x, z]
        })
        
        self.bids_received[task_id] = []
        self.pending_assignment = (task_id, idx)

    def assign_task(self, task_id):
        if task_id not in self.bids_received or not self.bids_received[task_id]:
            print("  No bids received")
            return None
        
        bids = self.bids_received[task_id]
        winner_id, winner_cost = min(bids, key=lambda x: x[1])
        
        print(f"\nLowest path cost {winner_cost:.2f} ({winner_id})")
        for sid, cost in sorted(bids, key=lambda x: x[1]):
            print(f"  {' ->' if sid == winner_id else '  '} {sid}: {cost:.2f}")
        
        _, idx = self.pending_assignment
        x, z, _ = self.test_locations[idx]
        
        self.send_message({
            "event": "assign_task",
            "collector_id": winner_id,
            "task_id": task_id,
            "target_x": x,
            "target_z": z
        })
        
        self.spotters_busy.add(winner_id)
        if winner_id in self.spotters_idle:
            del self.spotters_idle[winner_id]
        
        self.pending_assignment = None
        return winner_id

    def print_status(self):
        print(f"\n{'='*70}")
        print(f"STATUS (t={self.getTime():.1f}s)")
        print(f"{'='*70}")
        print(f"Idle: {len(self.spotters_idle)}, Busy: {len(self.spotters_busy)}")
        print(f"Auctions: {self.current_task_id}, Completed: {self.tasks_completed}")
        if self.current_task_id > 0:
            rate = self.tasks_completed / self.current_task_id * 100
            print(f"Success: {rate:.1f}%")
        print(f"{'='*70}\n")

    def handle_keyboard(self):
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
            self.simulationQuit(0)

    def run_auto(self):
        now = self.getTime()
        
        if self.test_phase == 0:
            if now < 5.0: return
            if len(self.spotters_idle) < 1:  return
            self.print_status()
            print("▶ Starting automated tests\n")
            self.test_phase = 1
            self.start_time = now
        
        elif 1 <= self.test_phase <= len(self.test_locations):
            phase_start = self.start_time + (self.test_phase - 1) * 12.0
            
            if now >= phase_start and now < phase_start + 0.1:
                self.start_auction(self.test_phase - 1)
            elif now >= phase_start + 2.0 and now < phase_start + 2.1:
                if self.pending_assignment:
                    task_id, _ = self.pending_assignment
                    self.assign_task(task_id)
            elif now >= phase_start + 12.0:
                self.test_phase += 1
                if self.test_phase > len(self.test_locations):
                    self.test_phase = 99
        
        elif self.test_phase == 99:
            print("\n" + "="*70)
            print("TEST COMPLETE")
            print(f"Auctions: {self.current_task_id}, Completed: {self.tasks_completed}")
            print("="*70 + "\n")
            
            self.logger.save_to_file()
             
            self.test_phase = 100

    def run(self):
        while self.step(self.timestep) != -1:
            messages = self.receive_messages()
            for msg in messages:
                event = msg.get("event")
                if event == "idle":
                    self.process_idle(msg)
                elif event == "bid":
                    self.process_bid(msg)
                elif event == "collected":
                    self.process_complete(msg)
            
            if self.manual:
                self.handle_keyboard()
            else:
                self.run_auto()

if __name__ == "__main__":
    tester = SpotterTester()
    tester.run()