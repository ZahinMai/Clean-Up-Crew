import math, os, sys, time

# ---- To Import Shared Libraries ---- #
if os.path.dirname(os.path.dirname(__file__)) not in sys.path:
    sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

# Import your modules
from lib_shared.navigation import Navigator
import lib_shared.map_module

class PatrolAgent:
    def __init__(self):
        # Initialize the Navigator
        # vis_period=1.5 will print the map every 1.5 seconds
        self.nav = Navigator(vis_period=1.5)
        
        # Define waypoints corresponding to the ASCII map numbers (1 -> 7)
        # Coordinates estimated based on map bounds: X[-4, 4], Y[-6, ~4]
        self.waypoints = [
            ( 3.0, -5.0), # 1: Bottom Right
            (-3.0, -5.0), # 2: Bottom Left
            (-3.0, -2.0), # 3: Left Aisle (Lower Mid)
            ( 3.0, -2.0), # 4: Right Aisle (Lower Mid)
            ( 3.0,  1.0), # 5: Right Aisle (Upper Mid)
            (-3.0,  1.0), # 6: Left Aisle (Upper Mid)
            (-3.0,  3.5), # 7: Top Left
        ]
        
        # Create snake pattern: 1->7 then 7->1 (Round trip)
        # We slice [:-1] on the return trip to avoid visiting point 7 twice in a row
        self.full_path = self.waypoints + self.waypoints[-2::-1]
        
        # Initial Robot State (Start at Point 1)
        self.x = 3.0
        self.y = -5.0
        self.yaw = 3.14 # Facing Left
        
    def run(self):
        print(f"Starting Patrol: {len(self.full_path)} waypoints in queue.")
        dt = 0.1 # Simulation time step
        
        for i, target in enumerate(self.full_path):
            print(f"\n>> Heading to Waypoint {i+1}: {target}")
            
            # 1. Use your A* planner to find a path
            success = self.nav.plan_path((self.x, self.y), target)
            
            if not success:
                print(f"FAILED to plan to {target}. Check map connectivity.")
                break
                
            # 2. Control Loop
            while True:
                current_time = time.time()
                
                # Update navigation (returns v, w, and active_status)
                # We pass [] for lidar because collision checking happens in A* here
                v, w, is_active = self.nav.update(
                    (self.x, self.y, self.yaw), 
                    [], 
                    current_time
                )
                
                if not is_active:
                    print(f"   Reached {target}")
                    break
                
                # 3. Simulate Robot Movement
                self.x += v * math.cos(self.yaw) * dt
                self.y += v * math.sin(self.yaw) * dt
                self.yaw += w * dt
                
                # Sleep briefly to make the console visualization readable
                time.sleep(0.05)
                
        print("\nPatrol Complete! Returned to start.")

if __name__ == "__main__":
    pass