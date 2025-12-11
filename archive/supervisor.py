# Supervisor Controller 
# Manages trash spawning and collection tracking
# Implemented by: Abdullateef Vahora

from controller import Supervisor
import random
import json
import math

# For experiment purposes
NUM_TRASH = 10 
DELETE_THRESHOLD_DIST = 0.1

CHANNEL = 1
MAP_X_MIN, MAP_X_MAX = -4.0, 4.0
MAP_Y_MIN, MAP_Y_MAX = -6.0, 6.0
WALL_MARGIN = 0.5

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
    mass 0.1
  }
}
"""

class SupervisorController:
    def __init__(self):
        self.supervisor = Supervisor()
        self.timestep = int(self.supervisor.getBasicTimeStep())
        
        self.receiver = self.supervisor.getDevice('receiver')
        self.receiver.enable(self.timestep)
        
        self.active_trash = []
        self.start_time = 0
        self.finished = False

    def spawn_trash(self):
        root = self.supervisor.getRoot()
        children = root.getField('children')
        
        print(f"Supervisor: Spawning {NUM_TRASH} items...")
        
        for i in range(NUM_TRASH):
            rx = random.uniform(MAP_X_MIN + WALL_MARGIN, MAP_X_MAX - WALL_MARGIN)
            ry = random.uniform(MAP_Y_MIN + WALL_MARGIN, MAP_Y_MAX - WALL_MARGIN)
            
            trash_str = TRASH_TEMPLATE % (i, rx, ry)
            children.importMFNodeFromString(-1, trash_str)
            
            node_ref = self.supervisor.getFromDef(f"TRASH_{i}")
            if node_ref:
                self.active_trash.append({'id': i, 'node': node_ref})

    def update_ui(self, time_elapsed):
        """For the timer, task management, and complete message display."""
        trash_left = len(self.active_trash)
        status_text = f"Time: {time_elapsed:.1f}s | Trash Left: {trash_left}/{NUM_TRASH}"
        
        # Timer and Trash Count in top-left
        self.supervisor.setLabel(0, status_text, 0.05, 0.05, 0.1, 0xFFFFFF, 0, "Arial")
        
        if self.finished:
            # Complete message in center
            self.supervisor.setLabel(1, "CLEANING COMPLETE!", 0.3, 0.5, 0.2, 0xFF0000, 0, "Arial Black")

    def check_complete(self, time_elapsed):
        if len(self.active_trash) == 0 and not self.finished:
            self.finished = True
            print(f"Experminet Complete: All {NUM_TRASH} trash collected in {time_elapsed:.2f} seconds.")
            self.supervisor.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE) # Pause simulation to get an accurate time

    def handle_collected_message(self, data):
        robot_x = data['x']
        robot_y = data['y']
        
        closest_trash = None
        min_dist = float('inf')
        
        for trash in self.active_trash:
            t_pos = trash['node'].getPosition() 
            dist = math.sqrt((t_pos[0] - robot_x)**2 + (t_pos[1] - robot_y)**2)
            
            if dist < min_dist:
                min_dist = dist
                closest_trash = trash

        if closest_trash and min_dist < DELETE_THRESHOLD_DIST:
            print(f"Supervisor: Collection Confirmed. Removing TRASH_{closest_trash['id']}")
            closest_trash['node'].remove()
            self.active_trash.remove(closest_trash)
        else:
            print(f"Supervisor Warning: Collector reported collection at ({robot_x:.1f}, {robot_y:.1f}), but closest trash is {min_dist:.2f}m away.")

    def run(self):
        self.spawn_trash()        

        self.start_time = self.supervisor.getTime()
        
        while self.supervisor.step(self.timestep) != -1:
            current_time = self.supervisor.getTime()
            elapsed = current_time - self.start_time
            
            self.update_ui(elapsed)
            
            # Process Messages
            while self.receiver.getQueueLength() > 0:
                message_str = self.receiver.getString()
                json_data = json.loads(message_str)
                if json_data.get('event') == 'collected':
                    self.handle_collected_message(json_data)
                    self.check_complete(elapsed) # Check if complete
           
                self.receiver.nextPacket()

if __name__ == "__main__":
    SupervisorController().run()