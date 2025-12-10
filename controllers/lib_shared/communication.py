# =============================================================================
#  COMM MODULE              -> AUTHOR: KUNAL
# =============================================================================
#  Messages are JSON objects. The 'event' key determines the payload.
#
#  1. AUCTION START (Spotter -> Collectors)
#     Triggered when Spotter initiates a new task auction.
#     {
#         "event": "auction_start",
#         "task_id": <int>,       # Unique ID for the task/location
#         "pos": [<x>, <z>]       # Target coordinates [float, float]
#     }
#
#  2. BID (Collector -> Spotter)
#     Sent by a Collector in response to an auction.
#     {
#         "event": "bid",
#         "collector_id": <str>,  # Robot's unique name
#         "task_id": <int>,       # Must match the auction's task_id
#         "cost": <float>         # Calculated path cost (e.g., Euclidean distance)
#     }
#
#  3. ASSIGN TASK (Spotter -> Collector)
#     Sent to the winner of the auction.
#     {
#         "event": "assign_task",
#         "collector_id": <str>,  # The winner's ID
#         "task_id": <int>,
#         "target_x": <float>,
#         "target_z": <float>
#     }
#
#  4. COLLECTED (Collector -> Spotter)
#     Sent when a Collector finishes a task (reaches goal).
#     {
#         "event": "collected",
#         "collector_id": <str>,
#         "task_id": <int>,
#         "x": <float>,           # Final X coordinate
#         "y": <float>            # Final Y/Z coordinate
#     }
#
#  5. IDLE (Collector -> Spotter)
#     Sent periodically or immediately after finishing a task to trigger new auctions.
#     {
#         "event": "idle",
#         "collector_id": <str>,
#         "pos": [<x>, <z>]
#     }
# =============================================================================

import json

class Communication:
    def __init__(self, robot, channel=1):
        self.robot = robot
        self.channel = channel
        
        # Setup emitter
        self.emitter = robot.getDevice('emitter')
        if self.emitter:
            self.emitter.setChannel(channel)
        
        # Setup receiver
        self.receiver = robot.getDevice('receiver')
        if self.receiver:
            self.receiver.setChannel(channel)
            timestep = int(robot.getBasicTimeStep())
            self.receiver.enable(timestep)

    def send(self, msg):
        """Send a JSON message."""
        if not self.emitter:
            return False
        try:
            data = json.dumps(msg)
            self.emitter.send(data)
            return True
        except Exception as e:
            print(f"[COMM] Send error: {e}")
            return False

    def receive(self):
        """Receive a single JSON message (oldest in queue)."""
        if not self.receiver:
            return None
        
        if self.receiver.getQueueLength() > 0:
            try:
                msg_str = self.receiver.getString()
                self.receiver.nextPacket()
                return json.loads(msg_str)
            except Exception as e:
                print(f"[COMM] Receive error: {e}")
                self.receiver.nextPacket()
                return None
        return None

    def receive_all(self):
        """Receive all pending messages."""
        messages = []
        while self.receiver and self.receiver.getQueueLength() > 0:
            msg = self.receive()
            if msg:
                messages.append(msg)
        return messages