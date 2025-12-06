# ============================================= #
#  COMM MODULE              -> AUTHOR: KUNAL    #
# ============================================= #
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