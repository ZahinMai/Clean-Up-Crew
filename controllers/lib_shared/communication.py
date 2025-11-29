import json

class Communication:
    """
    Shared communication library for all robots.
    Provides simple send() and receive() wrappers.
    """

    def __init__(self, robot, channel=1, verbose=False):
        self.robot = robot
        self.verbose = verbose

        # Emitter setup
        self.emitter = robot.getDevice("emitter")
        self.emitter.setChannel(channel)

        # Receiver setup
        self.receiver = robot.getDevice("receiver")
        self.receiver.enable(int(robot.getBasicTimeStep()))
        self.receiver.setChannel(channel)

    # ---------------------------
    # Sending messages
    # ---------------------------
    def send(self, data: dict):
        """
        Sends a JSON message to all devices in this channel.
        """
        try:
            msg = json.dumps(data)
            self.emitter.send(msg.encode('utf-8'))

            if self.verbose:
                print("[COMM-SEND]:", msg)

        except Exception as e:
            print("Communication send error:", e)

    # ---------------------------
    # Receiving messages
    # ---------------------------
    def receive(self):
        """
        Returns the newest message as a Python dict.
        Returns None if no messages are available.
        """
        if self.receiver.getQueueLength() > 0:
            raw = self.receiver.getString()
            if isinstance(raw, bytes): msg = raw.decode("utf-8")
            else: msg = raw

            try:
                data = json.loads(msg)
            except:
                data = None

            # Remove from queue
            self.receiver.nextPacket()

            if self.verbose:
                print("[COMM-RECV]:", data)

            return data

        return None

