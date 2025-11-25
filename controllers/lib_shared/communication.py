# Shared library for Emitter/Receiver communication.
# Implemented by: Kunal Khose

import json
from controller import Emitter, Receiver

class CommDevice:
    """
    Wrapper for Webots Emitter + Receiver.
    Allows all robots to send JSON dictionaries to each other cleanly.
    """

    def __init__(self, robot, tx_name="emitter", rx_name="receiver", channel=1):
        self.robot = robot
        self.emitter = robot.getDevice(tx_name)
        self.receiver = robot.getDevice(rx_name)

        # Bind to communication channel (same for all robots)
        self.emitter.setChannel(channel)
        self.receiver.setChannel(channel)
        self.receiver.enable(int(robot.getBasicTimeStep()))

    # ----------------------------------------------------------------------

    def send(self, data: dict):
        """Send a JSON dictionary to all robots on same channel."""
        try:
            msg = json.dumps(data).encode("utf-8")
            self.emitter.send(msg)
        except Exception as e:
            print("[CommDevice] ERROR while sending:", e)

    # ----------------------------------------------------------------------

    def receive_all(self):
        """Return a list of all JSON messages available this timestep."""
        messages = []

        while self.receiver.getQueueLength() > 0:
            try:
                raw = self.receiver.getData().decode("utf-8")
                messages.append(json.loads(raw))
            except Exception as e:
                print("[CommDevice] ERROR while receiving:", e)

            self.receiver.nextPacket()
