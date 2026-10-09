"""ROS command relay for the simulated camera's RViz panel."""
import json
import math
import socket

from std_msgs.msg import Float64


class CameraControls:
    def __init__(self, node):
        self.pitch = 0.0
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.subscription = node.create_subscription(
            Float64, '/camera/pitch_command', self.command, 10
        )
        self.timer = node.create_timer(0.2, self.send)

    def command(self, message):
        if math.isfinite(message.data):
            self.pitch = max(-30.0, min(45.0, message.data))
            self.send()

    def send(self):
        self.socket.sendto(json.dumps({'pitch_deg': self.pitch}).encode(), ('127.0.0.1', 19423))
