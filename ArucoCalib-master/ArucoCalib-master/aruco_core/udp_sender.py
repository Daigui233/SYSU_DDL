"""UDP sender for official robot_position packets."""
import json
import socket


class UdpPoseSender:
    """Send localized vehicle pose to the RK3588S AR receiver."""

    def __init__(self, target_ip="127.0.0.1", target_port=9005, enabled=False):
        self.target_ip = str(target_ip)
        self.target_port = int(target_port)
        self.enabled = bool(enabled)
        self.seq = 0
        self.sent_count = 0
        self.fail_count = 0
        self.last_error = ""
        self.last_packet = None
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def configure(self, target_ip=None, target_port=None, enabled=None):
        if target_ip is not None:
            self.target_ip = str(target_ip).strip()
        if target_port is not None:
            self.target_port = int(target_port)
        if enabled is not None:
            self.enabled = bool(enabled)

    def make_packet(self, x_m, z_m, yaw_deg, height_m=0.16):
        return {
            "type": "robot_position",
            "pos": [float(x_m), float(height_m), float(z_m)],
            "euler": [0.0, float(yaw_deg), 0.0],
        }

    def send_pose(self, x_m, z_m, yaw_deg, height_m=0.16):
        if not self.enabled:
            return False

        packet = self.make_packet(x_m, z_m, yaw_deg, height_m=height_m)
        payload = json.dumps(packet, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
        try:
            self.sock.sendto(payload, (self.target_ip, self.target_port))
        except Exception as exc:
            self.fail_count += 1
            self.last_error = str(exc)
            self.last_packet = packet
            return False

        self.seq += 1
        self.sent_count += 1
        self.last_error = ""
        self.last_packet = packet
        return True
