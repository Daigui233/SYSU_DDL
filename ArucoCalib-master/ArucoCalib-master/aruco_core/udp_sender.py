"""UDP sender for official robot_position packets."""
import json
import socket


class UdpPoseSender:
    """Send Windows localization poses in the official AR axis order."""

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
        # Map only at the Windows network boundary. The preview, filtering,
        # history, and calibration code continue using Windows (x, z).
        return {
            "type": "robot_position",
            "pos": [float(z_m), float(height_m), float(x_m)],
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


class UdpGamepadControlSender:
    """Send optional manual gamepad control packets to RK3588S.

    This is deliberately separate from robot_position UDP. Localization keeps
    using board_ip:9005; manual control uses board_ip:9010 by default.
    """

    def __init__(self, target_ip="127.0.0.1", target_port=9010):
        self.target_ip = str(target_ip)
        self.target_port = int(target_port)
        self.seq = 0
        self.sent_count = 0
        self.fail_count = 0
        self.last_error = ""
        self.last_packet = None
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def configure(self, target_ip=None, target_port=None):
        if target_ip is not None:
            self.target_ip = str(target_ip).strip()
        if target_port is not None:
            self.target_port = int(target_port)

    def make_packet(
        self,
        gamepad_mode=False,
        target_speed=0.0,
        track_error=0.0,
        state_cmd=1,
        flags=1,
        safe_stop=False,
        rt=0.0,
        lt=0.0,
        lx=0.0,
        b=False,
        connected=False,
    ):
        return {
            "type": "gamepad_control",
            "source": "ArucoCalib",
            "gamepad_mode": bool(gamepad_mode),
            "seq": int(self.seq),
            "timestamp": time_now(),
            "target_speed": float(target_speed),
            "track_error": float(track_error),
            "state_cmd": int(state_cmd),
            "flags": int(flags),
            "safe_stop": bool(safe_stop),
            "inputs": {
                "rt": float(rt),
                "lt": float(lt),
                "lx": float(lx),
                "b": bool(b),
                "connected": bool(connected),
            },
        }

    def send_control(self, **kwargs):
        packet = self.make_packet(**kwargs)
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


def time_now():
    # Kept local to avoid changing the public imports used by older builds.
    import time

    return time.time()
