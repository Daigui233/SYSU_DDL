import cv2
import time
import os
import math
import json
import socket
from datetime import datetime
from enum import Enum

import requests


class ProtocolType(Enum):
    HTTP = "http"
    UDP = "udp"


class NetworkPacketSender:
    def __init__(self, target_ip="127.0.0.1", http_port=80, udp_port=9005, timeout=5):
        self.target_ip = target_ip
        self.http_port = http_port
        self.udp_port = udp_port
        self.timeout = timeout
        self.http_headers = {
            'Content-Type': 'application/json',
            'User-Agent': 'Python-Client/1.0',
        }
        self.http_base_url = f"http://{target_ip}:{http_port}"
        self.udp_socket = None
        self.sequence = 0
        self.udp_initialized = False

    def send_sim_control(self, control_data, protocol=ProtocolType.UDP):
        if protocol == ProtocolType.HTTP:
            return self._send_http(control_data)
        elif protocol == ProtocolType.UDP:
            return self._send_udp(control_data)
        else:
            raise ValueError(f"不支持的协议类型: {protocol}")

    def _send_http(self, control_data):
        url = f"{self.http_base_url}/api/sim_control"
        try:
            response = requests.post(
                url, json=control_data, headers=self.http_headers, timeout=self.timeout
            )
            self._log_http_request(response.status_code)
            return response
        except requests.exceptions.RequestException as e:
            print(f"HTTP请求失败: {e}")
            return None

    def _send_udp(self, control_data):
        if not self.udp_initialized:
            if not self._init_udp():
                return False
        if control_data.get("seq") is None:
            self.sequence += 1
            control_data["seq"] = self.sequence
        if control_data.get("timestamp") is None:
            control_data["timestamp"] = time.time()
        try:
            json_data = json.dumps(control_data)
            self.udp_socket.sendto(
                json_data.encode('utf-8'), (self.target_ip, self.udp_port)
            )
            print(f"UDP发送: {json_data}")
            self._log_udp_request()
            return True
        except Exception as e:
            print(f"UDP发送错误: {e}")
            return False

    def _init_udp(self):
        try:
            self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_socket.settimeout(self.timeout)
            self.udp_initialized = True
            print(f"UDP socket已初始化 -> {self.target_ip}:{self.udp_port}")
            return True
        except socket.error as e:
            print(f"UDP socket初始化错误: {e}")
            return False

    def receive_udp_response(self, buffer_size=1024):
        if not self.udp_initialized:
            return None
        try:
            response, addr = self.udp_socket.recvfrom(buffer_size)
            print(f"收到来自 {addr} 的UDP响应: {response.decode('utf-8')}")
            return response.decode('utf-8')
        except socket.timeout:
            print("UDP接收超时")
            return None
        except Exception as e:
            print(f"UDP接收错误: {e}")
            return None

    def _log_http_request(self, status_code):
        timestamp = datetime.now().strftime('%d/%b/%Y %H:%M:%S')
        print(
            f"{self.target_ip} - - [{timestamp}] "
            f"\"POST /api/sim_control HTTP/1.1\" {status_code} -"
        )

    def _log_udp_request(self):
        timestamp = datetime.now().strftime('%d/%b/%Y %H:%M:%S')
        print(f"{self.target_ip}:{self.udp_port} - - [{timestamp}] UDP数据发送成功")

    def send_robot_position(self, pos, euler, protocol=ProtocolType.UDP, **kwargs):
        control_data = {
            "type": "robot_position",
            "pos": pos,
            "euler": euler,
            **kwargs,
        }
        return self.send_sim_control(control_data, protocol)

    def close(self):
        if self.udp_socket:
            self.udp_socket.close()
            self.udp_initialized = False
            print("UDP socket已关闭")


class DatasetCollector:
    def __init__(self, camera_source=0, target_ip="127.0.0.1", udp_port=9005):
        """
        camera_source: 摄像头索引（整数）或视频流URL（字符串）
        """
        self.cap = cv2.VideoCapture(camera_source)
        if not self.cap.isOpened():
            raise IOError(f"无法打开摄像头或视频源: {camera_source}")

        self.last_capture_time = 0
        self.continuous_interval = 0.1
        self.counter = 0
        self.current_session = None
        self.base_folder = "capture_sessions"
        os.makedirs(self.base_folder, exist_ok=True)

        self.sender = NetworkPacketSender(target_ip=target_ip, udp_port=udp_port)

        self.position = [0.0, 0.0, 0.0]
        self.euler = [0.0, 0.0, 0.0]

        self.move_step = 0.1
        self.rotate_step = 5.0

    def create_new_session(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.current_session = os.path.join(self.base_folder, f"session_{timestamp}")
        os.makedirs(self.current_session, exist_ok=True)
        self.counter = 0
        print(f"新建会话: {self.current_session}")
        return self.current_session

    def capture_frame(self):
        if not self.current_session:
            self.create_new_session()
        ret, frame = self.cap.read()
        if ret:
            filename = f"img_{self.counter:04d}_{datetime.now().strftime('%H%M%S')}.jpg"
            full_path = os.path.join(self.current_session, filename)
            cv2.imwrite(full_path, frame)
            print(f"[{self.counter}] 保存: {os.path.basename(full_path)}")
            self.counter += 1
            return True
        return False

    def _send_current_state(self):
        success = self.sender.send_robot_position(
            self.position, self.euler, ProtocolType.UDP
        )
        if success:
            print(
                f"已发送 -> 位置: x={self.position[0]:.2f}, "
                f"y={self.position[1]:.2f}, z={self.position[2]:.2f}, "
                f"欧拉角: roll={self.euler[0]:.1f}, pitch={self.euler[1]:.1f}, "
                f"yaw={self.euler[2]:.1f}"
            )
        else:
            print("发送失败")

    def control_forward(self):
        yaw_rad = math.radians(self.euler[2])
        dx = self.move_step * math.sin(yaw_rad)
        dz = self.move_step * math.cos(yaw_rad)
        self.position[0] += dx
        self.position[2] += dz
        print(f"向前移动 -> 新位置: ({self.position[0]:.2f}, {self.position[2]:.2f})")
        self._send_current_state()

    def control_back(self):
        yaw_rad = math.radians(self.euler[2])
        dx = -self.move_step * math.sin(yaw_rad)
        dz = -self.move_step * math.cos(yaw_rad)
        self.position[0] += dx
        self.position[2] += dz
        print(f"向后移动 -> 新位置: ({self.position[0]:.2f}, {self.position[2]:.2f})")
        self._send_current_state()

    def control_left(self):
        self.euler[2] = (self.euler[2] + self.rotate_step) % 360
        print(f"向左转 {self.rotate_step}° -> 当前Yaw: {self.euler[2]:.1f}°")
        self._send_current_state()

    def control_right(self):
        self.euler[2] = (self.euler[2] - self.rotate_step) % 360
        print(f"向右转 {self.rotate_step}° -> 当前Yaw: {self.euler[2]:.1f}°")
        self._send_current_state()

    def run(self):
        self.create_new_session()
        print("\n控制说明:")
        print("  SPACE : 截图")
        print("  C     : 创建新会话文件夹")
        print("  W     : 向前移动")
        print("  S     : 向后移动")
        print("  A     : 向左旋转")
        print("  D     : 向右旋转")
        print("  ESC   : 退出程序\n")

        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("无法获取视频帧，退出")
                break

            cv2.imshow('Dataset Collector', frame)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                break
            elif key == ord('c'):
                self.create_new_session()
            elif key == 32:
                now = time.time()
                if now - self.last_capture_time >= self.continuous_interval:
                    self.capture_frame()
                    self.last_capture_time = now
            elif key == ord('w'):
                self.control_forward()
            elif key == ord('s'):
                self.control_back()
            elif key == ord('a'):
                self.control_right()
            elif key == ord('d'):
                self.control_left()

        self.cap.release()
        cv2.destroyAllWindows()
        self.sender.close()


if __name__ == "__main__":
    camera_source = 'http://127.0.0.1:8080/ar_feed'
    collector = DatasetCollector(camera_source, target_ip="127.0.0.1", udp_port=9005)
    collector.run()
