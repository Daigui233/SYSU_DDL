import sys
import os
import glob

# Reconstruct ROS 2 python paths if running in a PyInstaller frozen environment
# where PYTHONPATH might have been cleared or overridden.
if "AMENT_PREFIX_PATH" in os.environ:
    for prefix in os.environ["AMENT_PREFIX_PATH"].split(os.pathsep):
        if prefix:
            for pattern in [
                os.path.join(prefix, "lib", "python*", "site-packages"),
                os.path.join(prefix, "lib", "python*", "dist-packages"),
                os.path.join(prefix, "local", "lib", "python*", "site-packages"),
                os.path.join(prefix, "local", "lib", "python*", "dist-packages"),
            ]:
                for path in glob.glob(pattern):
                    if path not in sys.path:
                        sys.path.append(path)

# Fallback to standard installation paths if AMENT_PREFIX_PATH is not in environment
for pattern in [
    "/opt/ros/*/lib/python*/site-packages",
    "/opt/ros/*/lib/python*/dist-packages",
    "/opt/ros/*/local/lib/python*/site-packages",
    "/opt/ros/*/local/lib/python*/dist-packages",
]:
    for path in glob.glob(pattern):
        if path not in sys.path:
            sys.path.append(path)

import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import socket
import math
import json
import threading
import sys
import select
import termios
import tty
import time
import os

# Suppress ROS 2 C++ logging (like TF_DENORMALIZED_QUATERNION) to avoid terminal spam
os.environ['RCUTILS_LOGGING_MIN_SEVERITY'] = '50'

# Default values
WINDOWS_IP = "127.0.0.1"
WINDOWS_PORT = 9005

# Try to load settings from local main_config.json
try:
    config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "main_config.json")
    if os.path.exists(config_path):
        with open(config_path, "r") as f:
            config = json.load(f)
        # Always send locally, app.py runs on the same board
        WINDOWS_IP = "127.0.0.1"
        WINDOWS_PORT = config.get("network", {}).get("control_port", 9005)
        print(f"[ODOM] Loaded config. Port: {WINDOWS_PORT}")
except Exception as e:
    print(f"[ODOM] Config read error: {e}. Using fallback port {WINDOWS_PORT}")

def quat2euler(x, y, z, w):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(2 * (w * y - z * x))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

class OdomToUDP(Node):
    def __init__(self):
        super().__init__("odom_to_udp_json")
        
        # --- 用于记录“零点”的偏移量（包含完整的6个自由度） ---
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.offset_z = 0.0
        self.offset_roll = 0.0
        self.offset_pitch = 0.0
        self.offset_yaw = 0.0
        self.need_reset = False
        self.send_count = 0
        self.last_log_time = self.get_clock().now()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 频率可根据需要调整，例如0.01秒=100Hz (解决位置数据延迟的问题)
        self.timer = self.create_timer(0.01, self.on_timer)
        self.log_timer = self.create_timer(2.0, self.on_log_timer)
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.get_logger().info(f"发送 JSON 数据到 {WINDOWS_IP}:{WINDOWS_PORT}")
        self.get_logger().info("====================================")
        self.get_logger().info(" 准备就绪！请在终端中按下 '1' 来重置原点")
        self.get_logger().info("====================================")

        # 开启后台线程监听键盘输入 (Linux环境下)
        self.key_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.key_thread.start()

        # 开启 UDP 命令监听，以便前端网页可以远程触发重置原点
        self.cmd_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.cmd_socket.bind(("127.0.0.1", WINDOWS_PORT + 1))
            self.cmd_thread = threading.Thread(target=self.cmd_listener, daemon=True)
            self.cmd_thread.start()
            self.get_logger().info(f"命令监听端口绑定成功: 127.0.0.1:{WINDOWS_PORT + 1}")
        except Exception as e:
            self.get_logger().error(f"命令监听端口绑定失败: {e}")

    def cmd_listener(self):
        """监听 UDP 命令以实现远程重置原点"""
        while rclpy.ok():
            try:
                data, _ = self.cmd_socket.recvfrom(1024)
                msg = json.loads(data.decode('utf-8'))
                if msg.get("cmd") == "init_app_system" or msg.get("cmd") == "reset":
                    self.need_reset = True
                    self.get_logger().info("收到远程重置命令，即将重置原点！")
            except Exception:
                pass

    def keyboard_listener(self):
        """监听终端键盘输入的后台线程"""
        if not sys.stdin.isatty():
            self.get_logger().info("非交互式终端，禁用键盘监听功能。")
            return
            
        # 获取当前终端属性
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            # 设置为非阻塞的单字符读取模式
            tty.setcbreak(sys.stdin.fileno())
            while rclpy.ok():
                # 使用 select 实现非阻塞等待
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    if key == '1':
                        self.need_reset = True
                        print("\n[系统提示] 收到指令，已将当前位置(XYZ)和旋转(Roll/Pitch/Yaw)重置为新的(0,0,0)原点！\n")
        finally:
            # 恢复终端原本的属性
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def on_timer(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time())
        except TransformException:
            try:
                t = self.tf_buffer.lookup_transform(
                    'odom',
                    'base_link',
                    rclpy.time.Time())
            except TransformException:
                return

        x = t.transform.translation.x
        y = t.transform.translation.y
        z = t.transform.translation.z

        qx = t.transform.rotation.x
        qy = t.transform.rotation.y
        qz = t.transform.rotation.z
        qw = t.transform.rotation.w

        roll, pitch, yaw = quat2euler(qx, qy, qz, qw)

        # ===================== 关键：处理重置逻辑 =====================
        if self.need_reset:
            self.offset_x = x
            self.offset_y = y
            self.offset_z = z
            self.offset_roll = roll
            self.offset_pitch = pitch
            self.offset_yaw = yaw
            self.need_reset = False

        # 1. 计算相对位置差值
        dx = x - self.offset_x
        dy = y - self.offset_y
        dz = z - self.offset_z

        # 2. 计算相对欧拉角
        rel_roll = roll - self.offset_roll
        rel_pitch = pitch - self.offset_pitch
        rel_yaw = yaw - self.offset_yaw

        # 3. 将世界坐标系的差值(dx, dy)旋转到新的本地坐标系中
        rad = math.radians(self.offset_yaw)
        cos_a = math.cos(rad)
        sin_a = math.sin(rad)
        
        local_x = dx * cos_a + dy * sin_a
        local_y = -dx * sin_a + dy * cos_a
        local_z = dz

        # Send raw relative coordinates. app.py will apply configured axis mappings.
        data = {
            "type": "robot_position",
            "ts": time.time(),
            "pos": [local_x, local_y, local_z],
            "euler": [rel_pitch, rel_yaw, rel_roll] 
        }

        json_str = json.dumps(data)
        self.udp_socket.sendto(json_str.encode(), (WINDOWS_IP, WINDOWS_PORT))
        self.send_count += 1

    def on_log_timer(self):
        if self.send_count > 0:
            self.get_logger().info(f"ODOM sent {self.send_count} pkts in last 2s")
        else:
            self.get_logger().warning("ODOM sent 0 pkts in last 2s (TF lookup 'map'->'base_link' or 'odom'->'base_link' failed or not available)")
        self.send_count = 0

def main(args=None):
    rclpy.init(args=args)
    node = OdomToUDP()
    
    from rclpy.executors import ExternalShutdownException
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        # 恢复终端状态，防止退出后终端输入不显示
        if sys.stdin.isatty():
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

if __name__ == "__main__":
    main()
