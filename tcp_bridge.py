#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import DisplayTrajectory
from std_msgs.msg import Int32
import socket
import time

# === 設定 (請填入 Pi 的 IP) ===
PI_IP = '10.3.14.59'  # <--- 請確認 IP
PORT = 9999

class TcpBridge(Node):
    def __init__(self):
        super().__init__("tcp_bridge_node")
        
        # 訂閱 MoveIt 路徑
        self.sub_path = self.create_subscription(
            DisplayTrajectory, "/display_planned_path", self.path_callback, 10)
            
        # 訂閱 夾爪指令 (新增的功能)
        self.sub_gripper = self.create_subscription(
            Int32, "/mycobot/gripper_cmd", self.gripper_callback, 10)
        
        print(f"正在連線到 Pi ({PI_IP}:{PORT})...")
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((PI_IP, PORT))
            print("連線成功！準備發送數據！")
        except Exception as e:
            print(f"連線失敗！錯誤: {e}")
            exit()

    def path_callback(self, msg):
        if not msg.trajectory: return
        traj = msg.trajectory[0].joint_trajectory
        print(f"傳輸路徑... ({len(traj.points)} 點)")
        for i, point in enumerate(traj.points):
            if i % 5 == 0 or i == len(traj.points) - 1:
                angles = list(point.positions)
                msg_str = ",".join([str(a) for a in angles])
                self.send_tcp(msg_str)
                time.sleep(0.05)

    def gripper_callback(self, msg):
        # 接收整數: 0=開, 100=閉
        value = msg.data
        print(f"發送夾爪指令: {value}")
        # 發送格式 "G:100"
        self.send_tcp(f"G:{value}")

    def send_tcp(self, text):
        try:
            self.sock.sendall(text.encode('utf-8'))
        except BrokenPipeError:
            print("連線斷了！")

def main(args=None):
    rclpy.init(args=args)
    node = TcpBridge()
    rclpy.spin(node)

if __name__ == "__main__":
    main()