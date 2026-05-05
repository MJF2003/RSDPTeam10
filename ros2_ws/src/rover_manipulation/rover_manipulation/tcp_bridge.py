import rclpy
from rclpy.node import Node
from moveit_msgs.msg import DisplayTrajectory
from std_msgs.msg import Int32, Float32MultiArray
import socket
import threading


class SmoothTcpBridge(Node):
    def __init__(self):
        super().__init__('smooth_tcp_bridge')
        self.declare_parameter('pi_host', '10.3.14.59')
        self.declare_parameter('pi_port', 9999)
        self.declare_parameter('socket_timeout_sec', 2.0)
        self.declare_parameter('display_planned_path_topic', '/display_planned_path')
        self.declare_parameter('gripper_cmd_topic', '/mycobot/gripper_cmd')
        self.declare_parameter('override_angles_topic', '/mycobot/override_angles')
        self.declare_parameter('gripper_state_topic', '/mycobot/gripper_state')
        self.declare_parameter('query_period_sec', 1.0)

        self.pi_ip = self.get_parameter('pi_host').value
        self.pi_port = int(self.get_parameter('pi_port').value)
        self.socket_timeout_sec = float(
            self.get_parameter('socket_timeout_sec').value
        )
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(self.socket_timeout_sec)
        self.sock_lock = threading.Lock()

        try:
            self.sock.connect((self.pi_ip, self.pi_port))
            self.get_logger().info(
                f"Connected to arm TCP server at {self.pi_ip}:{self.pi_port}"
            )
        except Exception as e:
            self.get_logger().error(
                f"Failed to connect to arm TCP server "
                f"{self.pi_ip}:{self.pi_port}: {e}"
            )
            return

        display_topic = self.get_parameter('display_planned_path_topic').value
        gripper_cmd_topic = self.get_parameter('gripper_cmd_topic').value
        override_topic = self.get_parameter('override_angles_topic').value
        gripper_state_topic = self.get_parameter('gripper_state_topic').value
        query_period = float(self.get_parameter('query_period_sec').value)

        self.path_sub = self.create_subscription(
            DisplayTrajectory, display_topic, self.path_callback, 10
        )
        self.grip_sub = self.create_subscription(
            Int32, gripper_cmd_topic, self.grip_callback, 10
        )
        self.override_sub = self.create_subscription(
            Float32MultiArray, override_topic, self.override_callback, 10
        )

        self.grip_state_pub = self.create_publisher(
            Int32, gripper_state_topic, 10
        )
        self.timer = self.create_timer(query_period, self.query_gripper)

    def query_gripper(self):
        with self.sock_lock:
            try:
                self.sock.sendall(b"Q")
                data = self.sock.recv(1024).decode('utf-8').strip()
                if data:
                    val = int(data)
                    msg = Int32()
                    msg.data = val
                    self.grip_state_pub.publish(msg)
            except:
                pass

    def send_to_pi(self, text_cmd):
        with self.sock_lock:
            try:
                self.sock.sendall(text_cmd.encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f"Failed to send TCP command: {e}")

    def path_callback(self, msg):
        if not msg.trajectory or not msg.trajectory[0].joint_trajectory.points:
            return
        radians = msg.trajectory[0].joint_trajectory.points[-1].positions[:6]
        self.send_to_pi(",".join([f"{r:.4f}" for r in radians]))

    def grip_callback(self, msg):
        self.send_to_pi(f"G:{msg.data}")

    def override_callback(self, msg):
        self.send_to_pi(",".join([f"{r:.4f}" for r in msg.data[:6]]))

def main():
    rclpy.init()
    node = SmoothTcpBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
