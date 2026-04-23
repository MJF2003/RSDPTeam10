from sensor_msgs.msg import JointState

import rclpy
from rclpy.node import Node


DEFAULT_ARM_JOINT_NAMES = [
    "arm_joint2_to_joint1",
    "arm_joint3_to_joint2",
    "arm_joint4_to_joint3",
    "arm_joint5_to_joint4",
    "arm_joint6_to_joint5",
    "arm_joint6output_to_joint6",
    "arm_gripper_controller",
]


class SimArmJointStatePublisher(Node):
    def __init__(self):
        super().__init__("sim_arm_joint_state_publisher")

        self.declare_parameter("joint_state_topic", "joint_states")
        self.declare_parameter("publish_period_sec", 0.1)
        self.declare_parameter("joint_names", DEFAULT_ARM_JOINT_NAMES)
        self.declare_parameter("joint_positions", [0.0] * len(DEFAULT_ARM_JOINT_NAMES))

        self.joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self.publish_period_sec = float(self.get_parameter("publish_period_sec").value)
        self.joint_names = [str(name) for name in self.get_parameter("joint_names").value]
        self.joint_positions = [
            float(position)
            for position in self.get_parameter("joint_positions").value
        ]

        if self.publish_period_sec <= 0.0:
            raise ValueError("publish_period_sec must be positive")
        if not self.joint_names:
            raise ValueError("joint_names must not be empty")
        if len(self.joint_names) != len(self.joint_positions):
            raise ValueError("joint_names and joint_positions must have matching lengths")
        if len(set(self.joint_names)) != len(self.joint_names):
            raise ValueError("joint_names must be unique")

        self.publisher = self.create_publisher(
            JointState,
            self.joint_state_topic,
            10,
        )
        self.timer = self.create_timer(self.publish_period_sec, self.publish_joint_state)

        self.get_logger().info(
            "Publishing fallback arm joint states to %s for joints: %s"
            % (self.joint_state_topic, ", ".join(self.joint_names))
        )

    def publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_names)
        msg.position = list(self.joint_positions)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimArmJointStatePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
