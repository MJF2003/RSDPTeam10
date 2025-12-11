import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume, JointConstraint
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
import math
import time

def euler_to_quaternion(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def quaternion_to_euler(q):
    t0 = +2.0 * (q.w * q.x + q.y * q.z)
    t1 = +1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (q.w * q.y - q.z * q.x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (q.w * q.z + q.x * q.y)
    t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(t3, t4)
    return roll, pitch, yaw

class AutoPickNode(Node):
    # === 🛠️ 參數設定區 ===
    # 1. 速度設定 (範圍 0.1 ~ 1.0)
    # 建議：0.2(慢), 0.6(中), 1.0(極速)
    ARM_SPEED = 0.6   
    
    # 2. 夾爪設定
    GRIPPER_OPEN = 100
    GRIPPER_CLOSE = 20
    
    # 3. 幾何參數
    GRIPPER_LEN = 0.11
    HOVER_HEIGHT = 0.08
    # ====================

    def __init__(self):
        super().__init__('auto_pick_node')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self.end_effector_link = "joint6_flange" 
        self.base_frame = "g_base" 
        self.initial_joint_state = None 

        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.vision_sub = self.create_subscription(PoseStamped, '/vision/target_pose', self.vision_callback, 10)
        self.gripper_pub = self.create_publisher(Int32, '/mycobot/gripper_cmd', 10)
        
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0

        print(f"系統啟動！速度設定為: {self.ARM_SPEED * 100}%")
        self._action_client.wait_for_server()
        
        while self.initial_joint_state is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        
        print(">>> ✅ 已記錄起始位置！")
        self.control_gripper(self.GRIPPER_OPEN) 

    def joint_callback(self, msg):
        if self.initial_joint_state is None:
            self.initial_joint_state = msg

    def vision_callback(self, msg):
        if self.initial_joint_state is None: return
        
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y
        self.target_z = msg.pose.position.z
        
        print(f"\n[收到訊號] 目標: ({self.target_x:.2f}, {self.target_y:.2f}, {self.target_z:.2f})")
        print(">>> 🚀 啟動直上直下抓取流程...")
        
        self.move_to_pose(
            self.target_x, 
            self.target_y, 
            self.target_z + self.HOVER_HEIGHT,
            task_type="HOVER"
        )

    def control_gripper(self, value):
        msg = Int32()
        msg.data = value
        self.gripper_pub.publish(msg)

    def move_to_pose(self, x, y, z_raw, task_type):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'arm_group'
        goal_msg.request.num_planning_attempts = 20
        goal_msg.request.allowed_planning_time = 10.0
        goal_msg.request.start_state.is_diff = True
        
        # === 這裡使用了我們最上面定義的速度 ===
        goal_msg.request.max_velocity_scaling_factor = self.ARM_SPEED
        goal_msg.request.max_acceleration_scaling_factor = self.ARM_SPEED
        # ===================================
        
        grasp_depth = 0.01 
        real_z = z_raw + self.GRIPPER_LEN - grasp_depth
        target_orientation = euler_to_quaternion(0, 3.14, 0) 

        pc = PositionConstraint()
        pc.header.frame_id = self.base_frame
        pc.link_name = self.end_effector_link
        pc.constraint_region = BoundingVolume()
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.01]
        pc.constraint_region.primitives.append(primitive)
        
        target_point = Pose()
        target_point.position.x = x
        target_point.position.y = y
        target_point.position.z = real_z
        target_point.orientation = target_orientation
        
        pc.constraint_region.primitive_poses.append(target_point)
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header.frame_id = self.base_frame
        oc.link_name = self.end_effector_link
        oc.orientation = target_orientation
        oc.absolute_x_axis_tolerance = 0.1
        oc.absolute_y_axis_tolerance = 0.1
        oc.absolute_z_axis_tolerance = 0.1
        oc.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pc)
        constraints.orientation_constraints.append(oc)
        goal_msg.request.goal_constraints.append(constraints)

        print(f"規劃路徑 -> {task_type}")
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(lambda f: self.goal_response_callback(f, task_type))

    def goal_response_callback(self, future, task_type):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print(f'❌ {task_type} 規劃失敗')
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self.execution_done_callback(f, task_type))

    def execution_done_callback(self, future, task_type):
        if task_type == "HOVER":
            print(">>> ⬇️ 下降抓取...")
            self.move_to_pose(self.target_x, self.target_y, self.target_z, "GRASP")
        elif task_type == "GRASP":
            self.execute_pick_sequence()
        elif task_type == "LIFT_DONE":
            print(">>> 🔄 回家...")
            self.back_to_start()

    def execute_pick_sequence(self):
        print(f"   [夾爪] 閉合... (數值:{self.GRIPPER_CLOSE})")
        self.control_gripper(self.GRIPPER_CLOSE)
        time.sleep(1.5) # 速度變快了，等待時間也可以縮短一點點 (原本2.0)
        
        print(">>> ⬆️ 提起...")
        self.move_to_pose(
            self.target_x, 
            self.target_y, 
            self.target_z + self.HOVER_HEIGHT, 
            "LIFT_DONE"
        )

    def back_to_start(self):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'arm_group'
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.start_state.is_diff = True
        
        # === 回家也要加速 ===
        goal_msg.request.max_velocity_scaling_factor = self.ARM_SPEED
        goal_msg.request.max_acceleration_scaling_factor = self.ARM_SPEED

        constraints = Constraints()
        for i, name in enumerate(self.initial_joint_state.name):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = self.initial_joint_state.position[i]
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
            
        goal_msg.request.goal_constraints.append(constraints)
        
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.home_response_callback)

    def home_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted: return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.home_result_callback)

    def home_result_callback(self, future):
        print(">>> 🏠 已平安回到原點！放下物品...")
        self.control_gripper(self.GRIPPER_OPEN)
        print(">>> ✅ 任務完成！")

def main(args=None):
    rclpy.init(args=args)
    node = AutoPickNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()