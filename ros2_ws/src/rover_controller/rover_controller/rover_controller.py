# TODO: implement main controller - it needs to handle
# transitions ketween states in the main
from enum import Enum, auto

import rclpy
from rclpy.action.client import ActionClient
from rclpy.node import Node
from rover_interface.action import NavigateToBlock

from rover_interface.msg import (
    BlockBinColor,
    BlockPoseObservation,
    BlockPoseSmoothed,
    BlockPoseSmoothedArray,
    BlockShape,
)


class Phase:
    class StartUp(Enum):
        WAIT = auto()
        OBSERVE_BINS = auto()

    class Explore(Enum):
        EXPLORE = auto()

    class ApproachBlock(Enum):
        PROPOSE_NAV_POSE = auto()
        EXECUTE_NAV = auto()

    class GraspBlock(Enum):
        ATTEMPT_GRASP = auto()
        RECOMPUTE_POSITION = auto()
        TRANSITION_TO_CARRY = auto()

    class ApproachBin(Enum):
        PROPOSE_NAV_POSE = auto()
        EXECUTE_NAV = auto()

    class DepositBlock(Enum):
        ATTEMPT_DEPOSIT = auto()
        REGRASP_BLOCK = auto()

    class PostDeposit(Enum):
        TERMINATE = auto()


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller_node")
        self.state = Phase.StartUp.WAIT
        self.get_logger().info(f"Launching controller node with state {self.state=}")

        block_topic = "/controller/block_poses"
        # Observation Management
        self.obs_sub = self.create_subscription(
            BlockPoseSmoothedArray,
            topic=block_topic,
            callback=self.block_pose_callback,
            qos_profile=1,
        )
        self.get_logger().info(f"Starting subscription to {block_topic}")
        self.blocks = []
        self.bins = [1, 2, 3]  # TODO(alex) - just a dummy variable!!!
        self.target_block: None | BlockPoseSmoothed = None

        # Action client for navigation
        action_topic = "/navigate_to_block"
        self.get_logger().info(
            f"Connecting to navigation action server {action_topic=}"
        )
        self.nav_action_client = ActionClient(
            self,
            NavigateToBlock,
            action_topic,
        )
        self.nav_goal_in_flight = False

        # Spin main controller loop
        controller_period = 0.1
        self.main_loop_timer = self.create_timer(controller_period, callback=self.loop)
        self.get_logger().info(
            f"Setting up main loop timer with frequency {1 / controller_period:.1f}Hz"
        )

    def loop(self):
        match self.state:
            case Phase.StartUp.WAIT:
                self.wait()
            case Phase.StartUp.OBSERVE_BINS:
                self.observe_bins()
            case Phase.Explore.EXPLORE:
                self.explore()
            case Phase.ApproachBlock.EXECUTE_NAV:
                self.navigate_to_block(self.target_block)

    def wait(self):
        self.get_logger().info("I am waiting :)")
        # TODO: how do you check how many other nodes are up? expected
        # number of pubs/subs?
        # TODO: add a proper check here
        all_nodes_ready = True
        if all_nodes_ready:
            self.get_logger().info("All nodes ready, progressing to bin observation")
            self.state = Phase.StartUp.OBSERVE_BINS
        else:
            self.get_logger().warning("Some nodes still not ready")

    def observe_bins(self):
        self.get_logger().info("I am observing O.O")
        # Sends a movement command to Nav node - rotate 360 pls (can nav do that?)
        # worst case it rotates 180 and just does it twice
        # TODO: implement rotation action
        if len(self.bins) >= 3:
            self.get_logger().info("BINS OBSERVED!")
            self.state = Phase.Explore.EXPLORE
        else:
            self.get_logger().warning("DIDN'T SEE ALL BINS!")

    def explore(self):
        ...
        # Tbd whether these happen in the controller or the navigation node
        # ideally should be in nav node
        self.get_logger().info("Beginning exploration!")

        # First - if we have a target block, move to the Approach Phase
        if self.target_block is not None:
            self.state = Phase.ApproachBlock.EXECUTE_NAV
            return
        # Otherwise - begin executing an ExploreMove
        return

    def block_pose_callback(self, msg: BlockPoseSmoothedArray):
        # don't pay attention if we're doing a grasp or deposition
        if (self.state in Phase.GraspBlock) or (self.state in Phase.DepositBlock):
            return
        # parse and store block poses
        self.blocks = msg.blocks
        # if we have any uncollected blocks - set it as the target
        for block in self.blocks:
            if not block.collected:
                self.target_block = block

    def navigate_to_block(self, block: BlockPoseSmoothed | None) -> bool:
        """Send a navigation action goal for a specific block target."""
        # select a block to go to

        if self.nav_goal_in_flight or (block is None):
            return False

        if not self.nav_action_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn(
                "Navigation action server '/navigate_to_block' is not available."
            )
            return False

        goal = NavigateToBlock.Goal()
        goal.target_block = block

        self.nav_goal_in_flight = True
        self.get_logger().info(
            f"Sending navigation goal for block id={block.id} "
            f"at ({block.position.point.x:.2f}, {block.position.point.y:.2f})"
        )

        goal_future = self.nav_action_client.send_goal_async(
            goal,
            feedback_callback=self._nav_feedback_callback,
        )
        goal_future.add_done_callback(self._nav_goal_response_callback)
        return True

    def _nav_feedback_callback(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f"Navigation feedback: distance_remaining={dist:.3f} m")

    def _nav_goal_response_callback(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.nav_goal_in_flight = False
            self.get_logger().error("Navigation goal was rejected.")
            return

        self.get_logger().info("Navigation goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_callback)

    def _nav_result_callback(self, future):
        self.nav_goal_in_flight = False
        result = future.result().result
        if result.success:
            status = "SUCCESS"
            self.state = Phase
        status = "SUCCESS" if result.success else "FAILURE"
        self.get_logger().info(
            f"Navigation result {status}: "
            f"final_distance={result.final_distance:.3f} m, message='{result.message}'"
        )


def main():
    try:
        rclpy.init()
        node = ControllerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == "__main__":
    main()
