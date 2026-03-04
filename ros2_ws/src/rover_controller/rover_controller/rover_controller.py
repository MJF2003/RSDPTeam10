# TODO: implement main controller - it needs to handle
# transitions ketween states in the main
from enum import Enum, auto

import rclpy
from rclpy.action.client import ActionClient
from rclpy.node import Node

from rover_interface.action import NavigateToPos
from rover_interface.msg import (
    BinPoseSmoothed,
    BinPoseSmoothedArray,
    BlockBinColor,
    BlockPoseObservation,
    BlockPoseSmoothed,
    BlockPoseSmoothedArray,
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


def color_to_str(color: BlockBinColor | int) -> str:
    color_value = int(color.color) if isinstance(color, BlockBinColor) else int(color)
    return {
        BlockBinColor.BLUE: "BLUE",
        BlockBinColor.RED: "RED",
        BlockBinColor.YELLOW: "YELLOW",
        BlockBinColor.PURPLE: "PURPLE",
        BlockBinColor.PINK: "PINK",
        BlockBinColor.GREEN: "GREEN",
    }.get(color_value, f"UNKNOWN({color_value})")


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

        bin_topic = "/controller/bin_poses"
        self.bin_obs_sub = self.create_subscription(
            BinPoseSmoothedArray,
            topic=bin_topic,
            callback=self.bin_pose_callback,
            qos_profile=1,
        )
        self.get_logger().info(f"Starting subscription to {bin_topic}")

        self.blocks: list[BlockPoseSmoothed] = []
        self.collected: set[int] = set()  # stores ids of collected blocks
        self.bins: list[BinPoseSmoothed] = []  # TODO(alex) - just a dummy variable!!!
        self.target_block: None | BlockPoseSmoothed = None
        self.target_bin: None | BinPoseSmoothed = None

        # Action client for navigation
        action_topic = "/navigate_to_pos"
        self.get_logger().info(
            f"Connecting to navigation action server {action_topic=}"
        )
        self.nav_action_client = ActionClient(
            self,
            NavigateToPos,
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
                self.navigate_to_pose(self.target_block)
            case Phase.GraspBlock.ATTEMPT_GRASP:
                self.grasp_block(self.target_block)
            case Phase.ApproachBin.PROPOSE_NAV_POSE:
                self.select_target_bin(self.target_block)
            case Phase.ApproachBin.EXECUTE_NAV:
                self.navigate_to_pose(self.target_bin)

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
        # TODO(Alex): implement rotation action
        # TODO(Alex): Remove the True check
        if len(self.bins) >= 3 or True:
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
        self.get_logger().info("Pretending to explore")
        return

    def block_pose_callback(self, msg: BlockPoseSmoothedArray):
        # don't pay attention if we're doing a grasp or deposition
        if (self.state in Phase.GraspBlock) or (self.state in Phase.DepositBlock):
            return
        # parse and store block poses
        self.blocks = msg.blocks  # type: ignore
        # if we have any uncollected blocks - set it as the target
        for block in self.blocks:
            if block.id not in self.collected:
                self.target_block = block

    def bin_pose_callback(self, msg: BinPoseSmoothedArray):
        # don't pay attention if we're doing a grasp or deposition
        if (self.state in Phase.GraspBlock) or (self.state in Phase.DepositBlock):
            return
        # parse and store bin poses
        self.bins = msg.bins  # type: ignore

    # ------------------------ NAVIGATION -----------------------------------------
    def navigate_to_pose(
        self, target: BlockPoseSmoothed | BinPoseSmoothed | None
    ) -> bool:
        """Send a navigation action goal for a specific block target."""
        # select a block to go to

        if self.nav_goal_in_flight or (target is None):
            # this check for block is None is purely for typing. It can't be None
            # when this function is called (I think)
            return False

        if not self.nav_action_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn(
                "Navigation action server '/navigate_to_block' is not available."
            )
            return False

        goal = NavigateToPos.Goal()
        goal.target_pos = target.position

        self.nav_goal_in_flight = True
        # TODO(Alex) Make this specific to block/bin
        self.get_logger().info(
            f"Sending navigation goal for target "
            f"at ({target.position.point.x:.2f}, {target.position.point.y:.2f})"
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
            if self.state in Phase.ApproachBlock:
                self.get_logger().info(
                    f"Succesful navigation. Transitioning to phase {Phase.GraspBlock.ATTEMPT_GRASP}"
                )
                self.state = Phase.GraspBlock.ATTEMPT_GRASP
            elif self.state in Phase.ApproachBin:
                self.get_logger().info(
                    f"Succesful navigation. Transitioning to phase {Phase.GraspBlock.ATTEMPT_GRASP}"
                )
                self.state = Phase.Explore.EXPLORE
            else:
                self.get_logger().warn(
                    f"Unexpected state at end of navigation: {self.state=}"
                )
        # TODO(Alex): what do we do if the navigation failed?
        status = "SUCCESS" if result.success else "FAILURE"
        self.get_logger().info(
            f"Navigation result {status}: "
            f"final_distance={result.final_distance:.3f} m, message='{result.message}'"
        )

    # ------------------------ MANIPULATION -----------------------------------------
    def grasp_block(self, target_block: BlockPoseSmoothed | None):
        # basic fake logic - mark the target_block acquired, set phase
        # to ApproachBin.
        # TODO(Alex): Add proper action server
        if target_block is None:
            self.get_logger().warn(f"Received invalid {target_block=}")
            self.state = Phase.Explore.EXPLORE
            return

        # TODO(Alex): This block targeting logic should maybe not be in this func
        self.collected.add(target_block.id)
        self.state = Phase.ApproachBin.PROPOSE_NAV_POSE

    def select_target_bin(self, target_block: BlockPoseSmoothed | None):
        # TODO(Alex): Remove this assert
        assert target_block is not None
        color = target_block.color
        color_value = int(color.color)
        self.get_logger().info(f"Grasping block with color: {color_to_str(color)}")
        for bin in self.bins:
            if int(bin.color.color) != color_value:
                self.get_logger().info(
                    f"Ignoring bin {color_to_str(bin.color)} for grasp"
                )
                continue
            self.target_bin = bin
            self.state = Phase.ApproachBin.EXECUTE_NAV
            self.get_logger().info(f"Succesfully grasped {color_to_str(color)} block")
            return
        self.get_logger().info(
            f"No matching bin known for block color {color_to_str(color)}"
        )

    def deposit_block(self):
        self.target_block = None
        self.target_bin = None
        self.get_logger().info("Depositing block")
        self.state = Phase.Explore.EXPLORE


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
