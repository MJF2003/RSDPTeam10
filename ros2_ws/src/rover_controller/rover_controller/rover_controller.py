from enum import Enum, auto

import rclpy
from rclpy.action.client import ActionClient
from rclpy.node import Node

from rover_interface.action import ManipulateBlock, NavigateToPos
from rover_interface.msg import (
    BinPoseSmoothed,
    BinPoseSmoothedArray,
    BlockBinColor,
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
        PROPOSE_NAV_POSE = auto()  # TODO(Alex): Implement this logic
        EXECUTE_NAV = auto()

    class GraspBlock(Enum):
        ATTEMPT_GRASP = auto()
        RECOMPUTE_POSITION = auto()
        TRANSITION_TO_CARRY = auto()  # TODO(Alex): Can the manipulator node do this?

    class ApproachBin(Enum):
        PROPOSE_NAV_POSE = auto()
        EXECUTE_NAV = auto()

    class DepositBlock(Enum):
        ATTEMPT_DEPOSIT = auto()
        REGRASP_BLOCK = auto()  # TODO(Alex): TBD whether we need this

    class PostDeposit(Enum):
        TERMINATE = auto()


def color_to_str(color: BlockBinColor | int) -> str:
    "Converts BlockBinColor enums to strings"
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
        self.bins: list[BinPoseSmoothed] = []
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

        manip_action_topic = "/manipulate_block"
        self.get_logger().info(
            f"Connecting to manipulation action server {manip_action_topic=}"
        )
        self.manip_action_client = ActionClient(
            self,
            ManipulateBlock,
            manip_action_topic,
        )
        self.manip_goal_in_flight = False
        self.manip_goal_operation: int | None = None

        # Spin main controller loop
        controller_period = 0.1
        self.main_loop_timer = self.create_timer(controller_period, callback=self.loop)
        self.get_logger().info(
            f"Setting up main loop timer with frequency {1 / controller_period:.1f}Hz"
        )

        self.loop_count = 0  # var for controlling frequency of state logging

    def loop(self):
        self.loop_count += 1
        if self.loop_count % 50 == 0:  # logs generic status every 5s
            self.log_status()
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
            case Phase.DepositBlock.ATTEMPT_DEPOSIT:
                self.deposit_block()
            case Phase.PostDeposit.TERMINATE:
                pass
            case _:
                self.get_logger().warn(f"Unexpected state {self.state=}")

    def log_status(self):
        self.get_logger().info(
            f"Current phase: {self.state}\nKnown blocks: {self.blocks}\nKnown bins: {self.bins}"
        )

    def wait(self):
        "Waits until all expected topics and action servers are ready"
        for sub in self.subscriptions:
            if self.count_publishers(sub.topic_name) == 0:
                self.get_logger().info(f"Waiting for publisher on {sub.topic_name}...")
                return

        for attr in self.__dict__.values():
            if isinstance(attr, ActionClient):
                if not attr.server_is_ready():
                    # self.get_logger().info(
                    #     f"Waiting for action server {attr._action_name}..."
                    # )
                    return

        self.get_logger().info("All interfaces ready, progressing to bin observation.")
        self.state = Phase.StartUp.OBSERVE_BINS

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
        # Tbd whether these happen in the controller or the navigation node
        # ideally should be in nav node
        # self.get_logger().info("Beginning exploration!")

        # First - if we have a target block, move to the Approach Phase
        if self.target_block is not None:
            self.state = Phase.ApproachBlock.EXECUTE_NAV
            return
        # Otherwise - begin executing an ExploreMove
        # self.get_logger().info("Pretending to explore")
        return

    def block_pose_callback(self, msg: BlockPoseSmoothedArray):
        # don't pay attention if we're doing a grasp or deposition
        if (self.state in Phase.GraspBlock) or (self.state in Phase.DepositBlock):
            return
        # parse and store block poses
        self.blocks = msg.blocks  # type: ignore
        # if we have any uncollected blocks - set it as the target
        for block in self.blocks:
            if (block.id not in self.collected) and (self.target_block is None):
                self.get_logger().info(
                    f"Targeting {color_to_str(block.color)} block, ignoring collected blocks {self.collected=}"
                )
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
                "Navigation action server '/navigate_to_pos' is not available."
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
        # self.get_logger().info(f"Navigation feedback: distance_remaining={dist:.3f} m")

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
                    f"Succesfully navigated to {color_to_str(self.target_bin.color)} bin. Transitioning to phase {Phase.DepositBlock.ATTEMPT_DEPOSIT}"  # type: ignore
                )
                self.state = Phase.DepositBlock.ATTEMPT_DEPOSIT
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
        if target_block is None:
            self.get_logger().warn(f"Received invalid {target_block=}")
            self.state = Phase.Explore.EXPLORE
            return

        color = target_block.color
        self.send_manipulation_goal(
            operation=ManipulateBlock.Goal.GRASP,
            target=target_block,
            description=f"Grasping block with color: {color_to_str(color)}",
        )

    def select_target_bin(self, target_block: BlockPoseSmoothed | None):
        # TODO(Alex): Remove this assert
        assert target_block is not None
        color = target_block.color
        color_value = int(color.color)
        for bin in self.bins:
            if int(bin.color.color) != color_value:
                self.get_logger().info(
                    f"Ignoring bin {color_to_str(bin.color)} for targetting"
                )
                continue
            self.target_bin = bin
            self.state = Phase.ApproachBin.EXECUTE_NAV
            self.get_logger().info(
                f"Succesfully identified matching {color_to_str(color)} bin"
            )
            return
        self.get_logger().info(
            f"No matching bin known for block color {color_to_str(color)}"
        )

    def deposit_block(self):
        if self.target_block is None or self.target_bin is None:
            self.get_logger().warn(
                "Cannot deposit without both a target block and target bin."
            )
            self.state = Phase.Explore.EXPLORE
            return

        self.send_manipulation_goal(
            operation=ManipulateBlock.Goal.DEPOSIT,
            target=self.target_bin,
            description=(
                f"Depositing {color_to_str(self.target_block.color)} block in "
                f"{color_to_str(self.target_bin.color)} bin"
            ),
        )

    def send_manipulation_goal(
        self,
        operation: int,
        target: BlockPoseSmoothed | BinPoseSmoothed | None,
        description: str,
    ) -> bool:
        if self.manip_goal_in_flight or (target is None):
            return False

        if not self.manip_action_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn(
                "Manipulation action server '/manipulate_block' is not available."
            )
            return False

        goal = ManipulateBlock.Goal()
        goal.operation = operation
        goal.target_pos = target.position

        self.manip_goal_in_flight = True
        self.manip_goal_operation = operation
        self.get_logger().info(
            f"{description} at "
            f"({target.position.point.x:.2f}, {target.position.point.y:.2f})"
        )

        goal_future = self.manip_action_client.send_goal_async(
            goal,
            feedback_callback=self._manip_feedback_callback,
        )
        goal_future.add_done_callback(self._manip_goal_response_callback)
        return True

    def _manip_feedback_callback(self, feedback_msg):
        stage = feedback_msg.feedback.current_stage
        # self.get_logger().info(f"Manipulation feedback: current_stage='{stage}'")

    def _manip_goal_response_callback(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Manipulation goal was rejected.")
            self._handle_manipulation_result(success=False, message="Goal rejected.")
            return

        self.get_logger().info("Manipulation goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._manip_result_callback)

    def _manip_result_callback(self, future):
        result = future.result().result
        status = "SUCCESS" if result.success else "FAILURE"
        self.get_logger().info(
            f"Manipulation result {status}: message='{result.message}'"
        )
        self._handle_manipulation_result(success=result.success, message=result.message)

    def _handle_manipulation_result(self, success: bool, message: str):
        operation = self.manip_goal_operation
        self.manip_goal_in_flight = False
        self.manip_goal_operation = None

        if operation == ManipulateBlock.Goal.GRASP:
            if success:
                if self.target_block is None:
                    self.get_logger().warn(
                        "Grasp reported success but there is no target block."
                    )
                    self.state = Phase.Explore.EXPLORE
                    return

                self.collected.add(self.target_block.id)
                self.get_logger().info(f"Adding target block to {self.collected=}")
                self.state = Phase.ApproachBin.PROPOSE_NAV_POSE
                return

            self.get_logger().warning(
                f"Grasp failed with message '{message}'. Re-approaching block."
            )
            self.state = Phase.ApproachBlock.EXECUTE_NAV
            return

        if operation == ManipulateBlock.Goal.DEPOSIT:
            if success:
                self.target_block = None
                self.target_bin = None

                # if we have successfully collected 3 blocks, terminate the mission,
                # or return to the exploration phase to find more
                if len(self.collected) < 3:
                    self.state = Phase.Explore.EXPLORE
                else:
                    self.state = Phase.PostDeposit.TERMINATE
                return

            self.get_logger().warning(
                f"Deposit failed with message '{message}'. Re-approaching bin."
            )
            self.state = Phase.ApproachBin.EXECUTE_NAV
            return

        self.get_logger().warn(
            f"Manipulation completed with unknown operation {operation=}; "
            f"returning to explore."
        )
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
