# TODO: implement main controller - it needs to handle
# transitions ketween states in the main

# question - how are we going to do this? Everywhere the mission
# plan can halt is basically a state which needs to be maintained
from enum import Enum, auto

import rclpy
from rclpy.node import Node

from rover_interface.msg import (
    BlockBinColor,
    BlockPoseSmoothed,
    BlockPoseSmoothedArray,
    BlockShape,
)


class Phase:
    class StartUp(Enum):
        WAIT = auto()
        OBSERVE_BINS = auto()

    class Explore(Enum):
        PROPOSE_NAV_POSE = auto()
        EXECUTE_MOVE = auto()

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
        self.state = Phase.Explore.PROPOSE_NAV_POSE

    def loop(self):
        match self.state:
            case Phase.StartUp.WAIT:
                self.wait()
            case Phase.StartUp.OBSERVE_BINS:
                self.observe_bins()
            case Phase.Explore.PROPOSE_NAV_POSE:
                self.propose_exploration_pose()
            case Phase.Explore.EXECUTE_MOVE:
                self.execute_exploration_move()

    def wait(self):
        self.get_logger().info("I am waiting :)")
        # TODO: how do you check how many other nodes are up? expected
        # number of pubs/subs?

    def observe_bins(self):
        self.get_logger().info("I am observing O.O")
        # Sends a movement command to Nav node - rotate 360 pls (can nav do that?)
        # worst case it rotates 180 and just does it twice

    def propose_exploration_pose(self): ...
    def execute_exploration_move(self):
        ...
        # Tbd whether these happen in the controller or the navigation node
        # ideally should be in nav node
