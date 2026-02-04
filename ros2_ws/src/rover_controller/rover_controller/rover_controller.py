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
        self.state = Phase.StartUp.WAIT

    def loop(self):
        match self.state:
            case Phase.StartUp.WAIT:
                self.wait()
            case Phase.StartUp.OBSERVE_BINS:
                self.observe_bins()

    def wait(self): ...

    def observe_bins(self): ...
