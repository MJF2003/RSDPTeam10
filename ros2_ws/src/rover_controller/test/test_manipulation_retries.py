from types import SimpleNamespace

from geometry_msgs.msg import PointStamped

from rover_controller.rover_controller import ControllerNode, Phase
from rover_interface.action import ManipulateBlock


class FakeLogger:
    def info(self, *_args, **_kwargs):
        pass

    def warn(self, *_args, **_kwargs):
        pass

    def warning(self, *_args, **_kwargs):
        pass


def make_point(x=1.0, y=2.0, frame_id="map"):
    point = PointStamped()
    point.header.frame_id = frame_id
    point.point.x = x
    point.point.y = y
    return point


def make_block(block_id=1):
    return SimpleNamespace(id=block_id, position=make_point())


def make_controller():
    controller = ControllerNode.__new__(ControllerNode)
    controller.state = Phase.Explore.EXPLORE
    controller.mission_stopped = False
    controller.manip_goal_in_flight = False
    controller.manip_goal_operation = None
    controller.manip_goal_handle = None
    controller.manip_cancel_in_flight = False
    controller.manipulation_max_attempts = 5
    controller.manipulation_failure_counts = {
        ManipulateBlock.Goal.GRASP: 0,
        ManipulateBlock.Goal.DEPOSIT: 0,
    }
    controller.manipulation_retry_target_block_id = None
    controller.target_block = None
    controller.target_bin = None
    controller.collected = set()
    controller.fixed_keepout_positions = {}
    controller.get_logger = lambda: FakeLogger()
    return controller


def handle_result(controller, operation, success=False, message="failed"):
    controller.manip_goal_operation = operation
    controller._handle_manipulation_result(success=success, message=message)


def test_grasp_failure_below_limit_reapproaches_block():
    controller = make_controller()
    block = make_block()
    controller.set_target_block(block)

    for _ in range(4):
        handle_result(controller, ManipulateBlock.Goal.GRASP)

    assert controller.state == Phase.ApproachBlock.EXECUTE_NAV
    assert controller.collected == set()
    assert controller.manipulation_failure_counts[ManipulateBlock.Goal.GRASP] == 4


def test_fifth_grasp_failure_is_treated_as_success():
    controller = make_controller()
    block = make_block()
    controller.set_target_block(block)

    for _ in range(5):
        handle_result(controller, ManipulateBlock.Goal.GRASP)

    assert controller.state == Phase.ApproachBin.PROPOSE_NAV_POSE
    assert controller.collected == {block.id}
    assert block.id in controller.fixed_keepout_positions
    assert controller.manipulation_failure_counts[ManipulateBlock.Goal.GRASP] == 0


def test_deposit_failure_below_limit_reapproaches_bin():
    controller = make_controller()
    controller.set_target_block(make_block())
    controller.target_bin = SimpleNamespace(position=make_point())

    for _ in range(4):
        handle_result(controller, ManipulateBlock.Goal.DEPOSIT)

    assert controller.state == Phase.ApproachBin.EXECUTE_NAV
    assert controller.target_block is not None
    assert controller.target_bin is not None
    assert controller.manipulation_failure_counts[ManipulateBlock.Goal.DEPOSIT] == 4


def test_fifth_deposit_failure_is_treated_as_success_and_explores():
    controller = make_controller()
    controller.collected = {1}
    controller.set_target_block(make_block())
    controller.target_bin = SimpleNamespace(position=make_point())

    for _ in range(5):
        handle_result(controller, ManipulateBlock.Goal.DEPOSIT)

    assert controller.state == Phase.Explore.EXPLORE
    assert controller.target_block is None
    assert controller.target_bin is None
    assert controller.manipulation_failure_counts[ManipulateBlock.Goal.DEPOSIT] == 0


def test_fifth_deposit_failure_terminates_after_third_collected_block():
    controller = make_controller()
    controller.collected = {1, 2, 3}
    controller.set_target_block(make_block(3))
    controller.target_bin = SimpleNamespace(position=make_point())

    for _ in range(5):
        handle_result(controller, ManipulateBlock.Goal.DEPOSIT)

    assert controller.state == Phase.PostDeposit.TERMINATE
    assert controller.target_block is None
    assert controller.target_bin is None


def test_manipulation_failure_counts_reset_for_new_target_block():
    controller = make_controller()
    controller.set_target_block(make_block(1))
    handle_result(controller, ManipulateBlock.Goal.GRASP)

    controller.set_target_block(make_block(2))

    assert controller.manipulation_failure_counts == {
        ManipulateBlock.Goal.GRASP: 0,
        ManipulateBlock.Goal.DEPOSIT: 0,
    }
    assert controller.manipulation_retry_target_block_id == 2
