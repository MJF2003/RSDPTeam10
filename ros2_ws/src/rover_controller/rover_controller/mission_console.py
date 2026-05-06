import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.action.client import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger

from rover_interface.action import Explore as ExploreAction
from rover_interface.action import ManipulateBlock


class MissionConsole(Node):
    def __init__(self):
        super().__init__("mission_console")
        self.start_client = self.create_client(Trigger, "/controller/start_mission")
        self.stop_client = self.create_client(Trigger, "/controller/stop_mission")

        self.explore_action_topic = str(
            self.declare_parameter("explore_action_name", "/explore").value
        )
        self.explore_goal_timeout_sec = float(
            self.declare_parameter("explore_goal_timeout_sec", 20.0).value
        )
        self.explore_action_client = ActionClient(
            self,
            ExploreAction,
            self.explore_action_topic,
        )

        self.manip_action_topic = str(
            self.declare_parameter("manipulate_action_name", "/manipulate_block").value
        )
        self.manual_grasp_target_frame = str(
            self.declare_parameter("manual_grasp_target_frame", "map").value
        )
        self.manual_grasp_target_x = float(
            self.declare_parameter("manual_grasp_target_x", 0.0).value
        )
        self.manual_grasp_target_y = float(
            self.declare_parameter("manual_grasp_target_y", 0.0).value
        )
        self.manual_grasp_target_z = float(
            self.declare_parameter("manual_grasp_target_z", 0.0).value
        )
        self.manip_action_client = ActionClient(
            self,
            ManipulateBlock,
            self.manip_action_topic,
        )

        self.start_future = None
        self.stop_future = None
        self.explore_goal_in_flight = False
        self.explore_goal_handle = None
        self.grasp_goal_in_flight = False
        self.grasp_goal_handle = None
        self.grasp_feedback_stage = None

    def wait_for_services(self):
        waiting_logged_at = 0.0
        while rclpy.ok():
            start_ready = self.start_client.service_is_ready()
            stop_ready = self.stop_client.service_is_ready()
            explore_ready = self.explore_action_client.server_is_ready()
            manip_ready = self.manip_action_client.server_is_ready()
            if start_ready and stop_ready and explore_ready and manip_ready:
                print("Controller mission services and action servers are ready.")
                return True

            now = time.monotonic()
            if now - waiting_logged_at >= 1.0:
                waiting_logged_at = now
                missing = []
                if not start_ready:
                    missing.append("/controller/start_mission")
                if not stop_ready:
                    missing.append("/controller/stop_mission")
                if not explore_ready:
                    missing.append(self.explore_action_topic)
                if not manip_ready:
                    missing.append(self.manip_action_topic)
                print(f"Waiting for controller interfaces: {', '.join(missing)}")

            rclpy.spin_once(self, timeout_sec=0.1)

        return False

    def request_start(self):
        if self.start_future is not None and not self.start_future.done():
            print("Start request already in flight.")
            return

        self.start_future = self.start_client.call_async(Trigger.Request())
        print("Start requested.")

    def request_stop(self):
        if self.stop_future is not None and not self.stop_future.done():
            print("Stop request already in flight.")
            return

        self.stop_future = self.stop_client.call_async(Trigger.Request())
        print("Stop requested.")

    def request_explore(self):
        if self.explore_goal_in_flight:
            print("Explore request already in flight.")
            return

        if not self.explore_action_client.wait_for_server(timeout_sec=0.5):
            print(
                "Explore action server is not available: "
                f"{self.explore_action_topic}"
            )
            return

        goal = ExploreAction.Goal()
        goal.max_runtime_sec = self.explore_goal_timeout_sec
        goal.run_until_cancelled = False

        self.explore_goal_in_flight = True
        print(f"Explore requested for {self.explore_goal_timeout_sec:.1f}s.")
        goal_future = self.explore_action_client.send_goal_async(goal)
        goal_future.add_done_callback(self._explore_goal_response_callback)

    def request_grasp(self):
        if self.grasp_goal_in_flight:
            print("Grasp request already in flight.")
            return

        if not self.manip_action_client.wait_for_server(timeout_sec=0.5):
            print(
                "Manipulation action server is not available: "
                f"{self.manip_action_topic}"
            )
            return

        goal = ManipulateBlock.Goal()
        goal.operation = ManipulateBlock.Goal.GRASP
        goal.target_pos = self._manual_grasp_target()

        self.grasp_goal_in_flight = True
        self.grasp_feedback_stage = None
        print("Grasp requested.")
        goal_future = self.manip_action_client.send_goal_async(
            goal,
            feedback_callback=self._grasp_feedback_callback,
        )
        goal_future.add_done_callback(self._grasp_goal_response_callback)

    def pump_responses(self):
        self.start_future = self._pump_response(self.start_future, "start")
        self.stop_future = self._pump_response(self.stop_future, "stop")

    def _pump_response(self, future, label):
        if future is None or not future.done():
            return future

        try:
            response = future.result()
        except Exception as exc:
            print(f"{label.capitalize()} service call failed: {exc}")
            return None

        status = "accepted" if response.success else "rejected"
        print(f"{label.capitalize()} {status}: {response.message}")
        return None

    def _manual_grasp_target(self):
        target = PointStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = self.manual_grasp_target_frame
        target.point.x = self.manual_grasp_target_x
        target.point.y = self.manual_grasp_target_y
        target.point.z = self.manual_grasp_target_z
        return target

    def _explore_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.explore_goal_in_flight = False
            self.explore_goal_handle = None
            print(f"Explore action call failed: {exc}")
            return

        if goal_handle is None or not goal_handle.accepted:
            self.explore_goal_in_flight = False
            self.explore_goal_handle = None
            print("Explore goal rejected.")
            return

        self.explore_goal_handle = goal_handle
        print("Explore goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._explore_result_callback)

    def _explore_result_callback(self, future):
        try:
            result = future.result().result
        except Exception as exc:
            print(f"Explore result failed: {exc}")
        else:
            status = "SUCCESS" if result.success else "FAILURE"
            print(
                f"Explore result {status}: message='{result.message}', "
                f"elapsed={result.elapsed_time_sec:.1f}s, "
                f"final_state='{result.final_state}'"
            )
        finally:
            self.explore_goal_in_flight = False
            self.explore_goal_handle = None

    def _grasp_feedback_callback(self, feedback_msg):
        stage = feedback_msg.feedback.current_stage
        if stage and stage != self.grasp_feedback_stage:
            self.grasp_feedback_stage = stage
            print(f"Grasp feedback: current_stage='{stage}'")

    def _grasp_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.grasp_goal_in_flight = False
            self.grasp_goal_handle = None
            print(f"Grasp action call failed: {exc}")
            return

        if goal_handle is None or not goal_handle.accepted:
            self.grasp_goal_in_flight = False
            self.grasp_goal_handle = None
            print("Grasp goal rejected.")
            return

        self.grasp_goal_handle = goal_handle
        print("Grasp goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._grasp_result_callback)

    def _grasp_result_callback(self, future):
        try:
            result = future.result().result
        except Exception as exc:
            print(f"Grasp result failed: {exc}")
        else:
            status = "SUCCESS" if result.success else "FAILURE"
            print(f"Grasp result {status}: message='{result.message}'")
        finally:
            self.grasp_goal_in_flight = False
            self.grasp_goal_handle = None
            self.grasp_feedback_stage = None


def print_help():
    print()
    print("Mission Console")
    print("Enter  start mission")
    print("Space  stop mission")
    print("e      explore action")
    print("g      grasp action")
    print("h      help")
    print("q      quit console")
    print("Ctrl-C quit console")
    print()


def read_key():
    readable, _, _ = select.select([sys.stdin], [], [], 0.1)
    if not readable:
        return None

    return sys.stdin.read(1)


def run_console(node: MissionConsole):
    print_help()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.0)
        node.pump_responses()

        key = read_key()
        if key is None:
            continue

        if key in ("\n", "\r"):
            node.request_start()
        elif key == " ":
            node.request_stop()
        elif key.lower() == "e":
            node.request_explore()
        elif key.lower() == "g":
            node.request_grasp()
        elif key.lower() == "h":
            print_help()
        elif key.lower() == "q":
            print("Exiting mission console.")
            return
        elif key == "\x03":
            raise KeyboardInterrupt


def main(args=None):
    if not sys.stdin.isatty():
        print("mission_console requires an interactive TTY.", file=sys.stderr)
        return 1

    rclpy.init(args=args)
    node = MissionConsole()
    original_terminal_settings = termios.tcgetattr(sys.stdin)

    try:
        tty.setcbreak(sys.stdin.fileno())
        if node.wait_for_services():
            run_console(node)
    except KeyboardInterrupt:
        print()
        print("Exiting mission console.")
    finally:
        termios.tcsetattr(
            sys.stdin,
            termios.TCSADRAIN,
            original_terminal_settings,
        )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
