import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class MissionConsole(Node):
    def __init__(self):
        super().__init__("mission_console")
        self.start_client = self.create_client(Trigger, "/controller/start_mission")
        self.stop_client = self.create_client(Trigger, "/controller/stop_mission")
        self.start_future = None
        self.stop_future = None

    def wait_for_services(self):
        waiting_logged_at = 0.0
        while rclpy.ok():
            start_ready = self.start_client.service_is_ready()
            stop_ready = self.stop_client.service_is_ready()
            if start_ready and stop_ready:
                print("Controller mission services are ready.")
                return True

            now = time.monotonic()
            if now - waiting_logged_at >= 1.0:
                waiting_logged_at = now
                missing = []
                if not start_ready:
                    missing.append("/controller/start_mission")
                if not stop_ready:
                    missing.append("/controller/stop_mission")
                print(f"Waiting for controller services: {', '.join(missing)}")

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


def print_help():
    print()
    print("Mission Console")
    print("Enter  start mission")
    print("Space  stop mission")
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
