#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TracksMonitor(Node):
    def __init__(self):
        super().__init__("rsdp_tracks_monitor")
        self.create_subscription(String, "/rsdp/tracks", self.cb, 10)
        self.get_logger().info("Listening to /rsdp/tracks ...")

    def cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception:
            self.get_logger().warn("Received non-JSON message")
            return

        tracks = data.get("tracks", [])
        if not tracks:
            self.get_logger().info("No stable tracks.")
            return

        # Print a compact table-like summary
        lines = [f"frame={data.get('frame_id','?')}  fps={data.get('yolo_fps','?')}  n={len(tracks)}"]
        for t in tracks:
            xyz = t.get("xyz_m")
            lines.append(
                f"  id={t.get('id'):>2}  {t.get('class'):<11} conf={t.get('conf'):<5}  xyz={xyz}  missed={t.get('missed')}"
            )
        self.get_logger().info("\n" + "\n".join(lines))

def main():
    rclpy.init()
    node = TracksMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()