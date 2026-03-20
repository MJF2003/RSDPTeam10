#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TracksMonitorAttrs(Node):
    def __init__(self):
        super().__init__("rsdp_tracks_monitor_attrs")
        #self.create_subscription(String, "/rsdp/tracks", self.cb, 10)
        self.create_subscription(String, "/cv/tracks_json", self.cb, 10)
        self.get_logger().info("Listening to /cv/tracks_json ...")

    def _fmt_attr(self, d, key):
        a = d.get(key, {}) or {}
        lab = a.get("label")
        conf = a.get("conf")
        votes = a.get("votes")
        if lab is None:
            return f"{key}=None(votes={votes})"
        return f"{key}={lab}({conf},v={votes})"

    def cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception:
            self.get_logger().warn("Bad JSON")
            return

        tracks = data.get("tracks", [])
        if not tracks:
            self.get_logger().info("No stable tracks.")
            return

        header = f"frame={data.get('frame_id','?')} fps={data.get('yolo_fps','?')} n={len(tracks)}"
        lines = [header]
        for t in tracks:
            xyz = t.get("xyz_m")
            lines.append(
                f"  id={t.get('id'):>2} {t.get('class'):<12} conf={t.get('conf'):<5} xyz={xyz} "
                f"{self._fmt_attr(t,'bin_color')} {self._fmt_attr(t,'block_color')} {self._fmt_attr(t,'block_shape')}"
            )
        self.get_logger().info("\n" + "\n".join(lines))

def main():
    rclpy.init()
    node = TracksMonitorAttrs()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()