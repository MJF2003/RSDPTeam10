#!/usr/bin/env python3
import json
import time
from typing import Optional, Tuple

import numpy as np
import cv2
import torch

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

# vision_msgs is optional; if missing, node still works (publishes JSON)
try:
    from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
    from geometry_msgs.msg import Pose
    VISION_MSGS_OK = True
except Exception:
    VISION_MSGS_OK = False

from message_filters import Subscriber, ApproximateTimeSynchronizer


def median_depth_m(depth_u16: np.ndarray, cx: int, cy: int, depth_scale_m: float, window: int = 10) -> Optional[float]:
    """Median depth in meters around (cx, cy) from aligned depth image (uint16)."""
    h, w = depth_u16.shape[:2]
    x0 = max(0, cx - window // 2)
    x1 = min(w, cx + window // 2)
    y0 = max(0, cy - window // 2)
    y1 = min(h, cy + window // 2)
    roi = depth_u16[y0:y1, x0:x1]
    vals = roi[roi > 0]
    if vals.size == 0:
        return None
    z_u = float(np.median(vals))
    return z_u * depth_scale_m


def deproject(u: float, v: float, z: float, fx: float, fy: float, cx: float, cy: float) -> Tuple[float, float, float]:
    """Pinhole deprojection: pixel (u,v) with depth z -> (X,Y,Z) in camera optical frame."""
    X = (u - cx) / fx * z
    Y = (v - cy) / fy * z
    return X, Y, z


class YoloV5RealsenseNode(Node):
    def __init__(self):
        super().__init__("yolov5_realsense_node")

        # ---- Parameters ----
        self.declare_parameter("weights", "")
        self.declare_parameter("conf", 0.25)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("depth_scale_m", 0.001)  # default for RealSense, verify with /camera topic if needed
        self.declare_parameter("depth_window", 10)
        self.declare_parameter("max_depth_m", 2.0)
        self.declare_parameter("publish_debug_image", True)

        self.declare_parameter("color_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("info_topic", "/camera/color/camera_info")

        weights = self.get_parameter("weights").get_parameter_value().string_value
        if not weights:
            raise RuntimeError("Parameter 'weights' is empty. Set it to your YOLOv5 .pt path.")

        self.conf = float(self.get_parameter("conf").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.depth_scale_m = float(self.get_parameter("depth_scale_m").value)
        self.depth_window = int(self.get_parameter("depth_window").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)
        self.publish_debug = bool(self.get_parameter("publish_debug_image").value)

        color_topic = self.get_parameter("color_topic").value
        depth_topic = self.get_parameter("depth_topic").value
        info_topic = self.get_parameter("info_topic").value

        # ---- Publishers ----
        self.pub_json = self.create_publisher(String, "/rsdp/detections_json", 10)
        self.pub_debug = self.create_publisher(Image, "/rsdp/debug_image", 10) if self.publish_debug else None

        if VISION_MSGS_OK:
            self.pub_det3d = self.create_publisher(Detection3DArray, "/rsdp/detections_3d", 10)
        else:
            self.pub_det3d = None
            self.get_logger().warn("vision_msgs not available; will publish JSON only.")

        self.bridge = CvBridge()

        # ---- Camera intrinsics (from CameraInfo) ----
        self.fx = self.fy = self.cx = self.cy = None

        # ---- Load YOLOv5 (local repo) ----
        device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Loading YOLOv5 weights on {device}: {weights}")
        # Use local repo copy if you run inside yolov5 folder; otherwise it will still work via torch.hub if cached.
        # For robustness in ROS2, we use source='local' with a repo path parameter if you want later.
        self.model = torch.hub.load("ultralytics/yolov5", "custom", path=weights)
        self.model.to(device).eval()
        self.model.conf = self.conf
        self.names = self.model.names  # class map
        self.device = device

        self.get_logger().info(f"YOLO classes: {self.names}")

        # ---- Subscriptions with time sync ----
        self.sub_color = Subscriber(self, Image, color_topic)
        self.sub_depth = Subscriber(self, Image, depth_topic)
        self.sub_info = Subscriber(self, CameraInfo, info_topic)

        self.ts = ApproximateTimeSynchronizer(
            [self.sub_color, self.sub_depth, self.sub_info],
            queue_size=10,
            slop=0.10,
        )
        self.ts.registerCallback(self.callback)

        self.get_logger().info("Node started. Waiting for messages...")

    def callback(self, color_msg: Image, depth_msg: Image, info_msg: CameraInfo):
        # ---- intrinsics ----
        if self.fx is None:
            # CameraInfo.K = [fx,0,cx, 0,fy,cy, 0,0,1]
            self.fx = info_msg.k[0]
            self.fy = info_msg.k[4]
            self.cx = info_msg.k[2]
            self.cy = info_msg.k[5]
            self.get_logger().info(f"Got intrinsics fx={self.fx:.2f} fy={self.fy:.2f} cx={self.cx:.2f} cy={self.cy:.2f}")

        # ---- decode images ----
        color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")

        # depth: RealSense aligned depth is usually 16UC1 (uint16)
        # cv_bridge returns np.uint16 array
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        if depth.dtype != np.uint16:
            # Sometimes depth comes as 32FC1; handle if needed
            if depth.dtype == np.float32:
                # convert meters->uint16-like units using depth_scale_m (best-effort)
                depth_u16 = (depth / self.depth_scale_m).astype(np.uint16)
                depth = depth_u16
            else:
                self.get_logger().warn(f"Unexpected depth dtype={depth.dtype}, skipping frame")
                return

        # ---- YOLO inference ----
        t0 = time.time()
        results = self.model(color, size=self.imgsz)
        det = results.xyxy[0].cpu().numpy()  # x1,y1,x2,y2,conf,cls
        fps = 1.0 / max(1e-3, time.time() - t0)

        # ---- build outputs ----
        out_list = []

        # vision_msgs output
        det3d_msg = None
        if self.pub_det3d is not None:
            det3d_msg = Detection3DArray()
            det3d_msg.header = color_msg.header  # same timestamp/frame
            det3d_msg.detections = []

        debug = color.copy() if self.publish_debug else None

        for (x1, y1, x2, y2, conf, cls_id) in det:
            cls_id = int(cls_id)
            cls_name = self.names.get(cls_id, str(cls_id)) if isinstance(self.names, dict) else self.names[cls_id]
            x1i, y1i, x2i, y2i = map(int, [x1, y1, x2, y2])
            w = max(1, x2i - x1i)
            h = max(1, y2i - y1i)
            u = x1i + w // 2
            v = y1i + h // 2

            z = median_depth_m(depth, u, v, self.depth_scale_m, window=self.depth_window)
            X = Y = Z = None
            if z is not None and 0.0 < z < self.max_depth_m:
                X, Y, Z = deproject(u, v, z, self.fx, self.fy, self.cx, self.cy)

            item = {
                "class": cls_name,
                "conf": float(conf),
                "bbox": [x1i, y1i, x2i, y2i],
                "center_px": [int(u), int(v)],
                "frame_id": color_msg.header.frame_id or "camera_color_optical_frame",
                "xyz_m": [X, Y, Z] if X is not None else None,
            }
            out_list.append(item)

            # vision_msgs
            if det3d_msg is not None:
                d3 = Detection3D()
                d3.header = det3d_msg.header

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = str(cls_name)
                hyp.hypothesis.score = float(conf)

                pose = Pose()
                if X is not None:
                    pose.position.x = float(X)
                    pose.position.y = float(Y)
                    pose.position.z = float(Z)
                hyp.pose.pose = pose

                d3.results.append(hyp)
                det3d_msg.detections.append(d3)

            # debug image
            if debug is not None:
                cv2.rectangle(debug, (x1i, y1i), (x2i, y2i), (0, 255, 0), 2)
                label = f"{cls_name} {conf:.2f}"
                if X is not None:
                    label += f" ({X:.2f},{Y:.2f},{Z:.2f})m"
                cv2.putText(debug, label, (x1i, max(0, y1i - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA)

        # Add fps
        if debug is not None:
            cv2.putText(debug, f"FPS ~ {fps:.1f}", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)

        # ---- publish JSON ----
        msg = String()
        msg.data = json.dumps({
            "header": {
                "stamp": {"sec": int(color_msg.header.stamp.sec), "nanosec": int(color_msg.header.stamp.nanosec)},
                "frame_id": color_msg.header.frame_id,
            },
            "detections": out_list,
        })
        self.pub_json.publish(msg)

        # ---- publish Detection3DArray ----
        if det3d_msg is not None:
            self.pub_det3d.publish(det3d_msg)

        # ---- publish debug image ----
        if debug is not None and self.pub_debug is not None:
            out_img = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
            out_img.header = color_msg.header
            self.pub_debug.publish(out_img)


def main():
    rclpy.init()
    node = YoloV5RealsenseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
