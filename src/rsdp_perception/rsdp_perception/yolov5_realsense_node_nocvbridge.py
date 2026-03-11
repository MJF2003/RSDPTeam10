#!/usr/bin/env python3
import json
import time
from typing import Optional, Tuple

import numpy as np
import torch

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo

from message_filters import Subscriber, ApproximateTimeSynchronizer


def image_msg_to_numpy(msg: Image) -> np.ndarray:
    """
    Convert sensor_msgs/Image to numpy without cv_bridge.
    Supports common encodings from realsense2_camera:
      - color: rgb8, bgr8
      - depth: 16UC1
    """
    h, w = msg.height, msg.width
    enc = (msg.encoding or "").lower()

    # raw bytes
    data = msg.data  # bytes-like
    if enc in ("rgb8", "bgr8"):
        # 3 channels uint8
        arr = np.frombuffer(data, dtype=np.uint8)
        # step is bytes per row; if step == w*3 typical
        arr = arr.reshape((h, msg.step // 3, 3))[:, :w, :]
        return arr

    if enc in ("mono8", "8uc1"):
        arr = np.frombuffer(data, dtype=np.uint8).reshape((h, msg.step))[:, :w]
        return arr

    if enc in ("16uc1", "mono16"):
        arr = np.frombuffer(data, dtype=np.uint16).reshape((h, msg.step // 2))[:, :w]
        return arr

    if enc in ("32fc1",):
        arr = np.frombuffer(data, dtype=np.float32).reshape((h, msg.step // 4))[:, :w]
        return arr

    raise ValueError(f"Unsupported encoding: {msg.encoding}")


def median_depth_m_from_u16(depth_u16: np.ndarray, u: int, v: int, depth_scale_m: float, window: int = 10) -> Optional[float]:
    """Median depth in meters around (u,v) from aligned depth (uint16)."""
    h, w = depth_u16.shape[:2]
    x0 = max(0, u - window // 2)
    x1 = min(w, u + window // 2)
    y0 = max(0, v - window // 2)
    y1 = min(h, v + window // 2)
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


class YoloV5RealsenseNoCvBridge(Node):
    def __init__(self):
        super().__init__("yolov5_realsense_node_nocvbridge")

        # Parameters
        self.declare_parameter("weights", "")
        self.declare_parameter("conf", 0.25)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("depth_scale_m", 0.001)  # D435i often 0.001
        self.declare_parameter("depth_window", 10)
        self.declare_parameter("max_depth_m", 2.0)
        self.declare_parameter("print_every_n", 10)  # print every N frames

        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("info_topic", "/camera/camera/color/camera_info")

        weights = self.get_parameter("weights").value
        if not weights:
            raise RuntimeError("Parameter 'weights' is empty. Set it to your YOLOv5 .pt path.")

        self.conf = float(self.get_parameter("conf").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.depth_scale_m = float(self.get_parameter("depth_scale_m").value)
        self.depth_window = int(self.get_parameter("depth_window").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)
        self.print_every_n = int(self.get_parameter("print_every_n").value)

        color_topic = self.get_parameter("color_topic").value
        depth_topic = self.get_parameter("depth_topic").value
        info_topic = self.get_parameter("info_topic").value

        # Publisher (JSON)
        self.pub_json = self.create_publisher(String, "/rsdp/detections_json", 10)

        # Intrinsics (from CameraInfo)
        self.fx = self.fy = self.cx = self.cy = None

        # Load YOLOv5
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Loading YOLOv5 weights on {self.device}: {weights}")

        # Use torch.hub. This may download/cache ultralytics/yolov5 code.
        self.model = torch.hub.load("ultralytics/yolov5", "custom", path=weights)
        self.model.to(self.device).eval()
        self.model.conf = self.conf

        self.names = self.model.names
        self.get_logger().info(f"YOLO classes: {self.names}")

        # Sync subscribers (color + depth + camera_info)
        self.sub_color = Subscriber(self, Image, color_topic)
        self.sub_depth = Subscriber(self, Image, depth_topic)
        self.sub_info = Subscriber(self, CameraInfo, info_topic)

        self.ts = ApproximateTimeSynchronizer(
            [self.sub_color, self.sub_depth, self.sub_info],
            queue_size=10,
            slop=0.10,
        )
        self.ts.registerCallback(self.callback)

        self.frame_count = 0
        self.get_logger().info("Node started. Waiting for messages...")

    def callback(self, color_msg: Image, depth_msg: Image, info_msg: CameraInfo):
        self.frame_count += 1

        # Intrinsics: store once
        if self.fx is None:
            self.fx = info_msg.k[0]
            self.fy = info_msg.k[4]
            self.cx = info_msg.k[2]
            self.cy = info_msg.k[5]
            self.get_logger().info(
                f"Got intrinsics fx={self.fx:.2f} fy={self.fy:.2f} cx={self.cx:.2f} cy={self.cy:.2f}"
            )

        # Convert messages to numpy
        try:
            color = image_msg_to_numpy(color_msg)  # rgb8 or bgr8 array
            depth = image_msg_to_numpy(depth_msg)  # uint16 array
        except Exception as e:
            self.get_logger().warn(f"Image decode failed: {e}")
            return

        # Ensure color is BGR for YOLO (it generally works with RGB too, but keep consistent)
        if (color_msg.encoding or "").lower() == "rgb8":
            # convert RGB->BGR
            color = color[:, :, ::-1].copy()

        # Depth may come as float32 meters, but aligned depth from realsense2_camera is usually 16UC1
        if depth.dtype == np.float32:
            # meters -> u16-like using depth_scale_m
            depth = (depth / self.depth_scale_m).astype(np.uint16)

        # YOLO inference
        t0 = time.time()
        results = self.model(color, size=self.imgsz)
        det = results.xyxy[0].cpu().numpy()  # x1,y1,x2,y2,conf,cls
        fps = 1.0 / max(1e-3, time.time() - t0)

        detections = []
        frame_id = color_msg.header.frame_id or "camera_color_optical_frame"

        for (x1, y1, x2, y2, conf, cls_id) in det:
            cls_id = int(cls_id)
            cls_name = self.names.get(cls_id, str(cls_id)) if isinstance(self.names, dict) else self.names[cls_id]

            x1i, y1i, x2i, y2i = map(int, [x1, y1, x2, y2])
            w = max(1, x2i - x1i)
            h = max(1, y2i - y1i)
            u = x1i + w // 2
            v = y1i + h // 2

            z = median_depth_m_from_u16(depth, u, v, self.depth_scale_m, window=self.depth_window)

            X = Y = Z = None
            if z is not None and 0.0 < z < self.max_depth_m:
                X, Y, Z = deproject(u, v, z, self.fx, self.fy, self.cx, self.cy)

            detections.append({
                "class": cls_name,
                "conf": float(conf),
                "bbox": [x1i, y1i, x2i, y2i],
                "center_px": [int(u), int(v)],
                "frame_id": frame_id,
                "xyz_m": [X, Y, Z] if X is not None else None,
            })

        # Publish JSON
        msg = String()
        msg.data = json.dumps({
            "header": {
                "stamp": {"sec": int(color_msg.header.stamp.sec), "nanosec": int(color_msg.header.stamp.nanosec)},
                "frame_id": frame_id,
                "fps_yolo": fps,
            },
            "detections": detections,
        })
        self.pub_json.publish(msg)

        # Print occasionally (so you can debug without any GUI)
        if self.print_every_n > 0 and (self.frame_count % self.print_every_n == 0):
            self.get_logger().info(f"Frame {self.frame_count}  YOLO_FPS~{fps:.1f}  det={len(detections)}")
            for d in detections[:10]:
                self.get_logger().info(
                    f"  {d['class']:12s} conf={d['conf']:.2f} xyz={d['xyz_m']}"
                )


def main():
    rclpy.init()
    node = YoloV5RealsenseNoCvBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()