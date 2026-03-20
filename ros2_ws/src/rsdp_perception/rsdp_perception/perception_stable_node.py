#!/usr/bin/env python3
import json
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np
import torch

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseArray
from message_filters import Subscriber, ApproximateTimeSynchronizer


# -------------------- image decode (no cv_bridge) --------------------

def image_msg_to_numpy(msg: Image) -> np.ndarray:
    h, w = msg.height, msg.width
    enc = (msg.encoding or "").lower()
    data = msg.data

    if enc in ("rgb8", "bgr8"):
        arr = np.frombuffer(data, dtype=np.uint8)
        arr = arr.reshape((h, msg.step // 3, 3))[:, :w, :]
        return arr

    if enc in ("16uc1", "mono16"):
        arr = np.frombuffer(data, dtype=np.uint16)
        arr = arr.reshape((h, msg.step // 2))[:, :w]
        return arr

    if enc in ("32fc1",):
        arr = np.frombuffer(data, dtype=np.float32)
        arr = arr.reshape((h, msg.step // 4))[:, :w]
        return arr

    raise ValueError(f"Unsupported encoding: {msg.encoding}")


def median_depth_m(depth_u16: np.ndarray, u: int, v: int, depth_scale_m: float, window: int) -> Optional[float]:
    h, w = depth_u16.shape[:2]
    x0 = max(0, u - window // 2)
    x1 = min(w, u + window // 2)
    y0 = max(0, v - window // 2)
    y1 = min(h, v + window // 2)
    roi = depth_u16[y0:y1, x0:x1]
    vals = roi[roi > 0]
    if vals.size == 0:
        return None
    return float(np.median(vals)) * depth_scale_m


def deproject(u: float, v: float, z: float, fx: float, fy: float, cx: float, cy: float) -> Tuple[float, float, float]:
    X = (u - cx) / fx * z
    Y = (v - cy) / fy * z
    return X, Y, z


# -------------------- tracking + smoothing --------------------

@dataclass
class Det:
    cls: str
    conf: float
    bbox: Tuple[int, int, int, int]
    center_px: Tuple[int, int]
    xyz: Optional[Tuple[float, float, float]]  # None if no valid depth


@dataclass
class Track:
    track_id: int
    cls: str
    conf_ema: float
    xyz_ema: Optional[np.ndarray]  # (3,)
    last_center_px: Tuple[int, int]
    age: int
    missed: int
    last_stamp_sec: float


def ema(old: float, new: float, alpha: float) -> float:
    return alpha * new + (1.0 - alpha) * old


def ema_vec(old: np.ndarray, new: np.ndarray, alpha: float) -> np.ndarray:
    return alpha * new + (1.0 - alpha) * old


class PerceptionStableNode(Node):
    def __init__(self):
        super().__init__("rsdp_perception_stable")

        # Params
        self.declare_parameter("weights", "")
        self.declare_parameter("conf", 0.25)
        self.declare_parameter("imgsz", 640)

        self.declare_parameter("depth_scale_m", 0.001)
        self.declare_parameter("depth_window", 20)
        self.declare_parameter("max_depth_m", 3.0)

        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("info_topic", "/camera/camera/color/camera_info")

        # Stabilization params
        self.declare_parameter("ema_alpha_xyz", 0.35)         # higher = more responsive, lower = smoother
        self.declare_parameter("ema_alpha_conf", 0.50)
        self.declare_parameter("assoc_max_dist_m", 0.10)      # associate detections to tracks if within 10 cm
        self.declare_parameter("max_missed_frames", 7)        # keep track alive for N missed frames
        self.declare_parameter("min_conf_to_publish", 0.35)   # publish only stable detections above this

        # Publish rate limiting
        self.declare_parameter("publish_every_n_frames", 1)

        weights = self.get_parameter("weights").value
        if not weights:
            raise RuntimeError("Set -p weights:=/path/to/best.pt")

        self.conf_th = float(self.get_parameter("conf").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.depth_scale_m = float(self.get_parameter("depth_scale_m").value)
        self.depth_window = int(self.get_parameter("depth_window").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)

        self.ema_alpha_xyz = float(self.get_parameter("ema_alpha_xyz").value)
        self.ema_alpha_conf = float(self.get_parameter("ema_alpha_conf").value)
        self.assoc_max_dist_m = float(self.get_parameter("assoc_max_dist_m").value)
        self.max_missed = int(self.get_parameter("max_missed_frames").value)
        self.min_conf_publish = float(self.get_parameter("min_conf_to_publish").value)
        self.publish_every = int(self.get_parameter("publish_every_n_frames").value)

        self.color_topic = self.get_parameter("color_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.info_topic = self.get_parameter("info_topic").value

        # Intrinsics
        self.fx = self.fy = self.cx = self.cy = None

        # YOLO
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Loading YOLOv5 on {self.device}: {weights}")
        self.model = torch.hub.load("ultralytics/yolov5", "custom", path=weights)
        self.model.to(self.device).eval()
        self.model.conf = self.conf_th
        self.names = self.model.names
        self.get_logger().info(f"Classes: {self.names}")

        # Tracks
        self.next_id = 1
        self.tracks: Dict[int, Track] = {}

        # Publishers
        self.pub_tracks = self.create_publisher(String, "/rsdp/tracks", 10)

        self.pub_blocks = self.create_publisher(PoseArray, "/rsdp/blocks/poses", 10)
        self.pub_bins = self.create_publisher(PoseArray, "/rsdp/bins/poses", 10)
        self.pub_openings = self.create_publisher(PoseArray, "/rsdp/bin_openings/poses", 10)
        self.pub_platforms = self.create_publisher(PoseArray, "/rsdp/platforms/poses", 10)

        # Sync subs
        self.sub_color = Subscriber(self, Image, self.color_topic)
        self.sub_depth = Subscriber(self, Image, self.depth_topic)
        self.sub_info = Subscriber(self, CameraInfo, self.info_topic)
        self.ts = ApproximateTimeSynchronizer([self.sub_color, self.sub_depth, self.sub_info], queue_size=10, slop=0.10)
        self.ts.registerCallback(self.cb)

        self.frame_count = 0
        self.get_logger().info("Stable perception node started.")

    def _class_name(self, cls_id: int) -> str:
        if isinstance(self.names, dict):
            return self.names.get(cls_id, str(cls_id))
        return self.names[cls_id]

    def cb(self, color_msg: Image, depth_msg: Image, info_msg: CameraInfo):
        self.frame_count += 1

        # Cache intrinsics
        if self.fx is None:
            self.fx = info_msg.k[0]
            self.fy = info_msg.k[4]
            self.cx = info_msg.k[2]
            self.cy = info_msg.k[5]
            self.get_logger().info(f"Intrinsics fx={self.fx:.2f} fy={self.fy:.2f} cx={self.cx:.2f} cy={self.cy:.2f}")

        # Decode
        try:
            color = image_msg_to_numpy(color_msg)
            depth = image_msg_to_numpy(depth_msg)
        except Exception as e:
            self.get_logger().warn(f"Decode failed: {e}")
            return

        # RGB -> BGR if needed
        if (color_msg.encoding or "").lower() == "rgb8":
            color = color[:, :, ::-1].copy()

        # Depth float->u16 if needed
        if depth.dtype == np.float32:
            depth_u16 = (depth / self.depth_scale_m).astype(np.uint16)
        else:
            depth_u16 = depth

        # Inference
        t0 = time.time()
        res = self.model(color, size=self.imgsz)
        det = res.xyxy[0].cpu().numpy()  # x1 y1 x2 y2 conf cls
        fps = 1.0 / max(1e-3, time.time() - t0)

        frame_id = color_msg.header.frame_id or "camera_color_optical_frame"

        # Build detections
        detections: List[Det] = []
        for (x1, y1, x2, y2, conf, cls_id) in det:
            cls_id = int(cls_id)
            cls = self._class_name(cls_id)
            x1i, y1i, x2i, y2i = map(int, [x1, y1, x2, y2])
            w = max(1, x2i - x1i)
            h = max(1, y2i - y1i)
            u = x1i + w // 2
            v = y1i + h // 2

            z = median_depth_m(depth_u16, u, v, self.depth_scale_m, self.depth_window)
            xyz = None
            if z is not None and 0.0 < z < self.max_depth_m:
                X, Y, Z = deproject(u, v, z, self.fx, self.fy, self.cx, self.cy)
                xyz = (float(X), float(Y), float(Z))

            detections.append(Det(
                cls=cls,
                conf=float(conf),
                bbox=(x1i, y1i, x2i, y2i),
                center_px=(u, v),
                xyz=xyz
            ))

        # Update tracks
        now = float(color_msg.header.stamp.sec) + float(color_msg.header.stamp.nanosec) * 1e-9
        self._update_tracks(detections, now)

        # Publish at chosen frame stride
        if self.publish_every > 1 and (self.frame_count % self.publish_every != 0):
            return

        self._publish(frame_id, color_msg, fps)

    def _update_tracks(self, dets: List[Det], now: float):
        # Mark all tracks as missed by default; we will reset those we match
        for tid, tr in self.tracks.items():
            tr.age += 1
            tr.missed += 1

        # Build candidate matches using XYZ when available; fallback to 2D center otherwise
        # We match within each class separately to avoid class swapping.
        dets_by_cls: Dict[str, List[Det]] = {}
        for d in dets:
            dets_by_cls.setdefault(d.cls, []).append(d)

        for cls, cls_dets in dets_by_cls.items():
            # Existing tracks of that class
            cls_tracks = [tr for tr in self.tracks.values() if tr.cls == cls]

            # Greedy nearest-neighbor assignment based on XYZ (preferred)
            used_dets = set()

            for tr in cls_tracks:
                best_j = None
                best_dist = 1e9

                for j, d in enumerate(cls_dets):
                    if j in used_dets:
                        continue

                    # distance metric
                    dist = None
                    if tr.xyz_ema is not None and d.xyz is not None:
                        dv = tr.xyz_ema - np.array(d.xyz, dtype=np.float32)
                        dist = float(np.linalg.norm(dv))
                        if dist > self.assoc_max_dist_m:
                            continue
                    else:
                        # fallback 2D pixel distance if depth missing
                        du = tr.last_center_px[0] - d.center_px[0]
                        dv = tr.last_center_px[1] - d.center_px[1]
                        dist = float(np.hypot(du, dv)) / 1000.0  # scale pixels->~meters-ish
                        # allow bigger for 2D fallback
                        if dist > (self.assoc_max_dist_m * 3.0):
                            continue

                    if dist < best_dist:
                        best_dist = dist
                        best_j = j

                if best_j is not None:
                    d = cls_dets[best_j]
                    used_dets.add(best_j)
                    self._apply_det_to_track(tr, d, now)

            # Create new tracks for unmatched dets
            for j, d in enumerate(cls_dets):
                if j in used_dets:
                    continue
                self._create_track(d, now)

        # Prune dead tracks
        dead = [tid for tid, tr in self.tracks.items() if tr.missed > self.max_missed]
        for tid in dead:
            del self.tracks[tid]

    def _apply_det_to_track(self, tr: Track, d: Det, now: float):
        tr.missed = 0
        tr.last_center_px = d.center_px
        tr.last_stamp_sec = now
        tr.conf_ema = ema(tr.conf_ema, d.conf, self.ema_alpha_conf)

        if d.xyz is not None:
            v = np.array(d.xyz, dtype=np.float32)
            if tr.xyz_ema is None:
                tr.xyz_ema = v
            else:
                tr.xyz_ema = ema_vec(tr.xyz_ema, v, self.ema_alpha_xyz)

    def _create_track(self, d: Det, now: float):
        tr = Track(
            track_id=self.next_id,
            cls=d.cls,
            conf_ema=d.conf,
            xyz_ema=(np.array(d.xyz, dtype=np.float32) if d.xyz is not None else None),
            last_center_px=d.center_px,
            age=1,
            missed=0,
            last_stamp_sec=now,
        )
        self.tracks[self.next_id] = tr
        self.next_id += 1

    def _pose_from_xyz(self, xyz: np.ndarray) -> Pose:
        p = Pose()
        p.position.x = float(xyz[0])
        p.position.y = float(xyz[1])
        p.position.z = float(xyz[2])
        p.orientation.w = 1.0
        return p

    def _publish(self, frame_id: str, color_msg: Image, fps: float):
        # Build PoseArrays from stable tracks
        blocks = PoseArray(); blocks.header = color_msg.header; blocks.header.frame_id = frame_id
        bins = PoseArray(); bins.header = color_msg.header; bins.header.frame_id = frame_id
        opens = PoseArray(); opens.header = color_msg.header; opens.header.frame_id = frame_id
        plats = PoseArray(); plats.header = color_msg.header; plats.header.frame_id = frame_id

        tracks_out = []
        for tid, tr in sorted(self.tracks.items(), key=lambda kv: kv[0]):
            # Publish only if we have XYZ and confidence is stable enough
            if tr.xyz_ema is None:
                continue
            if tr.conf_ema < self.min_conf_publish:
                continue

            xyz = tr.xyz_ema
            pose = self._pose_from_xyz(xyz)

            if tr.cls == "block":
                blocks.poses.append(pose)
            elif tr.cls == "bin":
                bins.poses.append(pose)
            elif tr.cls == "bin_opening":
                opens.poses.append(pose)
            elif tr.cls == "platform":
                plats.poses.append(pose)

            tracks_out.append({
                "id": tr.track_id,
                "class": tr.cls,
                "conf": round(float(tr.conf_ema), 3),
                "xyz_m": [round(float(x), 4) for x in xyz.tolist()],
                "age": tr.age,
                "missed": tr.missed,
                "center_px": [int(tr.last_center_px[0]), int(tr.last_center_px[1])],
            })

        self.pub_blocks.publish(blocks)
        self.pub_bins.publish(bins)
        self.pub_openings.publish(opens)
        self.pub_platforms.publish(plats)

        # Compact JSON status
        msg = String()
        msg.data = json.dumps({
            "stamp": {"sec": int(color_msg.header.stamp.sec), "nanosec": int(color_msg.header.stamp.nanosec)},
            "frame_id": frame_id,
            "yolo_fps": round(float(fps), 2),
            "tracks": tracks_out,
        })
        self.pub_tracks.publish(msg)

        # Useful periodic log
        if (self.frame_count % 20) == 0:
            self.get_logger().info(f"pub tracks={len(tracks_out)}  yolo_fps~{fps:.1f}")


def main():
    rclpy.init()
    node = PerceptionStableNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()