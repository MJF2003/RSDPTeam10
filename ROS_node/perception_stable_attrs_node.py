#!/usr/bin/env python3
import json
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple
from collections import deque, Counter

import numpy as np
import torch

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseArray
from message_filters import Subscriber, ApproximateTimeSynchronizer

import joblib
import torch.nn as nn
from torchvision import transforms, models
from PIL import Image as PILImage

def image_msg_to_numpy(msg):
    """
    Decode ROS2 sensor_msgs/Image into numpy without cv_bridge.

    Supports:
      - rgb8 / bgr8  -> uint8 HxWx3
      - 16UC1 / mono16 -> uint16 HxW (depth)
      - 32FC1 -> float32 HxW
    """
    h, w = msg.height, msg.width
    enc = (msg.encoding or "").lower()
    data = msg.data

    if enc in ("rgb8", "bgr8"):
        arr = np.frombuffer(data, dtype=np.uint8)
        # msg.step is bytes per row; step/3 is pixels per row for 3 channels
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

# -------------------- utility helpers --------------------

def softmax_np(x: np.ndarray) -> np.ndarray:
    x = np.asarray(x, dtype=np.float32)
    x = x - np.max(x)
    e = np.exp(x)
    return e / (np.sum(e) + 1e-9)


def clamp(v: int, lo: int, hi: int) -> int:
    return lo if v < lo else hi if v > hi else v


def crop_bgr(img_bgr: np.ndarray, bbox: Tuple[int, int, int, int], pad_frac: float = 0.08) -> np.ndarray:
    """Crop bbox from BGR image with optional padding fraction."""
    h, w = img_bgr.shape[:2]
    x1, y1, x2, y2 = bbox
    bw = max(1, x2 - x1)
    bh = max(1, y2 - y1)

    pad_x = int(round(bw * pad_frac))
    pad_y = int(round(bh * pad_frac))

    x1p = clamp(x1 - pad_x, 0, w - 1)
    y1p = clamp(y1 - pad_y, 0, h - 1)
    x2p = clamp(x2 + pad_x, 1, w)
    y2p = clamp(y2 + pad_y, 1, h)

    if x2p <= x1p + 1 or y2p <= y1p + 1:
        return img_bgr[max(0, y1):min(h, y2), max(0, x1):min(w, x2)].copy()

    return img_bgr[y1p:y2p, x1p:x2p].copy()


def deproject(u: int, v: int, z_m: float, fx: float, fy: float, cx: float, cy: float) -> Tuple[float, float, float]:
    """
    Pinhole deprojection from pixel (u,v) with depth z (meters) to camera XYZ (meters),
    in the camera optical frame.
    """
    x = (float(u) - float(cx)) * float(z_m) / float(fx)
    y = (float(v) - float(cy)) * float(z_m) / float(fy)
    return x, y, float(z_m)


def median_depth_m(depth_u16: np.ndarray, u: int, v: int, depth_scale_m: float, window: int = 20) -> Optional[float]:
    """
    Robust depth estimate around (u,v) using median of valid depths in a square window.
    depth_u16 is typical aligned depth image in uint16 (depth units).
    depth_scale_m converts raw units to meters (e.g. 0.001 for mm).
    """
    if depth_u16 is None:
        return None
    h, w = depth_u16.shape[:2]
    if u < 0 or v < 0 or u >= w or v >= h:
        return None

    r = max(1, int(window // 2))
    x1 = max(0, u - r)
    x2 = min(w, u + r + 1)
    y1 = max(0, v - r)
    y2 = min(h, v + r + 1)

    patch = depth_u16[y1:y2, x1:x2]
    if patch.size == 0:
        return None

    # valid: >0
    valid = patch[patch > 0]
    if valid.size == 0:
        return None

    med = float(np.median(valid.astype(np.float32)))
    z_m = med * float(depth_scale_m)
    if not np.isfinite(z_m) or z_m <= 0:
        return None
    return z_m
# ---------- classifier models (MATCH infer_live_realsense_final.py) ----------

class BlockAttrNet(nn.Module):
    """ResNet18 backbone with 2 heads: color + shape (multi-task)."""
    def __init__(self, n_colors: int, n_shapes: int):
        super().__init__()
        backbone = models.resnet18(weights=None)  # weights don't matter; we load state_dict next
        in_feat = backbone.fc.in_features
        backbone.fc = nn.Identity()
        self.backbone = backbone
        self.fc_color = nn.Linear(in_feat, n_colors)
        self.fc_shape = nn.Linear(in_feat, n_shapes)

    def forward(self, x):
        feat = self.backbone(x)
        return self.fc_color(feat), self.fc_shape(feat)


class BinColorNet(nn.Module):
    """ResNet18 backbone with 1 head: color only."""
    def __init__(self, n_colors: int):
        super().__init__()
        backbone = models.resnet18(weights=None)
        in_feat = backbone.fc.in_features
        backbone.fc = nn.Identity()
        self.backbone = backbone
        self.fc = nn.Linear(in_feat, n_colors)

    def forward(self, x):
        feat = self.backbone(x)
        return self.fc(feat)


def build_tfm():
    return transforms.Compose([
        transforms.Resize((224, 224)),
        transforms.ToTensor(),
        transforms.Normalize(
            mean=[0.485, 0.456, 0.406],
            std=[0.229, 0.224, 0.225],
        ),
    ])


class BinColorHead:
    """Single-head classifier -> bin_color (MATCH infer_live_realsense_final.py)"""
    def __init__(self, clf_dir: Path, device: str):
        self.device = device
        self.tfm = build_tfm()

        clf_dir = Path(clf_dir)
        enc = joblib.load(clf_dir / "label_encoder.pkl")
        # your script uses: bin_encoders["color_encoder"]
        self.color_enc = enc["color_encoder"]
        self.colors = list(self.color_enc.classes_)

        self.model = BinColorNet(len(self.colors))
        sd = torch.load(clf_dir / "best.pt", map_location=device)
        self.model.load_state_dict(sd, strict=True)
        self.model.to(device).eval()

        print(f"[bin_color] colors={self.colors}")

    @torch.no_grad()
    def predict(self, crop_bgr: np.ndarray) -> Tuple[str, float, Dict[str,float]]:
        # BGR -> RGB PIL
        rgb = crop_bgr[:, :, ::-1]
        pil = PILImage.fromarray(rgb)
        x = self.tfm(pil).unsqueeze(0).to(self.device)

        logits = self.model(x).squeeze(0).detach().cpu().float().numpy()
        probs = softmax_np(logits)
        idx = int(np.argmax(probs))
        label = self.colors[idx]
        conf = float(probs[idx])
        prob_dict = {self.colors[i]: float(probs[i]) for i in range(len(self.colors))}
        return label, conf, prob_dict


class BlockAttrsHead:
    """Two-head multitask classifier -> block_color + block_shape (MATCH infer_live_realsense_final.py)"""
    def __init__(self, clf_dir: Path, device: str):
        self.device = device
        self.tfm = build_tfm()

        clf_dir = Path(clf_dir)
        encs = joblib.load(clf_dir / "label_encoders.pkl")
        # your script uses: ["color_enc"], ["shape_enc"]
        self.color_enc = encs["color_enc"]
        self.shape_enc = encs["shape_enc"]
        self.colors = list(self.color_enc.classes_)
        self.shapes = list(self.shape_enc.classes_)

        self.model = BlockAttrNet(len(self.colors), len(self.shapes))
        sd = torch.load(clf_dir / "best.pt", map_location=device)
        self.model.load_state_dict(sd, strict=True)
        self.model.to(device).eval()

        print(f"[block_attrs] colors={self.colors} shapes={self.shapes}")

    @torch.no_grad()
    def predict(self, crop_bgr: np.ndarray):
        rgb = crop_bgr[:, :, ::-1]
        pil = PILImage.fromarray(rgb)
        x = self.tfm(pil).unsqueeze(0).to(self.device)

        logits_c, logits_s = self.model(x)
        c = logits_c.squeeze(0).detach().cpu().float().numpy()
        s = logits_s.squeeze(0).detach().cpu().float().numpy()

        cp = softmax_np(c)
        sp = softmax_np(s)

        ci = int(np.argmax(cp))
        si = int(np.argmax(sp))

        color = self.colors[ci]
        shape = self.shapes[si]
        cconf = float(cp[ci])
        sconf = float(sp[si])

        cdict = {self.colors[i]: float(cp[i]) for i in range(len(self.colors))}
        sdict = {self.shapes[i]: float(sp[i]) for i in range(len(self.shapes))}
        return color, cconf, cdict, shape, sconf, sdict
# -------------------- detection / track data --------------------

@dataclass
class Det:
    cls: str
    conf: float
    bbox: Tuple[int, int, int, int]
    center_px: Tuple[int, int]
    xyz: Optional[Tuple[float, float, float]]


@dataclass
class AttrVote:
    labels: deque = field(default_factory=lambda: deque(maxlen=10))
    confs: deque = field(default_factory=lambda: deque(maxlen=10))
    probs: deque = field(default_factory=lambda: deque(maxlen=10))  # list of dicts

    def push(self, label: str, conf: float, prob_dict: Dict[str,float]):
        self.labels.append(label)
        self.confs.append(conf)
        self.probs.append(prob_dict)

    def mode(self) -> Optional[str]:
        if not self.labels:
            return None
        return Counter(self.labels).most_common(1)[0][0]

    def mean_conf(self) -> Optional[float]:
        if not self.confs:
            return None
        return float(np.mean(np.array(self.confs, dtype=np.float32)))

    def mean_probs(self) -> Dict[str, float]:
        # average probability dicts over window
        if not self.probs:
            return {}
        keys = set()
        for d in self.probs:
            keys.update(d.keys())
        out = {}
        for k in keys:
            out[k] = float(np.mean([d.get(k, 0.0) for d in self.probs]))
        return out


@dataclass
class Track:
    track_id: int
    cls: str
    conf_ema: float
    xyz_ema: Optional[np.ndarray]
    last_center_px: Tuple[int, int]
    age: int
    missed: int
    last_stamp_sec: float

    # attributes w/ temporal voting
    bin_color: AttrVote = field(default_factory=AttrVote)
    block_color: AttrVote = field(default_factory=AttrVote)
    block_shape: AttrVote = field(default_factory=AttrVote)


def ema(old: float, new: float, alpha: float) -> float:
    return alpha * new + (1.0 - alpha) * old


def ema_vec(old: np.ndarray, new: np.ndarray, alpha: float) -> np.ndarray:
    return alpha * new + (1.0 - alpha) * old


# -------------------- main node --------------------

class PerceptionStableAttrsNode(Node):
    def __init__(self):
        super().__init__("rsdp_perception_stable_attrs")

        self.attr_ok = 0
        self.attr_fail = 0
        self._last_attr_warn = 0.0

        # YOLO params
        self.declare_parameter("weights", "")
        self.declare_parameter("conf", 0.25)
        self.declare_parameter("imgsz", 640)

        # Depth params
        self.declare_parameter("depth_scale_m", 0.001)
        self.declare_parameter("depth_window", 20)
        self.declare_parameter("max_depth_m", 3.0)

        # Topics
        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("info_topic", "/camera/camera/color/camera_info")

        # Tracking / smoothing
        self.declare_parameter("ema_alpha_xyz", 0.35)
        self.declare_parameter("ema_alpha_conf", 0.50)
        self.declare_parameter("assoc_max_dist_m", 0.12)
        self.declare_parameter("max_missed_frames", 7)
        self.declare_parameter("min_conf_to_publish", 0.35)

        # Attribute voting
        self.declare_parameter("vote_window", 10)
        self.declare_parameter("min_votes_to_output", 3)
        self.declare_parameter("min_attr_conf", 0.40)

        # Classifier dirs (NEW)
        self.declare_parameter("block_attrs_dir", "")  # multitask: block_color + block_shape
        self.declare_parameter("bin_color_dir", "")    # single: bin_color

        # Classifier preprocessing
        self.declare_parameter("clf_input_size", 224)
        self.declare_parameter("clf_mean", [0.485, 0.456, 0.406])
        self.declare_parameter("clf_std",  [0.229, 0.224, 0.225])
        self.declare_parameter("crop_pad_frac", 0.08)

        # Publish stride
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

        self.vote_window = int(self.get_parameter("vote_window").value)
        self.min_votes = int(self.get_parameter("min_votes_to_output").value)
        self.min_attr_conf = float(self.get_parameter("min_attr_conf").value)

        self.publish_every = int(self.get_parameter("publish_every_n_frames").value)

        self.color_topic = self.get_parameter("color_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.info_topic = self.get_parameter("info_topic").value

        self.clf_input_size = int(self.get_parameter("clf_input_size").value)
        mean = self.get_parameter("clf_mean").value
        std = self.get_parameter("clf_std").value
        self.clf_mean = tuple(float(x) for x in mean)
        self.clf_std = tuple(float(x) for x in std)
        self.crop_pad_frac = float(self.get_parameter("crop_pad_frac").value)

        # camera intrinsics
        self.fx = self.fy = self.cx = self.cy = None

        # YOLO
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Loading YOLOv5 on {self.device}: {weights}")
        self.model = torch.hub.load("ultralytics/yolov5", "custom", path=weights)
        self.model.to(self.device).eval()
        self.model.conf = self.conf_th
        self.names = self.model.names
        self.get_logger().info(f"YOLO classes: {self.names}")

        # Classifiers (optional)
        self.block_attrs = None
        self.bin_color = None

        block_attrs_dir = str(self.get_parameter("block_attrs_dir").value).strip()
        bin_color_dir = str(self.get_parameter("bin_color_dir").value).strip()

        if block_attrs_dir:
            try:
                self.block_attrs = BlockAttrsHead(Path(block_attrs_dir), self.device)
                self.get_logger().info(f"Loaded block attrs classifier from {block_attrs_dir}")
            except Exception as e:
                self.get_logger().warn(f"Failed to load block attrs classifier from {block_attrs_dir}: {e}")

        if bin_color_dir:
            try:
                self.bin_color = BinColorHead(Path(bin_color_dir), self.device)
                self.get_logger().info(f"Loaded bin color classifier from {bin_color_dir}")
            except Exception as e:
                self.get_logger().warn(f"Failed to load bin color classifier from {bin_color_dir}: {e}")

        # Tracks
        self.next_id = 1
        self.tracks: Dict[int, Track] = {}

        # Publishers
        #self.pub_tracks = self.create_publisher(String, "/rsdp/tracks", 10)
        #self.pub_blocks = self.create_publisher(PoseArray, "/rsdp/blocks/poses", 10)
        #self.pub_bins = self.create_publisher(PoseArray, "/rsdp/bins/poses", 10)
        #self.pub_openings = self.create_publisher(PoseArray, "/rsdp/bin_openings/poses", 10)
        #self.pub_platforms = self.create_publisher(PoseArray, "/rsdp/platforms/poses", 10)

        self.pub_tracks = self.create_publisher(String, "/cv/tracks_json", 10)
        self.pub_blocks = self.create_publisher(PoseArray, "/cv/block_poses", 10)
        self.pub_bins = self.create_publisher(PoseArray, "/cv/bin_poses", 10)
        self.pub_openings = self.create_publisher(PoseArray, "/cv/bin_opening_poses", 10)
        self.pub_platforms = self.create_publisher(PoseArray, "/cv/platform_poses", 10)

        # Subscribers
        self.sub_color = Subscriber(self, Image, self.color_topic)
        self.sub_depth = Subscriber(self, Image, self.depth_topic)
        self.sub_info = Subscriber(self, CameraInfo, self.info_topic)
        self.ts = ApproximateTimeSynchronizer([self.sub_color, self.sub_depth, self.sub_info], queue_size=10, slop=0.10)
        self.ts.registerCallback(self.cb)

        self.frame_count = 0
        self.get_logger().info("Stable+Attrs perception node started.")

    def _class_name(self, cls_id: int) -> str:
        if isinstance(self.names, dict):
            return self.names.get(cls_id, str(cls_id))
        return self.names[cls_id]

    def cb(self, color_msg: Image, depth_msg: Image, info_msg: CameraInfo):
        self.frame_count += 1

        if self.fx is None:
            self.fx = info_msg.k[0]
            self.fy = info_msg.k[4]
            self.cx = info_msg.k[2]
            self.cy = info_msg.k[5]
            self.get_logger().info(f"Intrinsics fx={self.fx:.2f} fy={self.fy:.2f} cx={self.cx:.2f} cy={self.cy:.2f}")

        try:
            color = image_msg_to_numpy(color_msg)
            depth = image_msg_to_numpy(depth_msg)
        except Exception as e:
            self.get_logger().warn(f"Decode failed: {e}")
            return

        # to BGR for YOLO + crop classifiers
        if (color_msg.encoding or "").lower() == "rgb8":
            color_bgr = color[:, :, ::-1].copy()
        else:
            color_bgr = color

        if depth.dtype == np.float32:
            depth_u16 = (depth / self.depth_scale_m).astype(np.uint16)
        else:
            depth_u16 = depth

        t0 = time.time()
        res = self.model(color_bgr, size=self.imgsz)
        det = res.xyxy[0].cpu().numpy()
        fps = 1.0 / max(1e-3, time.time() - t0)

        frame_id = color_msg.header.frame_id or "camera_color_optical_frame"

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

        now = float(color_msg.header.stamp.sec) + float(color_msg.header.stamp.nanosec) * 1e-9
        self._update_tracks(detections, color_bgr, now)

        if self.publish_every > 1 and (self.frame_count % self.publish_every != 0):
            return

        self._publish(frame_id, color_msg, fps)

    def _update_tracks(self, dets: List[Det], color_bgr: np.ndarray, now: float):
        # aging
        for tr in self.tracks.values():
            tr.age += 1
            tr.missed += 1

        # match per class
        dets_by_cls: Dict[str, List[Det]] = {}
        for d in dets:
            dets_by_cls.setdefault(d.cls, []).append(d)

        for cls, cls_dets in dets_by_cls.items():
            cls_tracks = [tr for tr in self.tracks.values() if tr.cls == cls]
            used = set()

            for tr in cls_tracks:
                best_j = None
                best_dist = 1e9

                for j, d in enumerate(cls_dets):
                    if j in used:
                        continue

                    dist = None
                    if tr.xyz_ema is not None and d.xyz is not None:
                        dv = tr.xyz_ema - np.array(d.xyz, dtype=np.float32)
                        dist = float(np.linalg.norm(dv))
                        if dist > self.assoc_max_dist_m:
                            continue
                    else:
                        # 2D fallback
                        du = tr.last_center_px[0] - d.center_px[0]
                        dv = tr.last_center_px[1] - d.center_px[1]
                        dist = float(np.hypot(du, dv)) / 1000.0
                        if dist > (self.assoc_max_dist_m * 3.0):
                            continue

                    if dist < best_dist:
                        best_dist = dist
                        best_j = j

                if best_j is not None:
                    d = cls_dets[best_j]
                    used.add(best_j)
                    self._apply_det_to_track(tr, d, color_bgr, now)

            for j, d in enumerate(cls_dets):
                if j in used:
                    continue
                self._create_track(d, color_bgr, now)

        # prune
        dead = [tid for tid, tr in self.tracks.items() if tr.missed > self.max_missed]
        for tid in dead:
            del self.tracks[tid]

        # enforce vote window on existing tracks
        for tr in self.tracks.values():
            tr.bin_color.labels = deque(tr.bin_color.labels, maxlen=self.vote_window)
            tr.bin_color.confs = deque(tr.bin_color.confs, maxlen=self.vote_window)
            tr.bin_color.probs = deque(tr.bin_color.probs, maxlen=self.vote_window)

            tr.block_color.labels = deque(tr.block_color.labels, maxlen=self.vote_window)
            tr.block_color.confs = deque(tr.block_color.confs, maxlen=self.vote_window)
            tr.block_color.probs = deque(tr.block_color.probs, maxlen=self.vote_window)

            tr.block_shape.labels = deque(tr.block_shape.labels, maxlen=self.vote_window)
            tr.block_shape.confs = deque(tr.block_shape.confs, maxlen=self.vote_window)
            tr.block_shape.probs = deque(tr.block_shape.probs, maxlen=self.vote_window)

    def _apply_det_to_track(self, tr: Track, d: Det, color_bgr: np.ndarray, now: float):
        tr.missed = 0
        tr.last_center_px = d.center_px
        tr.last_stamp_sec = now
        tr.conf_ema = ema(tr.conf_ema, d.conf, self.ema_alpha_conf)

        if d.xyz is not None:
            v = np.array(d.xyz, dtype=np.float32)
            tr.xyz_ema = v if tr.xyz_ema is None else ema_vec(tr.xyz_ema, v, self.ema_alpha_xyz)

        # attributes
        self._update_attributes(tr, d, color_bgr)

    def _create_track(self, d: Det, color_bgr: np.ndarray, now: float):
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

        self._update_attributes(tr, d, color_bgr)
    '''
    def _update_attributes(self, tr: Track, d: Det, color_bgr: np.ndarray):
        # Only classify when bbox is reasonable
        x1, y1, x2, y2 = d.bbox
        if (x2 - x1) < 10 or (y2 - y1) < 10:
            return

        crop = crop_bgr(color_bgr, d.bbox, pad_frac=self.crop_pad_frac)

        # bin_color
        if tr.cls == "bin" and self.bin_color is not None:
            try:
                label, conf, probd = self.bin_color.predict(crop)
                if conf >= self.min_attr_conf:
                    tr.bin_color.push(label, conf, probd)
            except Exception:
                pass

        # block attrs
        if tr.cls == "block" and self.block_attrs is not None:
            try:
                c_lab, c_conf, c_prob, s_lab, s_conf, s_prob = self.block_attrs.predict(crop)
                if c_conf >= self.min_attr_conf:
                    tr.block_color.push(c_lab, c_conf, c_prob)
                if s_conf >= self.min_attr_conf:
                    tr.block_shape.push(s_lab, s_conf, s_prob)
            except Exception:
                pass
    '''
    def _update_attributes(self, tr: Track, d: Det, color_bgr: np.ndarray):
        x1, y1, x2, y2 = d.bbox
        if (x2 - x1) < 10 or (y2 - y1) < 10:
            return

        crop = crop_bgr(color_bgr, d.bbox, pad_frac=self.crop_pad_frac)

        try:
            if tr.cls == "bin" and self.bin_color is not None:
                label, conf, probd = self.bin_color.predict(crop)
                if conf >= self.min_attr_conf:
                    tr.bin_color.push(label, conf, probd)

            if tr.cls == "block" and self.block_attrs is not None:
                c_lab, c_conf, c_prob, s_lab, s_conf, s_prob = self.block_attrs.predict(crop)
                if c_conf >= self.min_attr_conf:
                    tr.block_color.push(c_lab, c_conf, c_prob)
                if s_conf >= self.min_attr_conf:
                    tr.block_shape.push(s_lab, s_conf, s_prob)

            self.attr_ok += 1

        except Exception as e:
            self.attr_fail += 1
            now = time.time()
            if now - self._last_attr_warn > 2.0:  # warn at most every 2s
                self._last_attr_warn = now
                self.get_logger().warn(f"Attribute prediction failed (ok={self.attr_ok} fail={self.attr_fail}): {e}")   

    def _pose_from_xyz(self, xyz: np.ndarray) -> Pose:
        p = Pose()
        p.position.x = float(xyz[0])
        p.position.y = float(xyz[1])
        p.position.z = float(xyz[2])
        p.orientation.w = 1.0
        return p

    def _attr_summary(self, vote: AttrVote) -> Tuple[Optional[str], Optional[float], Dict[str,float], int]:
        if len(vote.labels) < self.min_votes:
            return None, None, {}, len(vote.labels)
        label = vote.mode()
        mconf = vote.mean_conf()
        probs = vote.mean_probs()
        return label, mconf, probs, len(vote.labels)

    def _publish(self, frame_id: str, color_msg: Image, fps: float):
        blocks = PoseArray(); blocks.header = color_msg.header; blocks.header.frame_id = frame_id
        bins = PoseArray(); bins.header = color_msg.header; bins.header.frame_id = frame_id
        opens = PoseArray(); opens.header = color_msg.header; opens.header.frame_id = frame_id
        plats = PoseArray(); plats.header = color_msg.header; plats.header.frame_id = frame_id

        tracks_out = []
        for tid, tr in sorted(self.tracks.items(), key=lambda kv: kv[0]):
            if tr.xyz_ema is None or tr.conf_ema < self.min_conf_publish:
                continue

            pose = self._pose_from_xyz(tr.xyz_ema)
            if tr.cls == "block":
                blocks.poses.append(pose)
            elif tr.cls == "bin":
                bins.poses.append(pose)
            elif tr.cls == "bin_opening":
                opens.poses.append(pose)
            elif tr.cls == "platform":
                plats.poses.append(pose)

            # attributes (voted)
            bin_color, bin_cconf, bin_probs, bin_n = self._attr_summary(tr.bin_color)
            blk_color, blk_cconf, blk_cprobs, blk_cn = self._attr_summary(tr.block_color)
            blk_shape, blk_sconf, blk_sprobs, blk_sn = self._attr_summary(tr.block_shape)

            tracks_out.append({
                "id": tr.track_id,
                "class": tr.cls,
                "conf": round(float(tr.conf_ema), 3),
                "xyz_m": [round(float(x), 4) for x in tr.xyz_ema.tolist()],
                "age": tr.age,
                "missed": tr.missed,
                "center_px": [int(tr.last_center_px[0]), int(tr.last_center_px[1])],

                "bin_color": {"label": bin_color, "conf": None if bin_cconf is None else round(float(bin_cconf), 3), "votes": bin_n, "probs": bin_probs},
                "block_color": {"label": blk_color, "conf": None if blk_cconf is None else round(float(blk_cconf), 3), "votes": blk_cn, "probs": blk_cprobs},
                "block_shape": {"label": blk_shape, "conf": None if blk_sconf is None else round(float(blk_sconf), 3), "votes": blk_sn, "probs": blk_sprobs},
            })

        self.pub_blocks.publish(blocks)
        self.pub_bins.publish(bins)
        self.pub_openings.publish(opens)
        self.pub_platforms.publish(plats)

        msg = String()
        msg.data = json.dumps({
            "stamp": {"sec": int(color_msg.header.stamp.sec), "nanosec": int(color_msg.header.stamp.nanosec)},
            "frame_id": frame_id,
            "yolo_fps": round(float(fps), 2),
            "tracks": tracks_out,
        })
        self.pub_tracks.publish(msg)

        if (self.frame_count % 30) == 0:
            self.get_logger().info(f"pub tracks={len(tracks_out)} yolo_fps~{fps:.1f}")
        if (self.frame_count % 50) == 0:
            self.get_logger().info(f"attr stats: ok={self.attr_ok} fail={self.attr_fail}")


def main():
    rclpy.init()
    node = PerceptionStableAttrsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()