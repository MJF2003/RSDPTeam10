#!/usr/bin/env python3
import argparse
import time
from pathlib import Path

import cv2
import numpy as np
import torch
import torch.nn as nn
from torchvision import transforms, models
from PIL import Image
import joblib
import pyrealsense2 as rs


# ---------- Classifier models ----------

class BlockAttrNet(nn.Module):
    """ResNet18 backbone with 2 heads: color + shape (multi-task)."""
    def __init__(self, n_colors, n_shapes):
        super().__init__()
        backbone = models.resnet18(weights="IMAGENET1K_V1")
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
    def __init__(self, n_colors):
        super().__init__()
        backbone = models.resnet18(weights="IMAGENET1K_V1")
        in_feat = backbone.fc.in_features
        backbone.fc = nn.Identity()
        self.backbone = backbone
        self.fc = nn.Linear(in_feat, n_colors)

    def forward(self, x):
        feat = self.backbone(x)
        return self.fc(feat)


# ---------- Utils ----------

def build_block_transform():
    # Inference transform: no heavy aug, just resize + normalize
    return transforms.Compose([
        transforms.Resize((224, 224)),
        transforms.ToTensor(),
        transforms.Normalize(
            mean=[0.485, 0.456, 0.406],
            std=[0.229, 0.224, 0.225],
        ),
    ])


def build_bin_transform():
    # Same idea for bins
    return transforms.Compose([
        transforms.Resize((224, 224)),
        transforms.ToTensor(),
        transforms.Normalize(
            mean=[0.485, 0.456, 0.406],
            std=[0.229, 0.224, 0.225],
        ),
    ])


def colorize_depth(depth_frame, max_meters, depth_scale):
    depth = np.asanyarray(depth_frame.get_data()).astype(np.float32) * depth_scale
    depth = np.clip(depth, 0, max_meters)
    depth_norm = (depth / max_meters * 255.0).astype(np.uint8)
    return cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)


def median_depth_Z(depth_frame, cx, cy, window=10):
    """Return median depth (meter units) around pixel (cx,cy), or None."""
    depth = np.asanyarray(depth_frame.get_data())
    h, w = depth.shape
    x0 = max(0, cx - window // 2)
    x1 = min(w, cx + window // 2)
    y0 = max(0, cy - window // 2)
    y1 = min(h, cy + window // 2)
    roi = depth[y0:y1, x0:x1]
    vals = roi[roi > 0]
    if vals.size == 0:
        return None
    return float(np.median(vals))


def main():
    ap = argparse.ArgumentParser(description="RealSense + YOLOv5 + block/bin classifiers + 3D positions")
    ap.add_argument("--weights", required=True, help="YOLOv5 detector weights (.pt)")
    ap.add_argument("--block_clf_dir", required=True, help="Dir with block attrs clf: best.pt + label_encoders.pkl")
    ap.add_argument("--bin_clf_dir", required=True, help="Dir with bin color clf: best.pt + label_encoder.pkl")
    ap.add_argument("--serial", default=None, help="RealSense serial (optional)")
    ap.add_argument("--color_width", type=int, default=1280)
    ap.add_argument("--color_height", type=int, default=720)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--max_depth_m", type=float, default=2.0)
    ap.add_argument("--show_depth", action="store_true", help="overlay depth colormap")
    args = ap.parse_args()

    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"[info] Using device: {device}")

    # ---------- Load YOLOv5 ----------
    repo = str(Path(__file__).resolve().parent)  # local yolov5 repo
    model = torch.hub.load(repo, "custom", path=args.weights, source="local")
    model.eval()
    model.to(device)
    model.conf = args.conf
    names = model.names  # class index -> name
    print(f"[info] YOLO classes: {names}")

    # ---------- Load block attribute classifier ----------
    block_dir = Path(args.block_clf_dir)
    block_encoders = joblib.load(block_dir / "label_encoders.pkl")
    color_enc = block_encoders["color_enc"]
    shape_enc = block_encoders["shape_enc"]
    block_colors = list(color_enc.classes_)
    block_shapes = list(shape_enc.classes_)
    print(f"[info] Block colors: {block_colors}")
    print(f"[info] Block shapes: {block_shapes}")

    block_model = BlockAttrNet(len(block_colors), len(block_shapes))
    block_model.load_state_dict(torch.load(block_dir / "best.pt", map_location=device))
    block_model.to(device)
    block_model.eval()
    block_tfm = build_block_transform()

    # ---------- Load bin color classifier ----------
    bin_dir = Path(args.bin_clf_dir)
    bin_encoders = joblib.load(bin_dir / "label_encoder.pkl")
    bin_color_enc = bin_encoders["color_encoder"]
    bin_colors = list(bin_color_enc.classes_)
    print(f"[info] Bin colors: {bin_colors}")

    bin_model = BinColorNet(len(bin_colors))
    bin_model.load_state_dict(torch.load(bin_dir / "best.pt", map_location=device))
    bin_model.to(device)
    bin_model.eval()
    bin_tfm = build_bin_transform()

    # ---------- RealSense setup ----------
    pipeline = rs.pipeline()
    config = rs.config()
    if args.serial:
        config.enable_device(args.serial)
    config.enable_stream(rs.stream.color, args.color_width, args.color_height, rs.format.bgr8, args.fps)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, args.fps)

    print("[info] Starting RealSense pipeline...")
    profile = pipeline.start(config)

    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print(f"[info] depth_scale = {depth_scale} m/unit")

    align = rs.align(rs.stream.color)

    # Intrinsics of color stream (for 3D deprojection)
    cprof = profile.get_stream(rs.stream.color).as_video_stream_profile()
    intr = cprof.get_intrinsics()
    fx, fy, ppx, ppy = intr.fx, intr.fy, intr.ppx, intr.ppy
    print(f"[info] intrinsics: fx={fx:.1f}, fy={fy:.1f}, ppx={ppx:.1f}, ppy={ppy:.1f}")

    # Warm-up
    print("[info] Warming up auto-exposure...")
    for _ in range(20):
        pipeline.wait_for_frames()

    try:
        while True:
            t0 = time.time()
            frames = pipeline.wait_for_frames()
            aligned = align.process(frames)
            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()
            if not color_frame or not depth_frame:
                print("[warn] missing color or depth frame")
                continue

            color = np.asanyarray(color_frame.get_data())
            vis = color.copy()

            # ---------- YOLO inference ----------
            # model can take numpy directly when loaded via torch.hub like this
            results = model(vis, size=640)
            det = results.xyxy[0].cpu().numpy()  # x1,y1,x2,y2,conf,cls

            detections_for_ros = []  # list of dicts per object (for ROS later)

            for (x1, y1, x2, y2, conf, cls_id) in det:
                cls_id = int(cls_id)
                cls_name = names[cls_id] if cls_id in names else str(cls_id)

                x1_i, y1_i, x2_i, y2_i = map(int, [x1, y1, x2, y2])
                w = x2_i - x1_i
                h = y2_i - y1_i
                if w <= 0 or h <= 0:
                    continue

                # Crop for classifier
                crop = color[y1_i:y2_i, x1_i:x2_i]
                attr_color = None
                attr_shape = None

                # Block attributes
                if cls_name == "block":
                    if crop.size > 0:
                        pil = Image.fromarray(cv2.cvtColor(crop, cv2.COLOR_BGR2RGB))
                        x = block_tfm(pil).unsqueeze(0).to(device)
                        with torch.no_grad():
                            logits_c, logits_s = block_model(x)
                            idx_c = logits_c.argmax(1).item()
                            idx_s = logits_s.argmax(1).item()
                            attr_color = block_colors[idx_c]
                            attr_shape = block_shapes[idx_s]

                # Bin color
                elif cls_name == "bin":
                    if crop.size > 0:
                        pil = Image.fromarray(cv2.cvtColor(crop, cv2.COLOR_BGR2RGB))
                        x = bin_tfm(pil).unsqueeze(0).to(device)
                        with torch.no_grad():
                            logits = bin_model(x)
                            idx_c = logits.argmax(1).item()
                            attr_color = bin_colors[idx_c]

                # 3D position from depth (center of bbox)
                cx = x1_i + w // 2
                cy = y1_i + h // 2
                Z_units = median_depth_Z(depth_frame, cx, cy, window=10)
                X = Y = Z = None
                if Z_units is not None:
                    Z = Z_units * depth_scale  # meters
                    if 0 < Z < args.max_depth_m:
                        X = (cx - ppx) / fx * Z
                        Y = (cy - ppy) / fy * Z

                # Draw on image
                label_txt = cls_name
                if cls_name == "block":
                    if attr_color:
                        label_txt += f" {attr_color}"
                    if attr_shape:
                        label_txt += f" {attr_shape}"
                elif cls_name == "bin":
                    if attr_color:
                        label_txt += f" {attr_color}"

                label_txt += f" {conf:.2f}"

                cv2.rectangle(vis, (x1_i, y1_i), (x2_i, y2_i), (0, 255, 0), 2)
                cv2.putText(
                    vis,
                    label_txt,
                    (x1_i, max(0, y1_i - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )

                if X is not None:
                    # Draw center point
                    cv2.circle(vis, (cx, cy), 3, (0, 255, 255), -1)
                    xyz_txt = f" ({X:.2f},{Y:.2f},{Z:.2f})m"
                else:
                    xyz_txt = " (no depth)"

                print(f"{cls_name:12s} conf={conf:.2f}  attr_color={attr_color} attr_shape={attr_shape}  XYZ={xyz_txt}")

                det_dict = {
                    "class": cls_name,
                    "conf": float(conf),
                    "bbox": [x1_i, y1_i, x2_i, y2_i],
                    "center_px": [cx, cy],
                    "attr_color": attr_color,
                    "attr_shape": attr_shape,
                    "X": X,
                    "Y": Y,
                    "Z": Z,
                    "frame_id": "camera_color_optical_frame",
                }
                detections_for_ros.append(det_dict)

            # (Here you could later call a function to publish detections_for_ros to ROS2)

            if args.show_depth:
                depth_color = colorize_depth(depth_frame, args.max_depth_m, depth_scale)
                vis = cv2.addWeighted(vis, 0.6, depth_color, 0.4, 0)

            fps = 1.0 / max(1e-3, time.time() - t0)
            cv2.putText(vis, f"FPS ~ {fps:.1f}", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)

            cv2.imshow("RealSense + YOLO + Attrs", vis)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord('q')):
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        print("[info] Stopped RealSense pipeline")


if __name__ == "__main__":
    main()
