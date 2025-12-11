#!/usr/bin/env python3
import argparse
import time
from pathlib import Path

import cv2
import numpy as np
import torch

import pyrealsense2 as rs


def colorize_depth(depth_frame, max_meters, depth_scale):
    depth = np.asanyarray(depth_frame.get_data()).astype(np.float32) * depth_scale
    depth = np.clip(depth, 0, max_meters)
    depth_norm = (depth / max_meters * 255.0).astype(np.uint8)
    return cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)


def main():
    ap = argparse.ArgumentParser(description="Live RealSense + YOLOv5 detection (blocks/bins/platform/opening)")
    ap.add_argument("--weights", required=True, help="YOLOv5 weights path (e.g. runs/train/blocks_bins_platform/weights/best.pt)")
    ap.add_argument("--serial", default=None, help="RealSense serial number (if multiple cameras)")
    ap.add_argument("--color-width", type=int, default=1280)
    ap.add_argument("--color-height", type=int, default=720)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--conf", type=float, default=0.25, help="confidence threshold")
    ap.add_argument("--show-depth", action="store_true", help="overlay depth colormap")
    ap.add_argument("--max-depth-m", type=float, default=2.0, help="max depth for colormap")
    args = ap.parse_args()

    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"[info] Using device: {device}")

    # Load YOLOv5 model from local repo
    # Adjust the repo path if your yolov5 folder is elsewhere
    repo = str(Path(__file__).resolve().parent)  # /home/.../yolov5
    model = torch.hub.load(repo, "custom", path=args.weights, source="local")
    model.eval()
    model.to(device)
    model.conf = args.conf  # confidence threshold
    names = model.names  # class names from training

    print(f"[info] Loaded YOLO model with classes: {names}")

    # Configure RealSense
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

    # Warm-up AE
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

            if not color_frame:
                print("[warn] No color frame")
                continue

            color = np.asanyarray(color_frame.get_data())
            vis = color.copy()

            # YOLO inference (BGR numpy -> model)
            # model is AutoShape-like, can take numpy directly
            results = model(vis, size=640)
            det = results.xyxy[0].cpu().numpy()  # x1, y1, x2, y2, conf, cls

            for (x1, y1, x2, y2, conf, cls_id) in det:
                cls_id = int(cls_id)
                label = names[cls_id] if cls_id in names else str(cls_id)

                x1_i, y1_i, x2_i, y2_i = map(int, [x1, y1, x2, y2])
                cv2.rectangle(vis, (x1_i, y1_i), (x2_i, y2_i), (0, 255, 0), 2)
                txt = f"{label} {conf:.2f}"
                cv2.putText(
                    vis, txt, (x1_i, max(0, y1_i - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA
                )

                # Also print to console
                print(f"{label:>12}  conf={conf:.2f}  box=({x1_i},{y1_i})-({x2_i},{y2_i})")

            if args.show_depth and depth_frame:
                depth_color = colorize_depth(depth_frame, args.max_depth_m, depth_scale)
                vis = cv2.addWeighted(vis, 0.6, depth_color, 0.4, 0)

            fps = 1.0 / max(1e-3, time.time() - t0)
            cv2.putText(vis, f"FPS ~ {fps:.1f}", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)

            cv2.imshow("RealSense YOLO (blocks/bins/platform)", vis)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord('q')):
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        print("[info] Stopped RealSense pipeline")


if __name__ == "__main__":
    main()
