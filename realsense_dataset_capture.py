#!/usr/bin/env python3
import argparse
import json
from pathlib import Path
import time

import cv2
import numpy as np

try:
    import pyrealsense2 as rs
except Exception as e:
    print("Failed to import pyrealsense2. Install with `pip install pyrealsense2` and set up librealsense for your OS.")
    raise

def save_intrinsics_once(path, profile, depth_scale=None):
    path = Path(path)
    if path.exists():
        return
    color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
    intr = color_stream.get_intrinsics()
    meta = {
        "width": intr.width,
        "height": intr.height,
        "fx": intr.fx,
        "fy": intr.fy,
        "ppx": intr.ppx,
        "ppy": intr.ppy,
        "model": str(intr.model),
        "coeffs": list(intr.coeffs),
        "depth_scale": depth_scale,
        "note": "depth_in_meters = uint16_depth_value * depth_scale"
    }
    path.write_text(json.dumps(meta, indent=2))
    print(f"[info] Wrote intrinsics to {path}")

def colorize_depth(depth_frame, max_meters, depth_scale):
    depth_image = np.asanyarray(depth_frame.get_data()).astype(np.float32) * depth_scale
    depth_image = np.clip(depth_image, 0, max_meters)
    depth_norm = (depth_image / max_meters * 255.0).astype(np.uint8)
    return cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)

def put_hud(img, lines, y0=24, scale=0.6):
    # compact overlay to remind controls; can be toggled off with 'H'
    for i, line in enumerate(lines):
        cv2.putText(img, line, (10, y0 + i * 22), cv2.FONT_HERSHEY_SIMPLEX, scale, (255, 255, 255), 2, cv2.LINE_AA)

def main():
    ap = argparse.ArgumentParser(description="Intel RealSense dataset capture (RGB + optional aligned depth)")
    ap.add_argument("--out", type=str, default="blocks_ds", help="Output root folder")
    ap.add_argument("--prefix", type=str, default="scene", help="Base filename prefix")
    ap.add_argument("--scene-tag", type=str, default="generic", help="Scene tag to include in filenames (e.g., blocks_platform, bins_only)")
    ap.add_argument("--serial", type=str, default=None, help="Device serial (if multiple cameras)")
    ap.add_argument("--color-width", type=int, default=1280)
    ap.add_argument("--color-height", type=int, default=720)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--save-depth", action="store_true", help="Save aligned depth PNGs (16-bit)")
    ap.add_argument("--max-depth-m", type=float, default=2.0, help="Depth overlay max range (meters)")
    ap.add_argument("--every", type=int, default=1, help="Save every Nth frame when pressing Space")
    ap.add_argument("--max", type=int, default=0, help="Stop after saving this many frames (0 = no limit)")
    ap.add_argument("--rotate", type=int, default=0, choices=[0, 90, 180, 270], help="Rotate preview & saved images")
    ap.add_argument("--no-hud", action="store_true", help="Hide on-screen help")
    args = ap.parse_args()

    out_root = Path(args.out)
    images_dir = out_root / "images"
    depth_dir = out_root / "depth" if args.save_depth else None
    labels_dir = out_root / "labels"  # empty placeholder for later
    images_dir.mkdir(parents=True, exist_ok=True)
    labels_dir.mkdir(parents=True, exist_ok=True)
    if depth_dir:
        depth_dir.mkdir(parents=True, exist_ok=True)

    # Configure RealSense
    pipeline = rs.pipeline()
    config = rs.config()
    if args.serial:
        config.enable_device(args.serial)
    config.enable_stream(rs.stream.color, args.color_width, args.color_height, rs.format.bgr8, args.fps)
    # enable depth (we'll align to color); if not saving we still enable so overlay is available
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, args.fps)

    profile = pipeline.start(config)

    # Device info
    dev = profile.get_device()
    try:
        serial = dev.get_info(rs.camera_info.serial_number)
        name = dev.get_info(rs.camera_info.name)
        print(f"[info] Using device: {name} (S/N: {serial})")
    except Exception:
        pass

    # Depth scale and align filter
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    align = rs.align(rs.stream.color)

    # Save intrinsics
    save_intrinsics_once(out_root / "camera_intrinsics.json", profile, depth_scale=depth_scale)

    # Warm-up for auto-exposure
    print("[info] Warming up auto-exposure...")
    for _ in range(20):
        pipeline.wait_for_frames()

    print("[info] Controls: SPACE=save  O=toggle depth overlay  T=cycle scene tag  H=toggle help  Q/ESC=quit")
    show_depth_overlay = False
    show_hud = not args.no_hud
    saved = 0
    frame_idx = 0

    # Determine starting index by checking existing files with this prefix+tag
    def next_index():
        # pattern: {prefix}_{tag}_NNNNNN.jpg
        existing = sorted(images_dir.glob(f"{args.prefix}_{args.scene_tag}_*.jpg"))
        if existing:
            try:
                last = existing[-1].stem.split("_")[-1]
                return int(last) + 1
            except Exception:
                return 1
        return 1

    img_counter = next_index()

    def rotate_img(img):
        if args.rotate == 0:   return img
        if args.rotate == 90:  return cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
        if args.rotate == 180: return cv2.rotate(img, cv2.ROTATE_180)
        if args.rotate == 270: return cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)

    # Scene tag cycling (useful if you want to quickly switch what you're shooting)
    scene_tags = [args.scene_tag, "blocks_platform", "bins", "mixed"]
    # ensure uniqueness but preserve order
    seen = set()
    scene_tags = [t for t in scene_tags if not (t in seen or seen.add(t))]
    tag_idx = 0
    args.scene_tag = scene_tags[tag_idx]

    try:
        while True:
            frames = pipeline.wait_for_frames()
            aligned = align.process(frames)
            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame() if (args.save_depth or show_depth_overlay) else None
            if not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            vis = color_image.copy()

            if show_depth_overlay and depth_frame:
                depth_color = colorize_depth(depth_frame, args.max_depth_m, depth_scale)
                vis = cv2.addWeighted(vis, 0.6, depth_color, 0.4, 0)

            if show_hud:
                put_hud(vis, [
                    f"Tag: {args.scene_tag}",
                    "SPACE=save  O=depth overlay  T=cycle tag  H=help  Q/ESC=quit",
                    f"Saved: {saved}"
                ])

            vis = rotate_img(vis)
            cv2.imshow("RealSense Capture", vis)
            key = cv2.waitKey(1) & 0xFF
            frame_idx += 1

            if key in (ord('q'), ord('Q'), 27):
                break
            elif key in (ord('o'), ord('O')):
                show_depth_overlay = not show_depth_overlay
            elif key in (ord('h'), ord('H')):
                show_hud = not show_hud
            elif key in (ord('t'), ord('T')):
                # cycle through scene tags and reset index counter for that tag
                tag_idx = (tag_idx + 1) % len(scene_tags)
                args.scene_tag = scene_tags[tag_idx]
                img_counter = next_index()
                print(f"[info] Switched scene tag -> {args.scene_tag}")
            elif key == 32:  # SPACE
                if frame_idx % max(1, args.every) == 0:
                    img_name = f"{args.prefix}_{args.scene_tag}_{img_counter:06d}.jpg"
                    img_path = images_dir / img_name
                    color_to_save = rotate_img(color_image)
                    ok = cv2.imwrite(str(img_path), color_to_save, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
                    if not ok:
                        print(f"[warn] Failed to write {img_path}")
                        continue
                    if args.save_depth and depth_frame:
                        depth_raw = np.asanyarray(depth_frame.get_data())  # uint16
                        depth_to_save = rotate_img(depth_raw)
                        depth_name = f"{args.prefix}_{args.scene_tag}_{img_counter:06d}.png"
                        depth_path = depth_dir / depth_name
                        ok2 = cv2.imwrite(str(depth_path), depth_to_save)
                        if not ok2:
                            print(f"[warn] Failed to write {depth_path}")

                    print(f"[save] {img_name}")
                    img_counter += 1
                    saved += 1
                    if args.max > 0 and saved >= args.max:
                        print("[info] Reached --max saves. Exiting.")
                        break

    except KeyboardInterrupt:
        pass
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        print(f"[done] Saved {saved} images to {images_dir}")
        if args.save_depth:
            print(f"[done] Depth PNGs in {depth_dir}")

if __name__ == "__main__":
    main()
