import argparse, json, time
from pathlib import Path
import cv2, torch, joblib, numpy as np
import torch.nn as nn
from torchvision import transforms, models
from PIL import Image

import pyrealsense2 as rs

# ---- Classifier (ResNet18) ----
class MultiTaskNet(nn.Module):
    def __init__(self, n_colors, n_shapes):
        super().__init__()
        backbone = models.resnet18(weights="IMAGENET1K_V1")
        in_feat = backbone.fc.in_features
        backbone.fc = nn.Identity()
        self.backbone = backbone
        self.fc_color = nn.Linear(in_feat, n_colors)
        self.fc_shape = nn.Linear(in_feat, n_shapes)
    def forward(self, x):
        f = self.backbone(x)
        return self.fc_color(f), self.fc_shape(f)

TFM = transforms.Compose([
    transforms.Resize((224,224)),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485,0.456,0.406], std=[0.229,0.224,0.225]),
])

def median_depth_Z(depth_frame, cx, cy, window=10):
    depth = np.asanyarray(depth_frame.get_data())
    y0 = max(0, cy-window//2); y1 = min(depth.shape[0], cy+window//2)
    x0 = max(0, cx-window//2); x1 = min(depth.shape[1], cx+window//2)
    roi = depth[y0:y1, x0:x1]
    vals = roi[roi>0]
    if vals.size==0: return None
    return float(np.median(vals))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", required=True)       # yolov5 best.pt
    ap.add_argument("--clf_dir", required=True)       # classifier folder
    ap.add_argument("--serial", default=None)
    ap.add_argument("--color_width", type=int, default=1280)
    ap.add_argument("--color_height", type=int, default=720)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--max_depth_m", type=float, default=2.0)
    ap.add_argument("--no_viz", action="store_true")
    args = ap.parse_args()

    device = "cuda" if torch.cuda.is_available() else "cpu"

    # YOLOv5
    yolo = torch.hub.load("/home/laptop30/yolov5", "custom", path=args.weights, source="local")
    yolo.eval()
    yolo.conf = args.conf


    # classifier + encoders
    enc = joblib.load(str(Path(args.clf_dir)/"label_encoders.pkl"))
    colors = list(enc["color_enc"].classes_)
    shapes = list(enc["shape_enc"].classes_)
    clf = MultiTaskNet(len(colors), len(shapes)).to(device)
    clf.load_state_dict(torch.load(str(Path(args.clf_dir)/"best.pt"), map_location=device))
    clf.eval()

    # RealSense pipeline
    pipeline = rs.pipeline()
    cfg = rs.config()
    if args.serial: cfg.enable_device(args.serial)
    cfg.enable_stream(rs.stream.color, args.color_width, args.color_height, rs.format.bgr8, args.fps)
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, args.fps)
    prof = pipeline.start(cfg)

    depth_sensor = prof.get_device().first_depth_sensor()
    depth_scale = float(depth_sensor.get_depth_scale())
    align = rs.align(rs.stream.color)

    # intrinsics
    cprof = prof.get_stream(rs.stream.color).as_video_stream_profile()
    intr = cprof.get_intrinsics()
    fx,fy,ppx,ppy = intr.fx, intr.fy, intr.ppx, intr.ppy

    print(f"[info] depth_scale={depth_scale} m/unit  fx={fx:.1f} fy={fy:.1f} ppx={ppx:.1f} ppy={ppy:.1f}")

    try:
        while True:
            frames = pipeline.wait_for_frames()
            aligned = align.process(frames)
            color = aligned.get_color_frame()
            depth = aligned.get_depth_frame()
            if not color: continue

            img = np.asanyarray(color.get_data())

            # YOLO detect
            t0 = time.time()
            results = yolo(img, size=640)
            det = results.xyxy[0].cpu().numpy()
            t1 = time.time()

            for (x1,y1,x2,y2,conf,_) in det:
                # classify crop
                crop = img[int(y1):int(y2), int(x1):int(x2)]
                if crop.size==0: continue
                x = TFM(Image.fromarray(cv2.cvtColor(crop, cv2.COLOR_BGR2RGB))).unsqueeze(0).to(device)
                with torch.no_grad():
                    pC, pS = clf(x)
                    c = colors[pC.argmax(1).item()]
                    s = shapes[pS.argmax(1).item()]

                # depth -> 3D
                cx = int((x1+x2)/2); cy = int((y1+y2)/2)
                z_units = median_depth_Z(depth, cx, cy, window=10)
                xyz_txt = ""
                if z_units is not None:
                    Z = z_units * depth_scale
                    if 0 < Z < args.max_depth_m:
                        X = (cx - ppx)/fx * Z
                        Y = (cy - ppy)/fy * Z
                        xyz_txt = f" XYZ=({X:.3f},{Y:.3f},{Z:.3f})m"
                        if not args.no_viz:
                            cv2.circle(img, (cx,cy), 3, (0,255,255), -1)

                print(f"{c:>10}, {s:<12} conf={conf:.2f}{xyz_txt}")

                if not args.no_viz:
                    cv2.rectangle(img,(int(x1),int(y1)),(int(x2),int(y2)),(0,255,0),2)
                    cv2.putText(img,f"{c},{s}",(int(x1),max(0,int(y1)-6)),cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,0),2)

            if not args.no_viz:
                fps = 1.0 / max(1e-3, (time.time()-t0))
                cv2.putText(img, f"FPS~{fps:.1f}", (10,25), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)
                cv2.imshow("live blocks", img)
                k = cv2.waitKey(1) & 0xFF
                if k in (27, ord('q')):
                    break

    finally:
        pipeline.stop()
        if not args.no_viz: cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
