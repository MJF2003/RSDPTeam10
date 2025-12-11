import argparse, json
from pathlib import Path
import cv2, torch, joblib
import numpy as np
from PIL import Image
import torch.nn as nn
from torchvision import transforms, models

# ----- classifier (ResNet18) -------
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

def depth_xyz_from_bbox_center(depth_png_path, box, fx, fy, ppx, ppy, depth_scale, window=10, max_m=2.0):
    if not Path(depth_png_path).exists():
        return None
    depth = cv2.imread(str(depth_png_path), cv2.IMREAD_UNCHANGED)  # uint16
    x1,y1,x2,y2 = map(int, box)
    cx = int((x1+x2)/2); cy = int((y1+y2)/2)
    y0 = max(0, cy-window//2); y1_ = min(depth.shape[0], cy+window//2)
    x0 = max(0, cx-window//2); x1_ = min(depth.shape[1], cx+window//2)
    roi = depth[y0:y1_, x0:x1_]
    vals = roi[roi>0]
    if vals.size==0: return None
    Z = float(np.median(vals) * depth_scale)
    if Z<=0 or Z>max_m: return None
    X = (cx - ppx)/fx * Z
    Y = (cy - ppy)/fy * Z
    return (X,Y,Z)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--images", required=True)                  # folder of RGB images
    ap.add_argument("--weights", required=True)                 # yolov5 best.pt
    ap.add_argument("--clf_dir", required=True)                 # folder with best.pt + label_encoders.pkl
    ap.add_argument("--intrinsics", default=None)              # camera_intrinsics.json
    ap.add_argument("--depth_dir", default=None)               # folder with depth PNGs
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--show", action="store_true")
    args = ap.parse_args()

    device = "cuda" if torch.cuda.is_available() else "cpu"

    # load YOLOv5
    yolo = torch.hub.load("/home/laptop30/yolov5", "custom", path=args.weights, source="local")
    yolo.eval()
    yolo.conf = args.conf


    # load classifier + encoders
    enc = joblib.load(str(Path(args.clf_dir)/"label_encoders.pkl"))
    colors = list(enc["color_enc"].classes_)
    shapes = list(enc["shape_enc"].classes_)
    clf = MultiTaskNet(len(colors), len(shapes)).to(device)
    clf.load_state_dict(torch.load(str(Path(args.clf_dir)/"best.pt"), map_location=device))
    clf.eval()

    # intrinsics (optional)
    if args.intrinsics:
        jj = json.loads(Path(args.intrinsics).read_text())
        fx,fy,ppx,ppy = jj["fx"], jj["fy"], jj["ppx"], jj["ppy"]
        depth_scale = jj.get("depth_scale", 0.001)
    else:
        fx=fy=ppx=ppy=depth_scale=None

    img_dir = Path(args.images)
    for img_path in sorted(img_dir.glob("*.jpg")):
        im = cv2.imread(str(img_path))
        if im is None: continue

        results = yolo(im, size=640)
        det = results.xyxy[0].cpu().numpy()  # x1,y1,x2,y2,conf,cls
        print(f"\n{img_path.name}: {len(det)} detections")

        for i,(x1,y1,x2,y2,conf,_) in enumerate(det):
            # crop + classify
            crop = im[int(y1):int(y2), int(x1):int(x2)]
            if crop.size==0:
                print(f"  #{i}: empty crop, skipped"); continue
            x = TFM(Image.fromarray(cv2.cvtColor(crop, cv2.COLOR_BGR2RGB))).unsqueeze(0).to(device)
            with torch.no_grad():
                pC, pS = clf(x)
                col = colors[pC.argmax(1).item()]
                shp = shapes[pS.argmax(1).item()]

            line = f"  #{i}: {col}, {shp}, conf={conf:.2f}"

            # depth -> 3D
            if args.depth_dir and args.intrinsics:
                dpth = Path(args.depth_dir)/(img_path.stem + ".png")
                xyz = depth_xyz_from_bbox_center(dpth, (x1,y1,x2,y2), fx,fy,ppx,ppy, depth_scale)
                if xyz: line += f", XYZ=({xyz[0]:.3f},{xyz[1]:.3f},{xyz[2]:.3f}) m"
            print(line)

            if args.show:
                cv2.rectangle(im,(int(x1),int(y1)),(int(x2),int(y2)),(0,255,0),2)
                cv2.putText(im,f"{col},{shp}",(int(x1),max(0,int(y1)-6)),cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,0),2)

        if args.show:
            cv2.imshow("results", im); 
            if cv2.waitKey(0) & 0xFF in (27, ord('q')): pass
    if args.show: cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
