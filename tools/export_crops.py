from pathlib import Path
import cv2

def load_txt(p):
    boxes = []
    if not p.exists(): return boxes
    for line in p.read_text().strip().splitlines():
        parts = line.split()
        if len(parts) >= 5:
            _, cx, cy, w, h = parts[:5]
            boxes.append(tuple(map(float, (cx, cy, w, h))))
    return boxes

def yolo_to_xyxy(box, W, H):
    cx, cy, w, h = box
    x1 = int((cx - w/2) * W); y1 = int((cy - h/2) * H)
    x2 = int((cx + w/2) * W); y2 = int((cy + h/2) * H)
    return max(0,x1), max(0,y1), min(W,x2), min(H,y2)

def export_split(images_dir: Path, out_dir: Path, pad: int = 2):
    if not images_dir.exists():
        return 0
    out_dir.mkdir(parents=True, exist_ok=True)
    n = 0
    for img_path in sorted(list(images_dir.glob("*.jpg")) + list(images_dir.glob("*.png"))):
        # infer label path by swapping /images/ -> /labels/ and .ext -> .txt
        lbl_path = Path(str(img_path).replace("/images/", "/labels/")).with_suffix(".txt")
        if not lbl_path.exists():
            continue
        img = cv2.imread(str(img_path))
        if img is None:
            continue
        H, W = img.shape[:2]
        for i, b in enumerate(load_txt(lbl_path)):
            x1,y1,x2,y2 = yolo_to_xyxy(b, W, H)
            x1, y1 = max(0, x1-pad), max(0, y1-pad)
            x2, y2 = min(W, x2+pad), min(H, y2+pad)
            crop = img[y1:y2, x1:x2]
            if crop.size == 0:
                continue
            out_name = f"{img_path.stem}_{i:02d}.jpg"
            cv2.imwrite(str(out_dir / out_name), crop)
            n += 1
    print(f"[done] {n} crops -> {out_dir}")
    return n

if __name__ == "__main__":
    # Point to the DATASET ROOT, not the 'train' folder
    ROOT = Path("/home/laptop30/yolov5/mydataset")

    # Handle common layouts automatically:
    # A) mydataset/train/{images,labels} and mydataset/valid/{images,labels}
    # B) mydataset/images/{train,val} and mydataset/labels/{train,val}
    total = 0
    total += export_split(ROOT / "train/images", ROOT / "crops/train")
    total += export_split(ROOT / "valid/images", ROOT / "crops/val")
    total += export_split(ROOT / "val/images",   ROOT / "crops/val")

    total += export_split(ROOT / "images/train", ROOT / "crops/train")
    total += export_split(ROOT / "images/val",   ROOT / "crops/val")

    if total == 0:
        print("[warn] No crops exported. Check that your image folders exist and that matching .txt files are in the parallel labels folders.")
