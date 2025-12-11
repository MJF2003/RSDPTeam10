from pathlib import Path
import cv2

# In your data.yaml, names[0] = "block"
BLOCK_CLASS_ID = 2

def load_block_boxes(txt_path):
    boxes = []
    if not txt_path.exists():
        return boxes
    for line in txt_path.read_text().strip().splitlines():
        parts = line.split()
        if len(parts) < 5:
            continue
        cls = int(float(parts[0]))
        if cls != BLOCK_CLASS_ID:
            continue
        _, cx, cy, w, h = parts[:5]
        boxes.append(tuple(map(float, (cx, cy, w, h))))
    return boxes

def yolo_to_xyxy(box, W, H):
    cx, cy, w, h = box
    x1 = int((cx - w/2) * W); y1 = int((cy - h/2) * H)
    x2 = int((cx + w/2) * W); y2 = int((cy + h/2) * H)
    return max(0,x1), max(0,y1), min(W,x2), min(H,y2)

def export_split(img_dir: Path, out_dir: Path, pad: int = 2):
    if not img_dir.exists():
        return 0
    out_dir.mkdir(parents=True, exist_ok=True)
    count = 0
    for img_path in sorted(list(img_dir.glob("*.jpg")) + list(img_dir.glob("*.png"))):
        lbl_path = Path(str(img_path).replace("/images/", "/labels/")).with_suffix(".txt")
        boxes = load_block_boxes(lbl_path)
        if not boxes:
            continue
        img = cv2.imread(str(img_path))
        if img is None:
            continue
        H, W = img.shape[:2]
        for i, b in enumerate(boxes):
            x1,y1,x2,y2 = yolo_to_xyxy(b, W, H)
            x1, y1 = max(0,x1-pad), max(0,y1-pad)
            x2, y2 = min(W,x2+pad), min(H,y2+pad)
            crop = img[y1:y2, x1:x2]
            if crop.size == 0:
                continue
            out_name = f"{img_path.stem}_block_{i:02d}.jpg"
            cv2.imwrite(str(out_dir / out_name), crop)
            count += 1
    print(f"[done] {count} block crops -> {out_dir}")
    return count

if __name__ == "__main__":
    ROOT = Path("/home/laptop30/yolov5/bins_blocks_dataset")
    total = 0
    total += export_split(ROOT / "train/images", ROOT / "crops/train")
    total += export_split(ROOT / "valid/images", ROOT / "crops/val")
    if total == 0:
        print("[warn] No block crops exported. Check class IDs and paths.")
