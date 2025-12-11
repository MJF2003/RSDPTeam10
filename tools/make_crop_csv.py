from pathlib import Path
import csv
import argparse

def list_imgs(p: Path):
    exts = {".jpg", ".jpeg", ".png"}
    return sorted([x.name for x in p.iterdir() if x.suffix.lower() in exts])

def write_csv(root: Path, split: str, out_csv: Path):
    crops_dir = root / "crops" / split
    imgs = list_imgs(crops_dir) if crops_dir.exists() else []
    out_csv.parent.mkdir(parents=True, exist_ok=True)
    with open(out_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["filename", "color", "shape"])
        for fn in imgs:
            w.writerow([fn, "", ""])
    print(f"[ok] Wrote {out_csv} ({len(imgs)} rows)")

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True, help="Dataset root that contains crops/train and crops/val")
    ap.add_argument("--train_csv", default="crops/labels_train.csv")
    ap.add_argument("--val_csv", default="crops/labels_val.csv")
    args = ap.parse_args()

    ROOT = Path(args.root)
    write_csv(ROOT, "train", ROOT / args.train_csv)
    # handle either val or valid
    if (ROOT / "crops/val").exists():
        write_csv(ROOT, "val", ROOT / args.val_csv)
    elif (ROOT / "crops/valid").exists():
        write_csv(ROOT, "valid", ROOT / args.val_csv.replace("val", "valid"))
    else:
        print("[note] No crops/val or crops/valid found; wrote train CSV only.")
