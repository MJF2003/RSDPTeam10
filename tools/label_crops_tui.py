# tools/label_crops_tui.py
import argparse, csv, json
from pathlib import Path
import cv2
import numpy as np

# ==== Your class sets ====
COLORS  = ["red","pink","blue","dark_blue","purple","yellow","green"]
SHAPES  = ["small_cube","large_cube","cuboid","bridge","triangular"]

# Key mappings
COLOR_KEYS = "1234567"   # 1..7 -> COLORS
SHAPE_KEYS = "qwert"     # q,w,e,r,t -> SHAPES

WIN_NAME = "Blocks Crop Labeler (zoom+pan, no overlay)"

HELP = """
[Controls]
  Colors: 1:red 2:pink 3:blue 4:dark_blue 5:purple 6:yellow 7:green
  Shapes: q:small_cube w:large_cube e:cuboid r:bridge t:triangular
  Navigation: N / Right -> next    B / Left -> back
  Save CSV+state: S      Clear current image labels: C
  Fit-to-window toggle: F    Reset zoom/pan: R
  Quit: ESC
  Mouse: wheel = zoom,  left-drag = pan
[Status prints to terminal; image window shows only the crop]
"""

def letterbox(image, target_w, target_h, pad_color=(0,0,0)):
    """Fit image into window keeping aspect ratio."""
    H, W = image.shape[:2]
    if H == 0 or W == 0:
        return image
    scale = min(target_w / W, target_h / H)
    nw, nh = max(1, int(W*scale)), max(1, int(H*scale))
    resized = cv2.resize(image, (nw, nh), interpolation=cv2.INTER_AREA)
    canvas = np.full((target_h, target_w, 3), pad_color, dtype=np.uint8)
    x0 = (target_w - nw) // 2
    y0 = (target_h - nh) // 2
    canvas[y0:y0+nh, x0:x0+nw] = resized
    return canvas

def render_zoom_pan(img, win_w, win_h, zoom, pan_x, pan_y, pad_color=(0,0,0)):
    """Render with arbitrary zoom and pan into window size."""
    H, W = img.shape[:2]
    sw, sh = max(1, int(W*zoom)), max(1, int(H*zoom))
    scaled = cv2.resize(img, (sw, sh), interpolation=cv2.INTER_AREA if zoom<1.0 else cv2.INTER_LINEAR)

    # Place scaled image on black canvas using pan offsets (top-left at (ox, oy))
    canvas = np.full((win_h, win_w, 3), pad_color, dtype=np.uint8)
    ox = int((win_w - sw)//2 + pan_x)
    oy = int((win_h - sh)//2 + pan_y)

    # compute overlap region
    x1 = max(0, ox); y1 = max(0, oy)
    x2 = min(win_w, ox + sw); y2 = min(win_h, oy + sh)
    sx1 = max(0, -ox); sy1 = max(0, -oy)
    sx2 = sx1 + (x2 - x1); sy2 = sy1 + (y2 - y1)

    if x2 > x1 and y2 > y1:
        canvas[y1:y2, x1:x2] = scaled[sy1:sy2, sx1:sx2]
    return canvas

def save_csv(csv_path: Path, items):
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["filename","color","shape"])
        for row in items:
            w.writerow(row)

def print_status(split, idx, total, name, rec, zoom=None):
    ztxt = f"  zoom:{zoom:.2f}" if zoom is not None else ""
    print(f"[{split}] {idx+1}/{total}  {name}   -> color: {rec.get('color','-')}, shape: {rec.get('shape','-')}{ztxt}")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True, help="dataset root (contains crops/train and/or crops/val)")
    ap.add_argument("--split", default="train", choices=["train","val","valid"])
    ap.add_argument("--out_csv", default=None, help="output CSV path (default: crops/labels_<split>.csv)")
    ap.add_argument("--state", default=None, help="state json (default: crops/label_state_<split>.json)")
    ap.add_argument("--width", type=int, default=1280, help="window width")
    ap.add_argument("--height", type=int, default=800, help="window height")
    args = ap.parse_args()

    root = Path(args.root)
    crops_dir = root / "crops" / args.split
    assert crops_dir.exists(), f"Missing {crops_dir}"
    imgs = sorted([p for p in crops_dir.iterdir() if p.suffix.lower() in [".jpg",".jpeg",".png"]])
    assert imgs, f"No crops found in {crops_dir}"

    out_csv = Path(args.out_csv) if args.out_csv else root / f"crops/labels_{args.split}.csv"
    state_path = Path(args.state) if args.state else root / f"crops/label_state_{args.split}.json"

    # load/initialize state
    if state_path.exists():
        st = json.loads(state_path.read_text())
    else:
        st = {"index": 0, "labels": {}}

    idx = min(max(0, st.get("index", 0)), len(imgs)-1)
    labels = st.get("labels", {})  # filename -> {"color":..., "shape":""}

    # UI state
    fit_mode = True           # fit-to-window vs manual zoom/pan
    zoom = 1.0
    pan_x = 0.0
    pan_y = 0.0
    MIN_ZOOM, MAX_ZOOM = 0.2, 10.0
    dragging = False
    last_mouse = (0, 0)

    # window
    cv2.namedWindow(WIN_NAME, cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_NORMAL)
    cv2.resizeWindow(WIN_NAME, args.width, args.height)

    print(HELP.strip())
    rec0 = labels.get(imgs[idx].name, {"color":"", "shape":""})
    print_status(args.split, idx, len(imgs), imgs[idx].name, rec0, zoom)

    # Mouse callback for zoom/pan
    def on_mouse(event, x, y, flags, param):
        nonlocal dragging, last_mouse, pan_x, pan_y, zoom, fit_mode
        # Start drag
        if event == cv2.EVENT_LBUTTONDOWN:
            dragging = True
            last_mouse = (x, y)
        elif event == cv2.EVENT_LBUTTONUP:
            dragging = False
        elif event == cv2.EVENT_MOUSEMOVE and dragging and not fit_mode:
            dx = x - last_mouse[0]
            dy = y - last_mouse[1]
            pan_x += dx
            pan_y += dy
            last_mouse = (x, y)
        # Wheel zoom (OpenCV sends positive for forward/up)
        elif event == cv2.EVENT_MOUSEWHEEL and not fit_mode:
            # flags contains delta in high word on some builds; sign is enough
            delta = 1 if flags > 0 else -1
            # zoom around cursor: adjust pan so point under cursor stays
            # current window size
            try:
                _, _, win_w, win_h = cv2.getWindowImageRect(WIN_NAME)
            except Exception:
                win_w, win_h = args.width, args.height

            old_zoom = zoom
            new_zoom = np.clip(zoom * (1.1 if delta > 0 else 1/1.1), MIN_ZOOM, MAX_ZOOM)
            if abs(new_zoom - old_zoom) < 1e-6:
                return
            # Convert cursor to image-plane offsets and update pan
            # Derivation: keep (x,y) fixed while changing scale => adjust pan by (x - cx)*(1 - new/old)
            # where cx,cy are current center of window (0,0 reference)
            cx = win_w / 2.0
            cy = win_h / 2.0
            # shift of image top-left due to pan and center placement:
            pan_x = (x - cx) - (x - cx - pan_x) * (new_zoom / old_zoom)
            pan_y = (y - cy) - (y - cy - pan_y) * (new_zoom / old_zoom)
            zoom = new_zoom

    cv2.setMouseCallback(WIN_NAME, on_mouse)

    while True:
        img_path = imgs[idx]
        im = cv2.imread(str(img_path))
        if im is None:
            idx = min(idx+1, len(imgs)-1)
            continue

        try:
            _, _, win_w, win_h = cv2.getWindowImageRect(WIN_NAME)
        except Exception:
            win_w, win_h = args.width, args.height

        if fit_mode:
            canvas = letterbox(im, win_w, win_h)
        else:
            canvas = render_zoom_pan(im, win_w, win_h, zoom, pan_x, pan_y)

        cv2.imshow(WIN_NAME, canvas)
        k = cv2.waitKey(16) & 0xFF  # ~60 FPS refresh

        if k == 255:  # no key pressed
            continue

        if k == 27:  # ESC
            break
        elif k in (ord('f'), ord('F')):
            fit_mode = not fit_mode
            # reset view when switching to fit, keep current when switching back
            if fit_mode:
                zoom, pan_x, pan_y = 1.0, 0.0, 0.0
            rec = labels.get(imgs[idx].name, {"color":"", "shape":""})
            print_status(args.split, idx, len(imgs), imgs[idx].name, rec, zoom)
        elif k in (ord('r'), ord('R')):
            # reset zoom/pan (manual mode)
            zoom, pan_x, pan_y = 1.0, 0.0, 0.0
            fit_mode = False
            rec = labels.get(imgs[idx].name, {"color":"", "shape":""})
            print_status(args.split, idx, len(imgs), imgs[idx].name, rec, zoom)
        elif k in (ord('h'), ord('H')):
            print(HELP.strip())
        elif k in (ord('n'), ord('N'), 83):  # Right
            idx = min(idx+1, len(imgs)-1)
            rec = labels.get(imgs[idx].name, {"color":"", "shape":""})
            print_status(args.split, idx, len(imgs), imgs[idx].name, rec, zoom)
        elif k in (ord('b'), ord('B'), 81):  # Left
            idx = max(idx-1, 0)
            rec = labels.get(imgs[idx].name, {"color":"", "shape":""})
            print_status(args.split, idx, len(imgs), imgs[idx].name, rec, zoom)
        elif k in (ord('c'), ord('C')):
            labels[img_path.name] = {"color":"", "shape":""}
            print_status(args.split, idx, len(imgs), img_path.name, labels[img_path.name], zoom)
        elif k in (ord('s'), ord('S')):
            # write CSV snapshot + state
            items = []
            for p in imgs:
                r = labels.get(p.name, {"color":"", "shape":""})
                items.append([p.name, r.get("color",""), r.get("shape","")])
            save_csv(out_csv, items)
            st["index"] = idx; st["labels"] = labels
            state_path.write_text(json.dumps(st, indent=2))
            print(f"[saved] {out_csv}  state:{state_path}")
        else:
            ch = chr(k) if 32 <= k <= 126 else ""
            updated = False
            if ch in COLOR_KEYS:
                ci = COLOR_KEYS.index(ch)
                labels[img_path.name] = {"color": COLORS[ci], "shape": labels.get(img_path.name, {}).get("shape","")}
                updated = True
            elif ch in SHAPE_KEYS:
                si = SHAPE_KEYS.index(ch)
                labels[img_path.name] = {"color": labels.get(img_path.name, {}).get("color",""), "shape": SHAPES[si]}
                updated = True

            if updated:
                rec = labels[img_path.name]
                print_status(args.split, idx, len(imgs), img_path.name, rec, zoom)
                if rec.get("color") and rec.get("shape"):
                    idx = min(idx+1, len(imgs)-1)
                    rec2 = labels.get(imgs[idx].name, {"color":"", "shape":""})
                    print_status(args.split, idx, len(imgs), imgs[idx].name, rec2, zoom)

    # Final save on exit
    items = []
    for p in imgs:
        r = labels.get(p.name, {"color":"", "shape":""})
        items.append([p.name, r.get("color",""), r.get("shape","")])
    save_csv(out_csv, items)
    st["index"] = idx; st["labels"] = labels
    state_path.write_text(json.dumps(st, indent=2))
    cv2.destroyAllWindows()
    print(f"[done] CSV: {out_csv}  state: {state_path}")
    print("[tip] You can edit the CSV in a spreadsheet if needed.")

if __name__ == "__main__":
    main()
