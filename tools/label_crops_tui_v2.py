import argparse, csv, json
from pathlib import Path
import cv2
import numpy as np


# ==== Your class sets ====
COLORS  = ["red","pink","blue","dark_blue","purple","yellow","green"]
SHAPES  = ["small_cube","large_cube","cuboid","bridge","triangular"]

# Key mappings (easy, one-hand)
COLOR_KEYS = "1234567"   # 1..7 map to COLORS
SHAPE_KEYS = "qwert"     # q,w,e,r,t map to SHAPES

# Window defaults
WIN_NAME      = "Blocks Crop Labeler"
WIN_W, WIN_H  = 1280, 800
FOOTER_H_INIT = 84   # height (px) reserved for footer
FONT_SCALE_INIT = 0.6
FONT          = cv2.FONT_HERSHEY_SIMPLEX

HELP_TEXT = [
    "Color: 1:red 2:pink 3:blue 4:dark_blue 5:purple 6:yellow 7:green",
    "Shape: q:small_cube w:large_cube e:cuboid r:bridge t:triangular",
    "[N]/[Right]=next   [B]/[Left]=back   [S]=save   [C]=clear current   [ESC]=quit",
    "[H]=toggle help   [+/-]=font size   [F]=fit mode on/off"
]

def draw_footer(canvas, lines, font_scale=0.6, opacity=0.85, pad=8):
    """Draw a semi-transparent footer at bottom with small text lines."""
    h, w = canvas.shape[:2]
    footer_h = int(max(48, FOOTER_H_INIT * (font_scale / FONT_SCALE_INIT)))
    overlay = canvas.copy()
    # footer rectangle
    cv2.rectangle(overlay, (0, h - footer_h), (w, h), (0, 0, 0), thickness=-1)
    canvas[:] = cv2.addWeighted(overlay, opacity, canvas, 1 - opacity, 0)

    y = h - footer_h + pad + 18
    for ln in lines:
        cv2.putText(canvas, ln, (10, y), FONT, font_scale, (255, 255, 255), 2, cv2.LINE_AA)
        y += int(24 * font_scale + 6)
    return canvas

def fit_into(image, target_w, target_h, pad_color=(0, 0, 0)):
    """Letterbox image into (target_w, target_h) keeping aspect ratio."""
    H, W = image.shape[:2]
    if H == 0 or W == 0:
        return image, (0, 0, W, H)
    scale = min(target_w / W, target_h / H)
    new_w, new_h = max(1, int(W * scale)), max(1, int(H * scale))
    resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_AREA)

    # Create black canvas and paste centered
    canvas = np.full((target_h, target_w, 3), pad_color, dtype=np.uint8)
    x0 = (target_w - new_w) // 2
    y0 = (target_h - new_h) // 2
    canvas[y0:y0 + new_h, x0:x0 + new_w] = resized
    return canvas, (x0, y0, new_w, new_h)


def render_frame(img_bgr, show_help, font_scale, info_line_left, info_line_right, fit_mode=True):
    """Render a frame with optional footer; returns canvas and the viewport rect."""
    # Determine current window size
    try:
        _, _, win_w, win_h = cv2.getWindowImageRect(WIN_NAME)
    except Exception:
        win_w, win_h = WIN_W, WIN_H

    footer_h = int(max(48, FOOTER_H_INIT * (font_scale / FONT_SCALE_INIT)))
    usable_h = max(100, win_h - (footer_h if show_help else 0))

    if fit_mode:
        canvas, viewport = fit_into(img_bgr, win_w, usable_h)
    else:
        # 1:1; if larger than window, it will be clipped by OS window manager
        canvas = cv2.resize(img_bgr, (min(img_bgr.shape[1], win_w),
                                      min(img_bgr.shape[0], usable_h)))
        viewport = (0, 0, canvas.shape[1], canvas.shape[0])
        # pad to window width/height
        if canvas.shape[1] < win_w or canvas.shape[0] < usable_h:
            bg = (canvas * 0).copy()
            bg = cv2.resize(bg, (win_w, usable_h))
            y = (usable_h - canvas.shape[0]) // 2
            x = (win_w - canvas.shape[1]) // 2
            bg[y:y+canvas.shape[0], x:x+canvas.shape[1]] = canvas
            canvas = bg
            viewport = (x, y, canvas.shape[1], canvas.shape[0])

    # top info (small, unobtrusive)
    top_line = f"{info_line_left}"
    right_line = f"{info_line_right}"
    # left
    cv2.putText(canvas, top_line, (10, 26), FONT, max(0.5, font_scale*0.85), (255,255,255), 2, cv2.LINE_AA)
    # right (align to right edge roughly)
    (tw, th), _ = cv2.getTextSize(right_line, FONT, max(0.5, font_scale*0.85), 2)
    cv2.putText(canvas, right_line, (canvas.shape[1]-tw-10, 26), FONT, max(0.5, font_scale*0.85), (255,255,255), 2, cv2.LINE_AA)

    if show_help:
        # Compose footer lines compactly
        lines = []
        lines += HELP_TEXT[:2]
        lines.append(HELP_TEXT[2])
        lines.append(HELP_TEXT[3])
        canvas = draw_footer(canvas, lines, font_scale=font_scale)

    return canvas, viewport

def save_csv(csv_path, items):
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["filename","color","shape"])
        for row in items:
            w.writerow(row)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True, help="dataset root (contains crops/train and/or crops/val)")
    ap.add_argument("--split", default="train", choices=["train","val","valid"])
    ap.add_argument("--out_csv", default=None, help="output CSV path (default: crops/labels_<split>.csv)")
    ap.add_argument("--state", default=None, help="state json (default: crops/label_state_<split>.json)")
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
    labels = st.get("labels", {})  # filename -> {"color":..., "shape":...}

    # UI state
    show_help   = True
    fit_mode    = True
    font_scale  = FONT_SCALE_INIT

    cv2.namedWindow(WIN_NAME, cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_NORMAL)
    cv2.resizeWindow(WIN_NAME, WIN_W, WIN_H)

    while True:
        img_path = imgs[idx]
        im = cv2.imread(str(img_path))
        if im is None:
            idx = min(idx+1, len(imgs)-1)
            continue

        rec = labels.get(img_path.name, {"color":"", "shape":""})
        left  = f"[{args.split}] {idx+1}/{len(imgs)}  {img_path.name}"
        right = f"color:{rec['color'] or '-'}  shape:{rec['shape'] or '-'}"
        canvas, _ = render_frame(im, show_help, font_scale, left, right, fit_mode=fit_mode)

        cv2.imshow(WIN_NAME, canvas)
        k = cv2.waitKey(0) & 0xFF

        if k in (27,):  # ESC
            break
        elif k in (ord('h'), ord('H')):
            show_help = not show_help
        elif k in (ord('f'), ord('F')):
            fit_mode = not fit_mode
        elif k in (ord('+'), ord('=')):
            font_scale = min(1.6, font_scale + 0.1)
        elif k in (ord('-'), ord('_')):
            font_scale = max(0.4, font_scale - 0.1)
        elif k in (ord('n'), ord('N'), 83):  # 83 = Right arrow
            idx = min(idx+1, len(imgs)-1)
        elif k in (ord('b'), ord('B'), 81):  # 81 = Left arrow
            idx = max(idx-1, 0)
        elif k in (ord('c'), ord('C')):
            labels[img_path.name] = {"color":"", "shape":""}
        elif k in (ord('s'), ord('S')):
            # write CSV snapshot + state
            items = []
            for p in imgs:
                r = labels.get(p.name, {"color":"", "shape":""})
                items.append([p.name, r["color"], r["shape"]])
            save_csv(out_csv, items)
            st["index"] = idx; st["labels"] = labels
            state_path.write_text(json.dumps(st, indent=2))
            print(f"[saved] {out_csv}  state:{state_path}")
        else:
            ch = chr(k) if 32 <= k <= 126 else ""
            # set color
            if ch in COLOR_KEYS:
                ci = COLOR_KEYS.index(ch)
                labels[img_path.name] = {"color": COLORS[ci], "shape": labels.get(img_path.name, {}).get("shape","")}
            # set shape
            elif ch in SHAPE_KEYS:
                si = SHAPE_KEYS.index(ch)
                labels[img_path.name] = {"color": labels.get(img_path.name, {}).get("color",""), "shape": SHAPES[si]}
            # auto-advance when complete
            rec2 = labels.get(img_path.name, {"color":"", "shape":""})
            if rec2["color"] and rec2["shape"]:
                idx = min(idx+1, len(imgs)-1)

    # Final save on exit
    items = []
    for p in imgs:
        r = labels.get(p.name, {"color":"", "shape":""})
        items.append([p.name, r["color"], r["shape"]])
    save_csv(out_csv, items)
    st["index"] = idx; st["labels"] = labels
    state_path.write_text(json.dumps(st, indent=2))
    cv2.destroyAllWindows()
    print(f"[done] CSV: {out_csv}  state: {state_path}")
    print("[tip] You can edit the CSV in a spreadsheet if needed.")
if __name__ == "__main__":
    main()
