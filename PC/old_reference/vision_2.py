#!/usr/bin/env python3
import argparse
import os
import re
import math
import numpy as np
import cv2
from ultralytics import YOLO

# ──────────────────────────────────────────────────────────────────────────────
# OFFICIAL MAPPING (from your table)
# ─ numbers: 1..9 → 11..19
# ─ letters subset: A..H → 20..27, S..Z → 28..35
# ─ arrows/stop: up=36, down=37, right=38, left=39, stop=40
# ──────────────────────────────────────────────────────────────────────────────

BASE_ID_MAP = {
    # digits
    "1": 11, "2": 12, "3": 13, "4": 14, "5": 15, "6": 16, "7": 17, "8": 18, "9": 19,

    # letters subset (case-insensitive keys will be normalized to upper)
    "A": 20, "B": 21, "C": 22, "D": 23, "E": 24, "F": 25, "G": 26, "H": 27,
    "S": 28, "T": 29, "U": 30, "V": 31, "W": 32, "X": 33, "Y": 34, "Z": 35,

    # arrows + stop (lowercase canonical keys)
    "up": 36, "down": 37, "right": 38, "left": 39, "stop": 40,
}

# Reverse map for pretty labels (ID → symbol)
ID_TO_PRETTY = {
    v: k for k, v in BASE_ID_MAP.items()
}

WORDS_TO_DIGITS = {
    "one": "1", "two": "2", "three": "3", "four": "4", "five": "5",
    "six": "6", "seven": "7", "eight": "8", "nine": "9"
}

def _normalize_label(raw) -> str:
    """Normalize model class names to canonical keys used in BASE_ID_MAP.

    Accepts:
      - numeric strings like '31'  → '31' (pass-through, meaning model already outputs IDs)
      - 'Alphabet V', 'alphabet_v' → 'V'
      - 'Right Arrow', 'rightarrow' → 'right'
      - 'three' → '3'
      - letters 'v' → 'V'
      - 'Stop' → 'stop'
    """
    if raw is None:
        return ""

    # handle ints/floats from some servers
    if isinstance(raw, (int, float)):
        raw = str(int(raw))

    s0 = str(raw).strip()

    # purely digits (e.g., '31'): treat as already an ID; return as-is
    if s0.isdigit():
        return s0

    # Alphabet_X / Alphabet X
    m = re.match(r"^alphabet[\s_:\-]*([a-zA-Z])$", s0, flags=re.IGNORECASE)
    if m:
        return m.group(1).upper()

    # remove trailing 'arrow'
    s1 = re.sub(r"[\s_-]*arrow$", "", s0, flags=re.IGNORECASE).strip()

    # word numbers -> digits
    lw = s1.lower()
    if lw in WORDS_TO_DIGITS:
        return WORDS_TO_DIGITS[lw]

    # single letter
    if len(s1) == 1 and s1.isalpha():
        return s1.upper()

    # canonical arrow words or stop
    lw = lw.lower()
    if lw in {"up", "down", "left", "right", "stop"}:
        return lw

    # compact forms
    s2 = re.sub(r"[^a-zA-Z0-9]", "", s0).lower()
    if s2 in WORDS_TO_DIGITS:
        return WORDS_TO_DIGITS[s2]
    if s2 in {"uparrow", "downarrow", "leftarrow", "rightarrow"}:
        return s2.replace("arrow", "")
    if s2 == "stop":
        return "stop"

    return ""


def class_to_id(cls):
    """Return numeric ID or 'NA' if we cannot map."""
    key = _normalize_label(cls)
    if not key:
        return "NA"

    # If the model already outputs numeric IDs, accept them directly.
    if key.isdigit():
        return int(key)

    # Else map via table (letters/arrows/stop)
    return BASE_ID_MAP.get(key, "NA")


# ──────────────────────────────────────────────────────────────────────────────
# Gallery helpers
# ──────────────────────────────────────────────────────────────────────────────

def save_rep_frame(img_bgr, save_dir, symbol_key):
    """Save individual detection frame."""
    os.makedirs(save_dir, exist_ok=True)
    cv2.imwrite(os.path.join(save_dir, f"{symbol_key}.jpg"), img_bgr)


def make_tiled_gallery(src_dir, out_path, tile_w=360, cols=3, pad=8):
    """Create a tiled gallery from individual frames."""
    files = [os.path.join(src_dir, f) for f in os.listdir(src_dir)
             if f.lower().endswith((".jpg", ".jpeg", ".png"))]
    if not files:
        return False
    
    imgs = []
    for fp in files:
        im = cv2.imread(fp)
        if im is None:
            continue
        h, w = im.shape[:2]
        imr = cv2.resize(im, (tile_w, int(h * tile_w / float(w))))
        imgs.append((fp, imr))
    
    if not imgs:
        return False
    
    rows = math.ceil(len(imgs) / cols)
    max_h_per_row = [max(im.shape[0] for _, im in imgs[r*cols:(r+1)*cols]) for r in range(rows)]
    canvas_w = cols * tile_w + (cols + 1) * pad
    canvas_h = sum(max_h_per_row) + (rows + 1) * pad
    canvas = np.full((canvas_h, canvas_w, 3), 255, dtype=np.uint8)
    
    y = pad
    idx = 0
    for r in range(rows):
        x = pad
        for c in range(cols):
            if idx >= len(imgs):
                break
            _, im = imgs[idx]
            h, w = im.shape[:2]
            canvas[y:y+h, x:x+w] = im
            x += tile_w + pad
            idx += 1
        y += max_h_per_row[r] + pad
    
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    cv2.imwrite(out_path, canvas)
    return True


# ──────────────────────────────────────────────────────────────────────────────
# Local YOLOv8 Inference
# ──────────────────────────────────────────────────────────────────────────────

def call_local_yolo(model_path: str, image_path: str, conf: float = 0.6, imgsz: int = 640, device=None):
    """Run inference using local YOLOv8 model."""
    model = YOLO(model_path)
    results = model.predict(source=image_path, imgsz=imgsz, conf=conf, verbose=False, device=device)
    return results


# ──────────────────────────────────────────────────────────────────────────────
# Drawing
# ──────────────────────────────────────────────────────────────────────────────

def draw_predictions(image_path: str, results, conf_thresh: float = 0.6,
                    gallery_dir: str = "./run_artifacts/raw_clips",
                    gallery_out: str = "./run_artifacts/raw_gallery.jpg"):
    """Extract detections and save individual frames + tiled gallery."""
    # Read base image
    base = cv2.imread(image_path)
    if base is None:
        raise RuntimeError(f"Failed to read image: {image_path}")

    recognised_ids = []
    saved_symbols = set()
    drawn = 0

    for r in results:
        boxes = r.boxes
        names = r.names  # dict: class_id -> class_name
        if boxes is None or len(boxes) == 0:
            continue

        for i in range(len(boxes)):
            # Get bbox coordinates, class, and confidence
            xyxy = boxes.xyxy[i].cpu().numpy().tolist()
            x1, y1, x2, y2 = map(int, xyxy)
            cls_id = int(boxes.cls[i].item())
            conf = float(boxes.conf[i].item())
            cls_name = names.get(cls_id, str(cls_id))

            # Clamp coordinates
            H, W = base.shape[:2]
            x1 = max(0, min(x1, W - 1))
            x2 = max(0, min(x2, W - 1))
            y1 = max(0, min(y1, H - 1))
            y2 = max(0, min(y2, H - 1))

            obj_id = class_to_id(cls_name)

            # pretty symbol for overlay
            if isinstance(obj_id, int):
                pretty = ID_TO_PRETTY.get(obj_id, str(cls_name))
            else:
                pretty = str(cls_name)

            if obj_id != "NA":
                recognised_ids.append(obj_id)

            drawn += 1
            label = f"{str(obj_id)}_{pretty}_{conf:.2f}"

            # save one representative frame per symbol
            symbol_key = f"{obj_id}_{pretty}"
            if symbol_key not in saved_symbols:
                single = base.copy()
                cv2.rectangle(single, (x1, y1), (x2, y2), (0, 255, 0), 2)
                (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                y_top = max(0, y1 - th - 8)
                cv2.rectangle(single, (x1, y_top), (x1 + tw + 6, y1), (0, 255, 0), -1)
                cv2.putText(single, label, (x1 + 3, max(0, y1 - 6)),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
                save_rep_frame(single, gallery_dir, symbol_key)
                saved_symbols.add(symbol_key)

    print(f"[i] Detections drawn (≥ {conf_thresh}): {drawn}")

    # Create tiled gallery if we saved any individual frames
    if saved_symbols:
        make_tiled_gallery(gallery_dir, gallery_out)

    if recognised_ids:
        print(f"[✓] Image is VALID (IDs recognised: {sorted(set(recognised_ids))})")
    else:
        print("[i] Image NOT valid (no mapped IDs recognised)")


# ──────────────────────────────────────────────────────────────────────────────
# CLI
# ──────────────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="Run local YOLOv8 inference and save individual frames + tiled gallery.")
    ap.add_argument("image", help="Path to input image (e.g., t4.jpg)")
    ap.add_argument("--model", required=True, help="Path to YOLOv8 .pt weights (e.g., v1.pt)")
    ap.add_argument("--conf", type=float, default=0.6, help="Confidence threshold for detection")
    ap.add_argument("--imgsz", type=int, default=640, help="Inference image size")
    ap.add_argument("--device", default=None, help="Device: None (auto), 'cpu', 'cuda', or 'cuda:0'")
    ap.add_argument("--gallery-dir", default="./run_artifacts/raw_clips", help="Where to save individual frames")
    ap.add_argument("--gallery-out", default="./run_artifacts/raw_gallery.jpg", help="Final tiled gallery path")
    args = ap.parse_args()

    # Call inference
    try:
        results = call_local_yolo(args.model, args.image, args.conf, args.imgsz, args.device)
    except Exception as e:
        print("Error calling inference:", e)
        return

    # Process detections and create gallery
    draw_predictions(args.image, results, conf_thresh=args.conf,
                    gallery_dir=args.gallery_dir, gallery_out=args.gallery_out)

if __name__ == "__main__":
    main()