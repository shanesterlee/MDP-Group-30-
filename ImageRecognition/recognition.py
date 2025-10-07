#!/usr/bin/env python3

import argparse
import os
import re
import cv2
import math
import numpy as np
import requests
from inference_sdk import InferenceHTTPClient

# ---- Roboflow defaults (only used if --use-server is NOT provided) ----
API_URL  = "https://serverless.roboflow.com"
API_KEY  = "FQ9ipHZtY3wdsxRHp6pZ"
MODEL_ID = "mdp-jau85/7"

# ──────────────────────────────────────────────────────────────────────────────
# OFFICIAL MAPPING (from your rules image)
# ─ numbers: 1..9 → 11..19
# ─ letters subset: A..H → 20..27, S..Z → 28..35 (V = 31)
# ─ arrows/stop: up=36, down=37, right=38, left=39, stop=40
# ──────────────────────────────────────────────────────────────────────────────
BASE_ID_MAP = {
    # digits
    "1": 11, "2": 12, "3": 13, "4": 14, "5": 15, "6": 16, "7": 17, "8": 18, "9": 19,
    # letters subset
    "A": 20, "B": 21, "C": 22, "D": 23, "E": 24, "F": 25, "G": 26, "H": 27,
    "S": 28, "T": 29, "U": 30, "V": 31, "W": 32, "X": 33, "Y": 34, "Z": 35,
    # arrows + stop
    "up": 36, "down": 37, "right": 38, "left": 39, "stop": 40,
}
ID_TO_PRETTY = {v: k for k, v in BASE_ID_MAP.items()}

WORDS_TO_DIGITS = {
    "one": "1", "two": "2", "three": "3", "four": "4", "five": "5",
    "six": "6", "seven": "7", "eight": "8", "nine": "9"
}

# ──────────────────────────────────────────────────────────────────────────────
# Normalization and mapping
# ──────────────────────────────────────────────────────────────────────────────
def _normalize_label(raw) -> str:
    """Normalize model class names to canonical keys used in BASE_ID_MAP.
    Accepts numeric strings like '31' (pass-through), 'Alphabet V',
    'Right Arrow', 'three', 'v', 'Stop', etc.
    """
    if raw is None:
        return ""

    if isinstance(raw, (int, float)):
        raw = str(int(raw))

    s0 = str(raw).strip()

    # purely digits (e.g., "31"): treat as already an ID
    if s0.isdigit():
        return s0

    # "Alphabet V", "alphabet_v", etc.
    m = re.match(r"^alphabet[\s_:\-]*([a-zA-Z])$", s0, flags=re.IGNORECASE)
    if m:
        return m.group(1).upper()

    # remove trailing "arrow"
    s1 = re.sub(r"[\s_-]*arrow$", "", s0, flags=re.IGNORECASE).strip()
    lw = s1.lower()

    # word numbers -> digits
    if lw in WORDS_TO_DIGITS:
        return WORDS_TO_DIGITS[lw]

    # single letter
    if len(s1) == 1 and s1.isalpha():
        return s1.upper()

    # canonical arrow words or stop
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
    key = _normalize_label(cls)
    if not key:
        return "NA"
    if key.isdigit():
        return int(key)  # accept numeric IDs directly
    return BASE_ID_MAP.get(key, "NA")


# ──────────────────────────────────────────────────────────────────────────────
# Inference callers
# ──────────────────────────────────────────────────────────────────────────────
def call_roboflow_sdk(image_path: str):
    client = InferenceHTTPClient(api_url=API_URL, api_key=API_KEY)
    return client.infer(image_path, model_id=MODEL_ID)

def call_local_server(image_path: str, server_url: str):
    with open(image_path, "rb") as f:
        files = {"file": (os.path.basename(image_path), f, "image/jpeg")}
        r = requests.post(server_url, files=files, timeout=30)
    r.raise_for_status()
    return r.json()

def _extract_predictions(result: dict):
    """Supports both {predictions:[...]} and {result:{predictions:[...]}} shapes."""
    return (result.get("predictions")
            or result.get("result", {}).get("predictions")
            or [])



def save_rep_frame(img_bgr, save_dir, symbol_key):
    """Save one representative RAW frame (full frame with one bbox drawn)."""
    os.makedirs(save_dir, exist_ok=True)
    cv2.imwrite(os.path.join(save_dir, f"{symbol_key}.jpg"), img_bgr)

def make_tiled_gallery(src_dir, out_path, tile_w=360, cols=3, pad=8):
    files = [os.path.join(src_dir, f) for f in os.listdir(src_dir)
             if f.lower().endswith((".jpg", ".jpeg", ".png"))]
    if not files:
        return False

    imgs = []
    for fp in files:
        im = cv2.imread(fp)
        if im is None:
            continue
        # resize by width; preserve aspect
        h, w = im.shape[:2]
        new_h = int(h * (tile_w / float(w)))
        imr = cv2.resize(im, (tile_w, new_h))
        imgs.append((fp, imr))

    if not imgs:
        return False

    rows = math.ceil(len(imgs) / cols)
    max_h_per_row = []
    for r in range(rows):
        row_imgs = imgs[r*cols:(r+1)*cols]
        max_h_per_row.append(max(im.shape[0] for _, im in row_imgs))

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
# Drawing + saving representatives + tiling (UPDATED)
# ──────────────────────────────────────────────────────────────────────────────
def draw_predictions(image_path: str,
                     result: dict,
                     out_path: str,
                     conf_thresh: float = 0.2,
                     gallery_dir: str = "./run_artifacts/raw_clips",
                     gallery_out: str = "./run_artifacts/raw_gallery.jpg"):
    # ensure output dir exists
    out_dir = os.path.dirname(out_path) or "."
    os.makedirs(out_dir, exist_ok=True)

    base_img = cv2.imread(image_path)
    if base_img is None:
        raise RuntimeError(f"Failed to read image: {image_path}")

    # We'll draw the final multi-box output on a copy
    out_img = base_img.copy()

    preds = _extract_predictions(result)
    if not isinstance(preds, list):
        preds = []

    recognised_ids = []
    drawn = 0
    saved_symbols = set()  # keep only one representative per symbol (ID_pretty)

    for p in preds:
        conf = float(p.get("confidence", 0))
        if conf < conf_thresh:
            continue

        # center-format bbox -> corners
        x = float(p.get("x", 0)); y = float(p.get("y", 0))
        w = float(p.get("width", 0)); h = float(p.get("height", 0))
        x1 = int(x - w / 2); y1 = int(y - h / 2)
        x2 = int(x + w / 2); y2 = int(y + h / 2)

        # clamp
        H, W = out_img.shape[:2]
        x1 = max(0, min(x1, W - 1)); x2 = max(0, min(x2, W - 1))
        y1 = max(0, min(y1, H - 1)); y2 = max(0, min(y2, H - 1))

        cls_raw = p.get("class", "obj")
        obj_id = class_to_id(cls_raw)

        # pretty for middle token
        pretty = str(cls_raw)
        if isinstance(obj_id, int):
            pretty = ID_TO_PRETTY.get(obj_id, str(cls_raw))

        if obj_id != "NA":
            recognised_ids.append(obj_id)

        # draw on final output image
        cv2.rectangle(out_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        label = f"{str(obj_id)}_{pretty}_{conf:.2f}"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        y_top = max(0, y1 - th - 8)
        cv2.rectangle(out_img, (x1, y_top), (x1 + tw + 6, y1), (0, 255, 0), -1)
        cv2.putText(out_img, label, (x1 + 3, max(0, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        drawn += 1

        # save ONE representative RAW frame per symbol (with only this bbox drawn)
        if isinstance(obj_id, int):
            symbol_key = f"{obj_id}_{pretty}"
        else:
            symbol_key = f"{obj_id}_{pretty}"

        if symbol_key not in saved_symbols:
            single = base_img.copy()
            cv2.rectangle(single, (x1, y1), (x2, y2), (0, 255, 0), 2)
            (tw2, th2), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            y_top2 = max(0, y1 - th2 - 8)
            cv2.rectangle(single, (x1, y_top2), (x1 + tw2 + 6, y1), (0, 255, 0), -1)
            cv2.putText(single, label, (x1 + 3, max(0, y1 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
            save_rep_frame(single, gallery_dir, symbol_key)
            saved_symbols.add(symbol_key)

    # write the multi-detection annotated image
    if not cv2.imwrite(out_path, out_img):
        raise RuntimeError(f"Failed to write {out_path}")
    print(f"[✓] Saved annotated image → {out_path}")
    print(f"[i] Detections drawn (≥ {conf_thresh}): {drawn}")

    # build tiled gallery (if we saved any reps)
    if saved_symbols:
        ok = make_tiled_gallery(gallery_dir, gallery_out)
        if ok:
            print(f"[✓] Saved RAW tiled gallery → {gallery_out}")
        else:
            print("[i] No gallery created (no valid frames).")

    if recognised_ids:
        print(f"[✓] Image is VALID (IDs recognised: {sorted(set(recognised_ids))})")
    else:
        print("[i] Image NOT valid (no mapped IDs recognised)")


# ──────────────────────────────────────────────────────────────────────────────
# CLI
# ──────────────────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description="Run inference (Roboflow or local server), draw boxes, and build a RAW tiled gallery.")
    ap.add_argument("image", help="Path to input image (e.g., test.jpg)")
    ap.add_argument("out", help="Path to output annotated image (e.g., out.jpg)")
    ap.add_argument("--conf", type=float, default=0.2, help="Confidence threshold for drawing")
    ap.add_argument("--use-server", default="", help="Optional: URL to local server /infer (e.g., http://127.0.0.1:8000/infer)")
    ap.add_argument("--gallery-dir", default="./run_artifacts/raw_clips", help="Directory to store representative RAW frames")
    ap.add_argument("--gallery-out", default="./run_artifacts/raw_gallery.jpg", help="Final tiled gallery image path")
    args = ap.parse_args()

    # Call inference
    try:
        if args.use_server:
            result = call_local_server(args.image, args.use_server)
        else:
            result = call_roboflow_sdk(args.image)
    except Exception as e:
        print("Error calling inference endpoint:", e)
        return

    # Draw + print + save gallery
    draw_predictions(
        args.image, result, args.out,
        conf_thresh=args.conf,
        gallery_dir=args.gallery_dir,
        gallery_out=args.gallery_out
    )

if __name__ == "__main__":
    main()

# To run this code:
# python recognition.py t9.jpg out.jpg --conf 0.6 --gallery-dir ./run_artifacts/raw_clips --gallery-out ./run_artifacts/raw_gallery.jpg
