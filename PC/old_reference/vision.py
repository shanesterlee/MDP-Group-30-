#!/usr/bin/env python3
import argparse
import os
import re
import cv2
import requests
from inference_sdk import InferenceHTTPClient

# ---- Roboflow defaults (used if --use-server is NOT provided) ----
API_URL  = "https://serverless.roboflow.com"
API_KEY  = "FQ9ipHZtY3wdsxRHp6pZ"
MODEL_ID = "mdp-jau85/7"

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


# ──────────────────────────────────────────────────────────────────────────────
# Drawing
# ──────────────────────────────────────────────────────────────────────────────

def draw_predictions(image_path: str, result: dict, out_path: str, conf_thresh: float = 0.2):
    # ensure output dir exists
    out_dir = os.path.dirname(out_path) or "."
    os.makedirs(out_dir, exist_ok=True)

    img = cv2.imread(image_path)
    if img is None:
        raise RuntimeError(f"Failed to read image: {image_path}")

    preds = _extract_predictions(result)
    if not isinstance(preds, list):
        preds = []

    recognised_ids = []
    drawn = 0

    for p in preds:
        conf = float(p.get("confidence", 0))
        if conf < conf_thresh:
            continue

        # Roboflow center-format bbox -> corner coords
        x = float(p.get("x", 0)); y = float(p.get("y", 0))
        w = float(p.get("width", 0)); h = float(p.get("height", 0))
        x1 = int(x - w / 2); y1 = int(y - h / 2)
        x2 = int(x + w / 2); y2 = int(y + h / 2)

        # clamp
        H, W = img.shape[:2]
        x1 = max(0, min(x1, W - 1)); x2 = max(0, min(x2, W - 1))
        y1 = max(0, min(y1, H - 1)); y2 = max(0, min(y2, H - 1))

        cls_raw = p.get("class", "obj")
        obj_id = class_to_id(cls_raw)

        # pretty symbol for overlay
        if isinstance(obj_id, int):
            pretty = ID_TO_PRETTY.get(obj_id, str(cls_raw))
        else:
            pretty = str(cls_raw)

        if obj_id != "NA":
            recognised_ids.append(obj_id)

        # draw
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        label = f"{str(obj_id)}_{pretty}_{conf:.2f}"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        y_top = max(0, y1 - th - 8)
        cv2.rectangle(img, (x1, y_top), (x1 + tw + 6, y1), (0, 255, 0), -1)
        cv2.putText(img, label, (x1 + 3, max(0, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        drawn += 1

    ok = cv2.imwrite(out_path, img)
    if not ok:
        raise RuntimeError(f"Failed to write {out_path}")

    print(f"[✓] Saved annotated image → {out_path}")
    print(f"[i] Detections drawn (≥ {conf_thresh}): {drawn}")

    if recognised_ids:
        print(f"[✓] Image is VALID (IDs recognised: {sorted(set(recognised_ids))})")
    else:
        print("[i] Image NOT valid (no mapped IDs recognised)")


# ──────────────────────────────────────────────────────────────────────────────
# CLI
# ──────────────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="Run inference (Roboflow or local server) and draw boxes with Image IDs.")
    ap.add_argument("image", help="Path to input image (e.g., test.jpg)")
    ap.add_argument("out", help="Path to output annotated image (e.g., out.jpg)")
    ap.add_argument("--conf", type=float, default=0.2, help="Confidence threshold for drawing")
    ap.add_argument("--use-server", default="", help="Optional: URL to local server /infer (e.g., http://127.0.0.1:8000/infer)")
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

    # Draw + print
    draw_predictions(args.image, result, args.out, conf_thresh=args.conf)

if __name__ == "__main__":
    main()
