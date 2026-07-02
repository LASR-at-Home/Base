"""Name a detected object crop with a local VLM (Ollama, e.g. gemma3:4b).
Talks to Ollama HTTP API via stdlib urllib — no extra deps, no lasr_vlm venv.
Ollama must be running at `host`; the model auto-pulls on first use."""
import json
import base64
import urllib.request

import cv2
import os
import argparse
import sys

_PROMPT = (
    "You are trying to find ONE laundry basket in the room. Look at the image and reply "
    "whether there is a laundry basket in the image. The laundry basket is grey, made of plastic, and is porous (has holes in its structure). Reply"
    "exactly Yes or No. If you are not sure, reply No."
)

def classify_crop(
    rgb_bgr,
    box_xywh_center,
    candidates=None,
    *,
    model="gemma3:4b",
    host="http://localhost:11434",
    timeout=60.0,
    pad=0.12,
):
    """VLM label for the crop at box (cx,cy,w,h) CENTRE in rgb_bgr, or None."""
    # cands = candidates or CANDIDATES
    cands = ["laundry basket"]
    h_img, w_img = rgb_bgr.shape[:2]
    cx, cy, w, h = box_xywh_center
    px, py = w * pad, h * pad
    x1 = int(max(0, cx - w / 2 - px))
    y1 = int(max(0, cy - h / 2 - py))
    x2 = int(min(w_img, cx + w / 2 + px))
    y2 = int(min(h_img, cy + h / 2 + py))
    if x2 <= x1 or y2 <= y1:
        return None
    crop = rgb_bgr[y1:y2, x1:x2]
    ok, buf = cv2.imencode(".jpg", crop)
    if not ok:
        return None
    img_b64 = base64.b64encode(buf.tobytes()).decode("utf-8")
    payload = {
        "model": model,
        "messages": [{
            "role": "user",
            "content": _PROMPT.format(labels=", ".join(cands)),
            "images": [img_b64],
        }],
        "stream": False,
        "options": {"temperature": 0.0},
    }
    req = urllib.request.Request(
        host.rstrip("/") + "/api/chat",
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"},
    )
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            resp = json.loads(r.read().decode("utf-8"))
        text = (resp.get("message", {}).get("content", "") or "").strip().lower()
    except Exception:
        return None
    if not text:
        return None
    for c in cands:
        if text == c:
            return c
    for c in cands:
        if c in text or text in c:
            return c
    return text.split("\n")[0][:40]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Classify a crop in an image using a local VLM."
    )
    # parser.add_argument("--image", help="Path to the image file.")
    parser.add_argument(
        "--box",
        nargs=4,
        type=float,
        help="Bounding box as cx cy w h (center x, center y, width, height).",
    )
    parser.add_argument(
        "--model", default="gemma3:4b", help="Ollama model name (default: gemma3:4b)."
    )
    parser.add_argument(
        "--host", default="http://localhost:11434", help="Ollama host URL."
    )
    args = parser.parse_args()

    current_dir = os.getcwd()
    image_dir = os.path.join(current_dir, "test_images/not_laundry_basket")
    for filename in os.listdir(image_dir):
        if filename.lower().endswith((".png", ".jpg", ".jpeg")):
            image_path = os.path.join(image_dir, filename)
            if not os.path.isfile(image_path):
                print(f"Image file not found: {image_path}", file=sys.stderr)
                sys.exit(1)

            img = cv2.imread(image_path)
            if img is None:
                print(f"Failed to read image", file=sys.stderr)
                sys.exit(1)

            # Set bounding box as full image if not provided
            if args.box is None:
                h, w = img.shape[:2]
                args.box = [0.5 * w, 0.5 * h, w, h]  # cx, cy, w, h

            label = classify_crop(img, args.box, model=args.model, host=args.host)
            print(f"Predicted label for image {filename}: {label}")