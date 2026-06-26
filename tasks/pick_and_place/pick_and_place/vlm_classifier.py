"""Name a detected object crop with a local VLM (Ollama, e.g. gemma3:4b).
Talks to Ollama HTTP API via stdlib urllib — no extra deps, no lasr_vlm venv.
Ollama must be running at `host`; the model auto-pulls on first use."""
import json
import base64
import urllib.request

import cv2

# Specific product labels the VLM must choose from — EDIT for your items.
CANDIDATES = [
    "water bottle", "iced tea", "coke can", "sprite can", "pringles",
    "red bull", "apple", "banana", "cup", "mug", "bowl", "sponge", "unknown",
]

_PROMPT = (
    "You are labelling ONE grocery item for a robot. Look at the image and reply "
    "with EXACTLY ONE label from this list, lowercase, and nothing else:\n"
    "{labels}\n"
    "If none clearly fits, reply 'unknown'."
)


def classify_crop(
    rgb_bgr,
    box_xywh_center,
    candidates=None,
    *,
    model="moondream",
    host="http://localhost:11434",
    timeout=60.0,
    pad=0.12,
):
    """VLM label for the crop at box (cx,cy,w,h) CENTRE in rgb_bgr, or None."""
    cands = candidates or CANDIDATES
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

    import os
    debug_dir = "/tmp/vlm_crops"
    os.makedirs(debug_dir, exist_ok=True)
    debug_count = len(os.listdir(debug_dir))
    cv2.imwrite(f"{debug_dir}/crop_{debug_count}.jpg", crop)

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