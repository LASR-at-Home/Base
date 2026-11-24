"""Helpers for DetectObjectsOpenVocab, without ROS: Grounding DINO phrase -> label, class-agnostic
NMS, depth at a box, and a check of each crop by a local VLM (Ollama HTTP API, stdlib only).

VLM: qwen2.5vl:3b works (ollama pull qwen2.5vl:3b). moondream does not: it ignores the image.
Boxes are [cx, cy, w, h] in pixels, as returned by lasr_vision_open_vocabulary.
"""

import base64
import json
import urllib.request

import numpy as np
import cv2

DEFAULT_VLM_MODEL = "qwen2.5vl:3b"
DEFAULT_VLM_HOST = "http://localhost:11434"
_STOP = {"a", "an", "the", "of", "and", "with", "on", "in"}


def label_for(phrase, queries, labels=None):
    """Grounding DINO returns the words of the prompt it matched ("red coca cola"), sometimes as
    word pieces ("pring"). Returns the query sharing most words with the phrase, as its label
    (labels: query -> label), or None if the phrase matches no query."""
    words = {w.strip("#") for w in (phrase or "").lower().split()} - _STOP

    def score(q):
        qw = q.lower().split()
        return sum(
            1
            for w in words
            if w and any(w == x or (len(w) >= 4 and w in x) for x in qw)
        )

    best = max(queries, key=score, default=None)
    if best is None or score(best) == 0:
        return None
    return (labels or {}).get(best, best)


def iou(a, b):
    """IoU of two [cx, cy, w, h] boxes."""
    ax1, ay1, ax2, ay2 = (
        a[0] - a[2] / 2,
        a[1] - a[3] / 2,
        a[0] + a[2] / 2,
        a[1] + a[3] / 2,
    )
    bx1, by1, bx2, by2 = (
        b[0] - b[2] / 2,
        b[1] - b[3] / 2,
        b[0] + b[2] / 2,
        b[1] + b[3] / 2,
    )
    inter = max(0.0, min(ax2, bx2) - max(ax1, bx1)) * max(
        0.0, min(ay2, by2) - max(ay1, by1)
    )
    union = a[2] * a[3] + b[2] * b[3] - inter
    return inter / union if union > 0 else 0.0


def nms(dets, iou_threshold=0.5):
    """dets: [(label, confidence, box)]. Keeps the most confident box of each overlapping group,
    whatever its label (DINO often puts two labels on one object)."""
    kept = []
    for d in sorted(dets, key=lambda x: x[1], reverse=True):
        if all(iou(d[2], k[2]) < iou_threshold for k in kept):
            kept.append(d)
    return kept


def box_depth(depth_m, box, frac=0.25):
    """Median valid depth (metres) in the middle `frac` of the box, or None."""
    h, w = depth_m.shape[:2]
    cx, cy, bw, bh = box
    rx, ry = max(1, int(bw * frac / 2)), max(1, int(bh * frac / 2))
    x1, x2 = int(np.clip(cx - rx, 0, w - 1)), int(np.clip(cx + rx, 0, w - 1)) + 1
    y1, y2 = int(np.clip(cy - ry, 0, h - 1)), int(np.clip(cy + ry, 0, h - 1)) + 1
    patch = depth_m[y1:y2, x1:x2].astype(np.float32).ravel()
    patch = patch[np.isfinite(patch) & (patch > 0.05)]
    return float(np.median(patch)) if patch.size else None


def draw(bgr, dets):
    """Copy of the image with each (label, confidence, box) drawn on it."""
    out = bgr.copy()
    for label, conf, (cx, cy, w, h) in dets:
        p1, p2 = (int(cx - w / 2), int(cy - h / 2)), (int(cx + w / 2), int(cy + h / 2))
        cv2.rectangle(out, p1, p2, (0, 200, 0), 2)
        text = f"{label} {conf:.2f}"
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        y = max(th + 6, p1[1])
        cv2.rectangle(out, (p1[0], y - th - 6), (p1[0] + tw + 6, y), (0, 200, 0), -1)
        cv2.putText(
            out, text, (p1[0] + 3, y - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2
        )
    return out


# ---- VLM (Ollama) ----


def _crop_b64(bgr, box, pad=0.12):
    h_img, w_img = bgr.shape[:2]
    cx, cy, w, h = box
    x1, y1 = int(max(0, cx - w / 2 - w * pad)), int(max(0, cy - h / 2 - h * pad))
    x2, y2 = int(min(w_img, cx + w / 2 + w * pad)), int(
        min(h_img, cy + h / 2 + h * pad)
    )
    if x2 <= x1 or y2 <= y1:
        return None
    ok, buf = cv2.imencode(".jpg", bgr[y1:y2, x1:x2])
    return base64.b64encode(buf.tobytes()).decode("utf-8") if ok else None


def ask_vlm(
    img_b64, prompt, model=DEFAULT_VLM_MODEL, host=DEFAULT_VLM_HOST, timeout=30.0
):
    """The VLM's answer, or None if Ollama is not reachable / times out."""
    payload = {
        "model": model,
        "messages": [{"role": "user", "content": prompt, "images": [img_b64]}],
        "stream": False,
        # small context = less VRAM, so the vision part stays on the GPU next to DINO
        "options": {"temperature": 0.0, "num_ctx": 2048},
    }
    req = urllib.request.Request(
        host.rstrip("/") + "/api/chat",
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"},
    )
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return (
                json.loads(r.read().decode("utf-8"))
                .get("message", {})
                .get("content", "")
                or ""
            )
    except Exception:
        return None


def vlm_is(bgr, box, label, **kw):
    """Is the crop a `label`? True / False / None (no answer)."""
    img = _crop_b64(bgr, box)
    if img is None or not label:
        return None
    text = ask_vlm(
        img, f"Is the object in this image a {label}? Answer only yes or no.", **kw
    )
    if text is None:
        return None
    t = text.strip().lower().lstrip(" .,'\"`*")
    return True if t.startswith("yes") else False if t.startswith("no") else None


def vlm_pick(bgr, box, labels, **kw):
    """The one label from `labels` the VLM gives the crop, or None."""
    cands = [
        c
        for c in dict.fromkeys(l.strip().lower() for l in labels if l and l.strip())
        if c != "unknown"
    ]
    img = _crop_b64(bgr, box) if cands else None
    if img is None:
        return None
    text = ask_vlm(
        img,
        "You are labelling ONE object for a robot. Reply with EXACTLY ONE label from this "
        "list, lowercase, and nothing else:\n"
        + ", ".join(cands + ["unknown"])
        + "\nIf none clearly fits, reply 'unknown'.",
        **kw,
    )
    t = (text or "").strip().lower().split("\n")[0].strip(" .,'\"`*")
    if t in cands:
        return t
    hits = [c for c in cands if c in t]
    hits = [c for c in hits if not any(c != o and c in o for o in hits)]
    return hits[0] if len(hits) == 1 else None
