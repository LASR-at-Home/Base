"""Detection names and an optional yes/no check by a local VLM (Ollama, stdlib urllib, no extra deps).

dino_name(): Grounding DINO returns the phrase it matched ("red coca cola"), sometimes in word
  pieces ("pring"). Map it to the query it came from, then to that query's short name from the
  dictionary: "red coca cola can" -> "coke can".
verify_crop(): ask the VLM "is this a <name>?" -> True / False / None (no answer).
rename_crop(): after a "no", the listed name the VLM picks for the crop AND confirms, or None.
moondream is not usable: it ignores the image ("iced tea" for every crop).
"""
import json
import base64
import urllib.request

import cv2

DEFAULT_MODEL = "qwen2.5vl:3b"
CANDIDATES = ["coke can", "paper cup", "water bottle", "granola box", "pringles tube"]


def dino_name(phrase, queries, names=None):
    """The query sharing most words with DINO's phrase, as its short name from `names`
    (dict query -> name); "" if the phrase matches no query."""
    words = {w.strip("#") for w in (phrase or "").lower().split()}
    words = {w for w in words if len(w) >= 3}

    def score(q):
        qw = q.lower().split()
        return sum(1 for w in words if any(w == x or (len(w) >= 4 and w in x) for x in qw))

    best = max(queries, key=score, default=None)
    if best is None or score(best) == 0:
        return ""
    return (names or {}).get(best, best)


def yes_no(ok):
    return "yes" if ok else ("no" if ok is False else "?")


def _crop_b64(rgb_bgr, box_xywh_center, pad):
    h_img, w_img = rgb_bgr.shape[:2]
    cx, cy, w, h = box_xywh_center
    px, py = w * pad, h * pad
    x1 = int(max(0, cx - w / 2 - px))
    y1 = int(max(0, cy - h / 2 - py))
    x2 = int(min(w_img, cx + w / 2 + px))
    y2 = int(min(h_img, cy + h / 2 + py))
    if x2 <= x1 or y2 <= y1:
        return None
    ok, buf = cv2.imencode(".jpg", rgb_bgr[y1:y2, x1:x2])
    return base64.b64encode(buf.tobytes()).decode("utf-8") if ok else None


def _ask(img_b64, prompt, model, host, timeout):
    payload = {
        "model": model,
        "messages": [{"role": "user", "content": prompt, "images": [img_b64]}],
        "stream": False,
        # small context = less VRAM, so Ollama keeps the vision part on the GPU next to DINO
        "options": {"temperature": 0.0, "num_ctx": 2048},
    }
    req = urllib.request.Request(
        host.rstrip("/") + "/api/chat",
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"},
    )
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            resp = json.loads(r.read().decode("utf-8"))
        return resp.get("message", {}).get("content", "") or ""
    except Exception:
        return None


def verify_crop(rgb_bgr, box_xywh_center, name, *, model=DEFAULT_MODEL,
                host="http://localhost:11434", timeout=60.0, pad=0.12):
    """Ask the VLM whether the crop at box (cx,cy,w,h) CENTRE is a `name`: True, False, or None."""
    if not name:
        return None
    img_b64 = _crop_b64(rgb_bgr, box_xywh_center, pad)
    if img_b64 is None:
        return None
    text = _ask(img_b64, f"Is the object in this image a {name}? Answer only yes or no.",
                model, host, timeout)
    t = (text or "").strip().lower().lstrip(" .,'\"`*")
    if t.startswith("yes"):
        return True
    if t.startswith("no"):
        return False
    return None


def rename_crop(rgb_bgr, box_xywh_center, old, candidates, *, model=DEFAULT_MODEL,
                host="http://localhost:11434", timeout=60.0):
    """After a "no" for `old`: the name from `candidates` the VLM picks for this crop and then
    confirms with "yes". None if it picks nothing, picks `old` again, or does not confirm."""
    new = classify_crop(rgb_bgr, box_xywh_center, candidates, model=model, host=host, timeout=timeout)
    if not new or new == (old or "").strip().lower():
        return None
    ok = verify_crop(rgb_bgr, box_xywh_center, new, model=model, host=host, timeout=timeout)
    return new if ok is True else None


def classify_crop(rgb_bgr, box_xywh_center, candidates=None, *, model=DEFAULT_MODEL,
                  host="http://localhost:11434", timeout=60.0, pad=0.12):
    """The one listed name the VLM gives for the crop, or None."""
    cands = [c.strip().lower() for c in (candidates or CANDIDATES) if c and c.strip()]
    cands = [c for c in dict.fromkeys(cands) if c != "unknown"]
    img_b64 = _crop_b64(rgb_bgr, box_xywh_center, pad) if cands else None
    if img_b64 is None:
        return None
    text = _ask(img_b64, "You are labelling ONE object for a robot. Reply with EXACTLY ONE label "
                "from this list, lowercase, and nothing else:\n" + ", ".join(cands + ["unknown"])
                + "\nIf none clearly fits, reply 'unknown'.", model, host, timeout)
    t = (text or "").strip().lower().split("\n")[0].strip(" .,'\"`*")
    if t in cands:
        return t
    hits = [c for c in cands if c in t]
    hits = [c for c in hits if not any(c != o and c in o for o in hits)]
    return hits[0] if len(hits) == 1 else None
