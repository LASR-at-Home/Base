"""
yoloe_test.py — standalone YOLOE tester. No ROS, no package imports; just
ultralytics + OpenCV. Drop this single file anywhere and run it.

Swap model size with --weights:
    yoloe-v8s-seg.pt   (small)
    yoloe-v8m-seg.pt   (medium)        <-- the one to compare
  (the YOLO11 family yoloe-11s-seg.pt / yoloe-11m-seg.pt also works)

USAGE

  1) Visual + speed test on a folder (or single image). Saves annotated copies
     and prints inference speed (ms/image and FPS):

       python yoloe_test.py \
         --weights yoloe-v8m-seg.pt --device cuda \
         --classes "bottle,cup,cereal box,banana" \
         --source ./my_images --save-dir ./out

  2) Add F1 scoring, if you have LabelMe polygon .json labels for the images:

       python yoloe_test.py \
         --weights yoloe-v8m-seg.pt --device cuda \
         --classes "fruit smoothie,crisps,bottle" \
         --source ./images --labels ./labelme_json

Compare S vs M by running it twice with different --weights and reading the
printed speed (and F1, if labels are given).
"""
import argparse
import collections
import json
import time
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLOE

IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}


def build_model(weights: str, device: str, classes: list):
    model = YOLOE(weights)
    model.to(device)
    model.set_classes(classes, model.get_text_pe(classes))  # YOLOE needs text embeddings
    return model


def gather_images(source: str) -> list:
    p = Path(source)
    if p.is_file():
        return [p]
    return sorted(f for f in p.iterdir() if f.suffix.lower() in IMAGE_EXTS)


def polygon_to_box(points):
    xs = [pt[0] for pt in points]
    ys = [pt[1] for pt in points]
    return [min(xs), min(ys), max(xs), max(ys)]


def iou(a, b):
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0, x2 - x1) * max(0, y2 - y1)
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


def _norm(name: str) -> str:
    # match prompt vs label regardless of spaces/underscores/case
    return name.lower().replace("_", " ").strip()


def load_labelme_boxes(labels_dir, classes):
    """image stem -> list of (class, box). Label names are matched to the prompt
    classes by normalising underscores/spaces, so 'fruit_smoothie' == 'fruit smoothie'."""
    norm_to_class = {_norm(c): c for c in classes}
    gt = {}
    for jp in Path(labels_dir).glob("*.json"):
        data = json.load(open(jp))
        boxes = [
            (norm_to_class[_norm(s["label"])], polygon_to_box(s["points"]))
            for s in data.get("shapes", [])
            if s.get("shape_type") == "polygon" and _norm(s["label"]) in norm_to_class
        ]
        if boxes:
            gt[Path(data.get("imagePath", jp.stem)).stem] = boxes
    return gt


def main():
    parser = argparse.ArgumentParser(description="Standalone YOLOE tester")
    parser.add_argument("--weights", default="yoloe-v8m-seg.pt", help="yoloe-v8s-seg.pt or yoloe-v8m-seg.pt")
    parser.add_argument("--device", default="cuda", help="cuda or cpu")
    parser.add_argument("--classes", required=True, help='comma-separated open-vocab prompt, e.g. "bottle,cup"')
    parser.add_argument("--source", required=True, help="image file or folder of images")
    parser.add_argument("--save-dir", default="./yoloe_out", help="where to write annotated images")
    parser.add_argument("--labels", default="", help="optional LabelMe .json dir -> compute F1")
    parser.add_argument("--conf", type=float, default=0.25, help="confidence threshold")
    parser.add_argument("--iou-match", type=float, default=0.4, help="IoU for an F1 match (only with --labels)")
    args = parser.parse_args()

    classes = [c.strip() for c in args.classes.split(",") if c.strip()]
    images = gather_images(args.source)
    if not images:
        print(f"no images found at {args.source}")
        return
    save_dir = Path(args.save_dir)
    save_dir.mkdir(parents=True, exist_ok=True)

    print(f"weights={args.weights}  device={args.device}  classes={classes}")
    print(f"loading model ...")
    model = build_model(args.weights, args.device, classes)

    ground_truth = load_labelme_boxes(args.labels, classes) if args.labels else {}
    stats = {c: {"tp": 0, "fp": 0, "fn": 0, "gt": 0} for c in classes} if ground_truth else None

    times = []
    print(f"running on {len(images)} image(s) ...")
    for img_path in images:
        image = cv2.imread(str(img_path))
        if image is None:
            continue

        t0 = time.time()
        result = model.predict(image, conf=args.conf, device=args.device, verbose=False)[0]
        times.append(time.time() - t0)

        names = result.names
        preds = []  # (class, score, box)
        if result.boxes is not None:
            for box, score, cls_idx in zip(
                result.boxes.xyxy.tolist(),
                result.boxes.conf.tolist(),
                result.boxes.cls.tolist(),
            ):
                preds.append((names[int(cls_idx)], float(score), box))

        # save annotated image
        annotated = image.copy()
        for label, score, (x1, y1, x2, y2) in preds:
            cv2.rectangle(annotated, (int(x1), int(y1)), (int(x2), int(y2)), (0, 200, 0), 3)
            cv2.putText(annotated, f"{label} {score:.2f}", (int(x1), int(y1) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 0), 2)
        cv2.imwrite(str(save_dir / img_path.name), annotated)

        # F1 scoring against ground truth, if provided
        if stats is not None and img_path.stem in ground_truth:
            gt_by_class = collections.defaultdict(list)
            for cls, box in ground_truth[img_path.stem]:
                gt_by_class[cls].append(box)
                stats[cls]["gt"] += 1
            claimed = collections.defaultdict(set)
            for cls, score, box in sorted(preds, key=lambda p: -p[1]):
                if cls not in stats:
                    continue
                best_iou, best_idx = 0.0, -1
                for idx, gt_box in enumerate(gt_by_class.get(cls, [])):
                    if idx in claimed[cls]:
                        continue
                    v = iou(box, gt_box)
                    if v > best_iou:
                        best_iou, best_idx = v, idx
                if best_iou >= args.iou_match:
                    stats[cls]["tp"] += 1
                    claimed[cls].add(best_idx)
                else:
                    stats[cls]["fp"] += 1
            for cls in classes:
                stats[cls]["fn"] += len(gt_by_class.get(cls, [])) - len(claimed[cls])

    # Build the results text once, then both PRINT it and SAVE it to a file,
    # so nothing is lost when the terminal closes.
    lines = []
    mean_ms = 1000 * sum(times) / len(times)
    lines.append(f"weights={args.weights}  device={args.device}  classes={classes}")
    lines.append(f"images={len(times)}  conf={args.conf}")
    lines.append(f"speed: {mean_ms:.1f} ms/image  ({1000 / mean_ms:.1f} FPS)  on {args.device}")

    if stats is not None:
        lines.append("")
        lines.append(f"F1 (IoU>={args.iou_match}):")
        lines.append(f"{'class':16s} {'GT':>4s} {'TP':>4s} {'FP':>4s} {'FN':>4s} {'P':>6s} {'R':>6s} {'F1':>6s}")
        totals = {"tp": 0, "fp": 0, "fn": 0, "gt": 0}
        for cls in classes:
            s = stats[cls]
            p = s["tp"] / (s["tp"] + s["fp"]) if s["tp"] + s["fp"] else 0.0
            r = s["tp"] / (s["tp"] + s["fn"]) if s["tp"] + s["fn"] else 0.0
            f1 = 2 * p * r / (p + r) if p + r else 0.0
            lines.append(f"{cls:16s} {s['gt']:>4d} {s['tp']:>4d} {s['fp']:>4d} {s['fn']:>4d} {p:>6.2f} {r:>6.2f} {f1:>6.2f}")
            for k in totals:
                totals[k] += s[k]
        P = totals["tp"] / (totals["tp"] + totals["fp"]) if totals["tp"] + totals["fp"] else 0.0
        R = totals["tp"] / (totals["tp"] + totals["fn"]) if totals["tp"] + totals["fn"] else 0.0
        F1 = 2 * P * R / (P + R) if P + R else 0.0
        lines.append(f"{'OVERALL':16s} {totals['gt']:>4d} {totals['tp']:>4d} {totals['fp']:>4d} {totals['fn']:>4d} {P:>6.2f} {R:>6.2f} {F1:>6.2f}")

    report = "\n".join(lines)
    print("\n" + report)
    (save_dir / "results.txt").write_text(report + "\n")
    print(f"\nannotated images -> {save_dir}/")
    print(f"results summary   -> {save_dir}/results.txt")


if __name__ == "__main__":
    main()
