"""YOLO + ByteTrackLib demo — run on any video (or image folder) end-to-end.

Usage (run from the `test/` directory after a `-DWITH_PYTHON=true` build):

    # against the bundled sample clip:
    python demo_yolo.py --source ../assets/demo.mp4 \
                        --weights yolo11s.pt \
                        --out ./output/demo.mp4

    # against your own video / image directory (e.g. MOT16-04's frames):
    python demo_yolo.py --source MOT16-04/img1 \
                        --out ./output/mot16-04_yolo.mp4

YOLO weights are NOT bundled — ultralytics will auto-download ``yolo11s.pt``
on first use, or point ``--weights`` at any local ``.pt`` you already have.

By default only the COCO ``person`` class (id 0) is fed to the tracker; pass
``--classes 0 2 5`` to include e.g. cars/buses. YOLO already outputs a
probability in [0, 1], so no score normalisation is needed.
"""

import argparse
import glob
import os
import sys
from collections import defaultdict, deque

import cv2

# Make the built pymotrack module importable (mirrors test_motrack.py).
sys.path.append(os.path.join(os.path.dirname(__file__), "..", "build"))
import pymotrack as mt  # noqa: E402

from ultralytics import YOLO  # noqa: E402


# ---------------------------------------------------------------------------
# Source iterators
# ---------------------------------------------------------------------------
_IMAGE_EXTS = (".jpg", ".jpeg", ".png", ".bmp")


def open_source(source):
    """Return (frames_iter, fps, (w, h), total_or_None).

    Accepts:
      - a video file path
      - a directory of images (natural sort by filename)
      - an integer string for a webcam device id ("0", "1", ...)
    """
    if os.path.isdir(source):
        files = sorted(
            f for ext in _IMAGE_EXTS
            for f in glob.glob(os.path.join(source, f"*{ext}"))
        )
        if not files:
            raise FileNotFoundError(f"no images in {source}")
        first = cv2.imread(files[0])
        if first is None:
            raise RuntimeError(f"cannot read {files[0]}")
        h, w = first.shape[:2]

        def gen():
            for p in files:
                img = cv2.imread(p)
                if img is not None:
                    yield img

        return gen(), 30.0, (w, h), len(files)

    # File or camera.
    cap_arg = int(source) if source.isdigit() else source
    cap = cv2.VideoCapture(cap_arg)
    if not cap.isOpened():
        raise FileNotFoundError(f"cannot open source: {source}")
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    w   = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h   = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    n   = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) or None

    def gen():
        try:
            while True:
                ok, img = cap.read()
                if not ok:
                    break
                yield img
        finally:
            cap.release()

    return gen(), fps, (w, h), n


# ---------------------------------------------------------------------------
# Drawing (same style as test/demo.py)
# ---------------------------------------------------------------------------
_COLOR_TABLE = None

def color_for_id(tid):
    global _COLOR_TABLE
    if _COLOR_TABLE is None:
        import numpy as np
        hsv = np.zeros((1, 256, 3), dtype=np.uint8)
        hsv[0, :, 0] = np.arange(256) * 179 // 256
        hsv[0, :, 1] = 220
        hsv[0, :, 2] = 255
        bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0]
        _COLOR_TABLE = [tuple(int(v) for v in bgr[i]) for i in range(256)]
    return _COLOR_TABLE[int(tid) % 256]


def draw_frame(img, tracks, trails, class_names):
    for tr in tracks:
        if not tr.b_activated:
            continue
        r = tr.object.rect
        x1, y1 = int(r.x), int(r.y)
        x2, y2 = int(r.x + r.width), int(r.y + r.height)
        col = color_for_id(tr.track_id)

        cv2.rectangle(img, (x1, y1), (x2, y2), col, 2)
        cls_name = class_names.get(int(tr.object.label), str(int(tr.object.label)))
        label = f"{cls_name} #{tr.track_id} {tr.object.prob:.2f}"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(img, (x1, y1 - th - 6), (x1 + tw + 4, y1), col, -1)
        cv2.putText(img, label, (x1 + 2, y1 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)

        pts = list(trails.get(tr.track_id, ()))
        for i in range(1, len(pts)):
            cv2.line(img, pts[i - 1], pts[i], col, 2)
    return img


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def run(args):
    frames, src_fps, (w, h), total = open_source(args.source)
    fps = args.fps or src_fps

    yolo = YOLO(args.weights)
    class_names = yolo.model.names if hasattr(yolo.model, "names") else {}
    keep = set(args.classes)

    config = mt.TrackerConfig(
        max_age=args.max_age,
        track_thresh=args.track_thresh,
        high_thresh=args.high_thresh,
        match_thresh=args.match_thresh,
    )
    tracker = mt.Tracker(mt.TrackerType.ByteTrack, config)

    writer = None
    if args.out:
        os.makedirs(os.path.dirname(os.path.abspath(args.out)) or ".", exist_ok=True)
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(args.out, fourcc, fps, (w, h))
        if not writer.isOpened():
            raise RuntimeError(f"cannot open video writer: {args.out}")

    results_fp = open(args.results, "w") if args.results else None
    trails = defaultdict(lambda: deque(maxlen=args.trail_len))

    print(f"[demo_yolo] source={args.source}  {w}x{h}@{fps:.1f}  "
          f"frames={total}  weights={os.path.basename(args.weights)}  "
          f"classes={sorted(keep)}  device={args.device}")

    frame_id = 0
    for img in frames:
        frame_id += 1

        # YOLO inference. verbose=False keeps stdout clean.
        pred = yolo.predict(
            img, conf=args.conf, iou=args.nms_iou,
            imgsz=args.imgsz, device=args.device,
            classes=list(keep) if keep else None,
            verbose=False,
        )[0]

        objects = []
        if pred.boxes is not None and len(pred.boxes) > 0:
            xyxy   = pred.boxes.xyxy.cpu().numpy()
            confs  = pred.boxes.conf.cpu().numpy()
            clses  = pred.boxes.cls.cpu().numpy().astype(int)
            for (x1, y1, x2, y2), c, k in zip(xyxy, confs, clses):
                objects.append(mt.Object(
                    prob=float(c),
                    label=int(k),
                    rect=mt.Rect(float(x1), float(y1),
                                   float(x2 - x1), float(y2 - y1)),
                ))

        tracks = tracker.update(objects)

        for tr in tracks:
            if not tr.b_activated:
                continue
            r = tr.object.rect
            cx = int(r.x + r.width  * 0.5)
            cy = int(r.y + r.height * 0.5)
            trails[tr.track_id].append((cx, cy))

            if results_fp is not None:
                results_fp.write(
                    f"{frame_id},{tr.track_id},"
                    f"{r.x:.2f},{r.y:.2f},{r.width:.2f},{r.height:.2f},"
                    f"{tr.object.prob:.4f},{tr.object.label},-1,-1\n"
                )

        if writer is not None or args.show:
            draw_frame(img, tracks, trails, class_names)
            hud = f"frame {frame_id}"
            if total:
                hud += f"/{total}"
            hud += f"  dets={len(objects)}  tracks={sum(1 for t in tracks if t.b_activated)}"
            cv2.putText(img, hud, (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
            if writer is not None:
                writer.write(img)
            if args.show:
                cv2.imshow("demo_yolo", img)
                if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    break

        if frame_id % 50 == 0:
            print(f"[demo_yolo] processed {frame_id}"
                  + (f"/{total}" if total else ""))

    if writer is not None:
        writer.release()
    if results_fp is not None:
        results_fp.close()
    if args.show:
        cv2.destroyAllWindows()

    print(f"[demo_yolo] done ({frame_id} frames)")
    if args.out:
        print(f"[demo_yolo]   video   → {args.out}")
    if args.results:
        print(f"[demo_yolo]   results → {args.results}")


def parse_args():
    default_weights = "yolo11s.pt"

    p = argparse.ArgumentParser(description="YOLO + ByteTrackLib demo")
    p.add_argument("--source",  required=True,
                   help="video file, image directory, or webcam id (e.g. 0)")
    p.add_argument("--weights", default=default_weights,
                   help=f"YOLO weights (.pt). Default: {default_weights}")
    p.add_argument("--out",     default=None, help="annotated output MP4 (optional)")
    p.add_argument("--results", default=None, help="MOT-format results txt (optional)")
    p.add_argument("--show",    action="store_true", help="also show live cv2 window")

    # YOLO knobs.
    p.add_argument("--conf",    type=float, default=0.25,
                   help="YOLO confidence threshold (below this is discarded before tracking)")
    p.add_argument("--nms_iou", type=float, default=0.7, help="YOLO NMS IoU")
    p.add_argument("--imgsz",   type=int,   default=640)
    p.add_argument("--device",  default=None, help="e.g. 'cuda:0' / 'cpu'; ultralytics auto by default")
    p.add_argument("--classes", type=int, nargs="*", default=[0],
                   help="COCO class ids to keep (default: person=0). Empty = all classes.")

    # Tracker knobs (defaults match TrackerConfig).
    p.add_argument("--max_age",      type=int,   default=30)
    p.add_argument("--track_thresh", type=float, default=0.3)
    p.add_argument("--high_thresh",  type=float, default=0.6)
    p.add_argument("--match_thresh", type=float, default=0.8)

    # Render.
    p.add_argument("--fps",       type=float, default=None,
                   help="override output video fps (default: source fps, or 30 for image dirs)")
    p.add_argument("--trail_len", type=int,   default=30)
    return p.parse_args()


if __name__ == "__main__":
    run(parse_args())
