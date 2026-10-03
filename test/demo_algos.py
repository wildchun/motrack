"""Run YOLO26 + motrack on the bundled demo clip and render IDs/trails.

Usage:
    python demo_algos.py <Sort|ByteTrack|OCSort> <out.mp4>

Must be run from test/ with the pymotrack .so built under ../build-py.
Use the pytorch28 conda env: PYTHONNOUSERSITE=1 to avoid user-site clashes.
"""
import sys, os, hashlib

sys.path.append(os.path.join(os.getcwd(), "..", "build-py"))
import pymotrack as mt
import cv2
from ultralytics import YOLO

ALGOS = {
    "Sort":      mt.TrackerType.Sort,
    "ByteTrack": mt.TrackerType.ByteTrack,
    "OCSort":    mt.TrackerType.OCSort,
}

def color(tid):
    h = int(hashlib.md5(str(tid).encode()).hexdigest(), 16)
    return (h & 255, (h >> 8) & 255, (h >> 16) & 255)

def main():
    algo_name, out_path = sys.argv[1], sys.argv[2]
    model = YOLO("/home/tangzixing/data/deeplearning/program/object/yolo26/yolo26n.pt")

    cap = cv2.VideoCapture("../assets/demo.mp4")
    fps = cap.get(cv2.CAP_PROP_FPS) or 25
    W, H = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    out = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*"mp4v"), fps, (W, H))

    cfg = mt.TrackerConfig()
    cfg.max_age = 30
    cfg.track_thresh = 0.3
    cfg.high_thresh = 0.6
    cfg.match_thresh = 0.8
    tracker = mt.Tracker(ALGOS[algo_name], cfg)

    trail = {}
    f = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        pred = model(frame, verbose=False)[0]
        objects = []
        if pred.boxes is not None and len(pred.boxes) > 0:
            xyxy = pred.boxes.xyxy.cpu().numpy()
            confs = pred.boxes.conf.cpu().numpy()
            clses = pred.boxes.cls.cpu().numpy().astype(int)
            for (x1, y1, x2, y2), c, k in zip(xyxy, confs, clses):
                if int(k) != 0:  # keep persons only
                    continue
                objects.append(mt.Object(float(c), int(k),
                    mt.Rect(float(x1), float(y1), float(x2 - x1), float(y2 - y1))))

        tracks = tracker.update(objects)
        for t in tracks:
            if not t.b_activated:
                continue
            r = t.object.rect
            col = color(t.track_id)
            cv2.rectangle(frame, (int(r.x), int(r.y)),
                          (int(r.x + r.width), int(r.y + r.height)), col, 2)
            cv2.putText(frame, f"ID {t.track_id}", (int(r.x), max(0, int(r.y) - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)
            cx, cy = int(r.x + r.width / 2), int(r.y + r.height / 2)
            trail.setdefault(t.track_id, []).append((cx, cy))
            if len(trail[t.track_id]) > 40:
                trail[t.track_id].pop(0)
            pts = trail[t.track_id]
            for i in range(1, len(pts)):
                cv2.line(frame, pts[i - 1], pts[i], col, 2)

        cv2.putText(frame, f"motrack {algo_name.lower()}", (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
        out.write(frame)
        f += 1

    cap.release()
    out.release()
    print(f"{algo_name}: {f} frames -> {out_path}")

if __name__ == "__main__":
    main()
