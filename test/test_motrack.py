import sys
sys.path.append("../build")
import pymotrack as mt
tracker = mt.Tracker(mt.TrackerType.ByteTrack)

for i in range(20):
    objects = [mt.Object(prob=0.9, label=0, rect=mt.Rect(100+i*10, 100, 50, 50)),
        mt.Object(prob=0.85, label=1, rect=mt.Rect(150, 150+i*10, 40, 40))]
    tracks = tracker.update(objects)
    print(tracks)
