<div align="center">
<h2>motrack</h2>

<p><b>Multi-object tracking in pure C++11 — ByteTrack / Sort / OC-Sort / DeepSort / JDE in one framework. Zero external fetch, embeddable, cross-compilable.</b></p>

<p>
  <a href="https://github.com/ifzhang/ByteTrack">ByteTrack</a> ·
  <a href="https://arxiv.org/abs/2203.14366">OC-SORT</a> ·
  DeepSORT · JDE ·
  C++11 · CMake · pybind11 bindings · rv1106-ready
</p>

<p align="center">
  <img src="assets/demo.gif" alt="motrack demo: YOLO detections tracked across frames" width="560"/>
</p>

<p>
  <i>YOLO26 + motrack on the bundled sample clip — stable IDs with motion trails.</i>
</p>

<p>
  <a href="#compilation"><img alt="build" src="https://img.shields.io/badge/build-cmake-blue"></a>
  <a href="#cross-compile"><img alt="platform" src="https://img.shields.io/badge/target-rv1106-orange"></a>
  <a href="README_cn.md"><img alt="docs" src="https://img.shields.io/badge/docs-中文-success"></a>
</p>
</div>

This repository provides a C++ multi-object tracking library (motrack) with pluggable association algorithms — ByteTrack, Sort, OC-Sort, DeepSort and JDE-style joint embedding tracking — plus Python bindings and easy cross-compilation.

**Highlights**

- 🧩 **Single-header public API** (`Motrack.h`) — one unified `Tracker` facade, pimpl-hidden internals, no Eigen leak into your build.
- 🔀 **Pluggable algorithms** — `TrackerType::ByteTrack / Sort / OCSort / DeepSort / JDE`; motion-only and appearance-based families live in separate `src/algos/` folders (design docs under `docs/`).
- 🪶 **C++11 only** — vendored Eigen 3.3.9, nothing fetched at configure time; hermetic cross-compilation.
- 🐍 **Optional Python bindings** — a pip-installable wheel (`pymotrack`) falls out of a normal CMake build.
- 🎯 **YOLO demo included** — `test/demo_yolo.py` goes from video/frames/webcam to annotated MP4 + MOT results.

### Compilation
To compile the project, follow these steps:
```bash
mkdir build && cd build && cmake ..
make -j4 install
```
Use `-DWITH_PYTHON=true` if you need to build the Python library.

### Cross Compile
For rv1106 compilation, use:
```bash
cmake -DCMAKE_TOOLCHAIN_FILE=../toolchain/rv1106.toolchain.cmake ..
```

### Dependencies
No need to download separately; dependencies are embedded within the project:
- Eigen 3.3.9
- pybind11-2.10.4 (optional)

### Example Usage
```cpp
#include "Motrack.h"
#include <iostream>
int main(int argc, char* argv[])
{
    motrack::Tracker tracker(motrack::TrackerType::ByteTrack);
    for (int i = 0; i < 20; i++)
    {
        std::vector<motrack::Object> objects;
        objects.push_back({0.9, 0, {100+i*10, 100, 50, 50}});
        std::vector<motrack::Track> tracks = tracker.update(objects);
    }
    return 0;
}
```

Switch algorithms with one argument:

```cpp
motrack::TrackerConfig config;          // shared by all algorithms
config.max_age = 30;
motrack::Tracker deep(motrack::TrackerType::DeepSort, config);  // Re-ID based
```

### Python wheel

When configured with `-DWITH_PYTHON=true`, the build also produces an
installable wheel under `build/dist/`:

```bash
mkdir build && cd build
cmake -DWITH_PYTHON=true ..
make -j4                                              # builds .so + wheel
pip install dist/pymotrack-*.whl                      # installs `pymotrack`
```

The wheel is platform-tagged (e.g. `pymotrack-1.0.0-cp310-cp310-linux_x86_64.whl`)
and re-exports the same API as the in-tree `.so`:

```python
import pymotrack as mt

tracker = mt.Tracker(mt.TrackerType.ByteTrack, mt.TrackerConfig(max_age=30))
tracks = tracker.update([
    mt.Object(prob=0.9, label=0, rect=mt.Rect(100, 100, 50, 50)),
])
```

Override the version with `-DMOTRACK_WHEEL_VERSION=x.y.z` at configure time.

### YOLO + tracking demo (`test/demo_yolo.py`)

`test/demo_yolo.py` runs an off-the-shelf YOLO detector, feeds its boxes into
`pymotrack`, and renders IDs / trails to an MP4. It accepts a
video file, an image directory (one frame per file), or a webcam device id.

`assets/demo.mp4` is a short sample clip bundled for a self-contained run.
YOLO weights are **not** shipped — grab any `ultralytics`-compatible checkpoint
yourself, e.g.:

```bash
# ultralytics auto-downloads on first use:
python -c "from ultralytics import YOLO; YOLO('yolo11s.pt')"
# or fetch manually:
wget https://github.com/ultralytics/assets/releases/download/v8.2.0/yolo11s.pt
```

Requirements: build the Python module first, then
`pip install ultralytics opencv-python` (or install the wheel from the previous
section). Run from the `test/` directory so the `../build` shim in
`demo_yolo.py` can find the freshly-built `.so`:

```bash
cd test
python demo_yolo.py \
    --source  ../assets/demo.mp4 \
    --weights yolo11s.pt \
    --out     ./output/demo.mp4
```

Common knobs (see `demo_yolo.py --help` for the full list):

- `--classes 0 2 5` — COCO ids to keep (default `0` = person, empty = all).
- `--conf 0.25` / `--nms_iou 0.7` / `--imgsz 640` — YOLO thresholds.
- `--track_thresh` / `--high_thresh` / `--match_thresh` / `--max_age` —
  forwarded verbatim to the tracker (defaults match the C++ side).
- `--results out.txt` — dump MOT-format results alongside the video.
- `--show` — also open a live cv2 window (press `q` / `Esc` to stop).

### References:
- [ByteTrack](https://github.com/ifzhang/ByteTrack)
- [ByteTrack-cpp](https://github.com/Vertical-Beach/ByteTrack-cpp)

### Citation
```bibtex
@article{zhang2022bytetrack,
  title={ByteTrack: Multi-Object Tracking by Associating Every Detection Box},
  author={Zhang, Yifu and Sun, Peize and Jiang, Yi and Yu, Dongdong and Weng, Fucheng and Yuan, Zehuan and Luo, Ping and Liu, Wenyu and Wang, Xinggang},
  booktitle={Proceedings of the European Conference on Computer Vision (ECCV)},
  year={2022}
}
```
