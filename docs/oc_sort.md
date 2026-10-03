# OC-SORT 设计说明（`src/algos/motion/ocsort/`）

> 对应实现：`OCSortTracker`（`OCSortTracker.h/.cpp`），注册名 `TrackerType::OCSort`。
> 原论文：*Observation-Centric SORT* (Cao et al., CVPR 2022)。

## 1. 动机

SORT/ByteTrack 类跟踪器在目标被遮挡时会依赖 Kalman 滤波"盲推"轨迹。误差随时间累积，
重新出现时常关联失败或速度估计已漂移。OC-SORT 的核心主张是：**以观测为中心**——
所有补偿都围绕"最后一次真实观测"展开，而不是信任未观测区间内的虚拟更新。

## 2. 三个核心机制（本实现的对应方式）

### OCM — Observation-Centric Momentum（观测中心动量）

关联代价不只是 IoU 距离，还惩罚"方向突变"：

```
cost(i,j) = iou_cost(i,j) + w_momentum * momentum_cost(i,j)
```

- `momentum_cost`：取轨迹**上一次真实匹配**记录的运动方向
  （`STrack::recordMomentum`，= 新观测中心 − 更新前框中心），
  与候选检测相对预测框的方向做余弦相似度，
  `(1 − cos) / 2` 归一化到 [0,1]，同方向为 0。
- 权重固定 `w_momentum = 0.2`（简化；原论文里该权重参与可调超参）。
- 无历史（新轨迹/首帧）时不惩罚，返回 0。

### ORT — Observation-based Recovery（观测驱动恢复）

丢失轨迹不立即剔除，而是与在跟轨迹**同池关联**（`jointStracks(tracked, lost)`），
直到 `max_age` 帧后才 `markAsRemoved`。复现基于 ByteTrack 式丢失池，
本实现里体现为：丢失轨迹每帧照常 `predict()`，继续参与 IoU+OCM 匹配。

### OCR — Observation-Centric Re-Update（观测中心再更新）

一条**丢失状态**的轨迹被重新匹配到观测时，走 `STrack::reActivate`（而非 `update`）：
Kalman 直接用新观测做一次"新鲜"更新，速度估计从观测重建，
避免长期虚拟外推的漂移被继承下去。原论文还包含对丢失区间轨迹的虚拟回溯再更新
（virtual trajectory re-update），本简化实现以 `reActivate` 语义近似。

## 3. 每帧流程

```
1. 过滤检测: rect 有效 且 prob >= track_thresh → det_stracks
2. 池化: tracked ∪ lost → 逐个 KF predict()
3. 代价矩阵: iou_cost + 0.2 * momentum_cost   (OCM)
4. linearAssignment (lapjv 匈牙利, match_thresh 门限)
5. 匹配对:
     - Tracked 状态 → update() + recordMomentum()
     - Lost 状态   → reActivate() + recordMomentum()   (OCR)
6. 未匹配 Tracked → markAsLost() 进丢失池          (ORT)
7. 未匹配检测  → activate() 直接建新轨迹 (无 unconfirmed 阶段)
8. 丢失池中 frame_id 差 > max_age → markAsRemoved
9. 返回 tracked 集合
```

## 4. 与 ByteTrack 的取舍

| 维度 | ByteTrack | OC-Sort |
|---|---|---|
| 低分检测 | 第二阶段专门回收 | 不使用（只用高置信度检测） |
| 关联阶段 | 3 次（高/低分 + unconfirmed） | 1 次 |
| 新轨迹 | 需 `high_thresh`，有 unconfirmed 缓冲 | 匹配不上即建（更快但更易碎） |
| 遮挡恢复 | 依赖丢失池 IoU | 丢失池 + 方向一致性 + 重激活 |

## 5. 配置项（`TrackerConfig` 中实际读取）

| 字段 | 用途 |
|---|---|
| `track_thresh` | 检测置信度过滤 |
| `match_thresh` | 匈牙利分配的 IoU 代价门限 |
| `max_age` | 丢失轨迹存活帧数 |

外观相关字段（`appearance_thresh` 等）在本算法中忽略。
