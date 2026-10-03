# DeepSORT 设计说明（`src/algos/appearance/deepsort/`）

## 1. 算法定位

DeepSORT = SORT + 外观关联。检测来自外部分割/检测网络，每个框附一个
Re-ID 嵌入（`Object::feature`）；跟踪层用"运动 + 外观"双代价做级联匹配，
在长遮挡、密集人群下保持身份。

本实现为 **余弦度量简化版**：不做外观网络（特征由调用方提供），
省略原论文的深度外观模型训练部分，保留级联匹配与融合代价两个核心。

## 2. 核心设计

### 级联匹配（Cascade Matching）

原论文按 `time_since_update` 从小到大分轮匹配：越"新鲜"的轨迹优先认领
歧义检测，避免长时间丢失的轨迹抢走本属于在跟轨迹的框。

本实现按缺失帧数（`frame_id - track->getFrameId()`）分轮：
- 第 `age` 轮只允许缺失恰好 `age` 帧的轨迹参与；
- 检测池逐轮收缩（`det_taken` 标记）；
- 最多级联 `min(max_age, 30)` 轮。

### 融合代价

```
cost = lambda * iou_cost + (1 - lambda) * cosine_cost
```

- `cosine_cost`：`FeatureMetric::cosineDistance` —— 轨迹特征库（gallery）
  中与检测嵌入的**最小**余弦距离（取最优代表，抗单帧噪声）。
- `lambda` = `TrackerConfig::lambda_weight`（默认 0.98，运动主导）。
- 门控：`cosine_cost > appearance_thresh` 的配对直接置 1（禁止匹配）。

### 特征库（Gallery）

`STrack::addFeature(feature, feature_budget)`：
- 首个特征直接入库；
- 同维度后续特征以 EMA 融合（`0.9*old + 0.1*new`），库大小恒为 1 的滑动代表；
- `feature_budget` 为上限（本 EMA 实现下实际只保留 1 条，预算参数保留以兼容
  未来切换为多向量库的实现）。

### IoU 兜底轮

级联轮结束后仍未匹配的轨迹与检测，再做一轮**纯 IoU** 匹配：
- 兜住"没带特征"或"特征被门控"的检测；
- 保证特征缺失时退化行为接近 SORT 而不是失效。

## 3. 每帧流程

```
1. 过滤检测 (rect 有效, prob >= track_thresh)
2. 池化 tracked ∪ lost → KF predict()
3. 预计算 iou_cost 与 appearance_cost (全池)
4. 级联轮 age = 0..min(max_age,30):
     参与者 = 缺失恰好 age 帧的轨迹 × 未认领检测
     cost = fuseCost(iou, cosine, lambda, appearance_thresh)
     linearAssignment → 匹配对 update/reActivate + addFeature
5. IoU 兜底轮（剩余轨迹 × 剩余检测, 纯 IoU）
6. 未匹配 Tracked → markAsLost()
7. 未匹配检测 → activate() 建新轨迹 (gallery 种子)
8. 丢失 > max_age → markAsRemoved
```

## 4. 配置项

| 字段 | 用途 |
|---|---|
| `track_thresh` | 检测过滤 |
| `match_thresh` | 分配门限（对融合代价生效） |
| `max_age` | 丢失存活帧数 & 级联上限 |
| `lambda_weight` | 运动外观混合权重 |
| `appearance_thresh` | 余弦距离门控 |
| `feature_budget` | gallery 容量上限 |

## 5. 调用方要求

- 每帧检测的 `Object::feature` **必须同维度**（如 512）；
- 完全不带特征的调用可用，但此时等同 SORT + 丢失池。
