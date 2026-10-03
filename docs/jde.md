# JDE 框架设计说明（`src/algos/appearance/jde/`）

> 对应实现：`JDETracker`（`JDETracker.h/.cpp`），注册名 `TrackerType::JDE`。
> 原论文：*Towards Real-Time Multi-Object Tracking*（Wang et al., ECCV 2020）——JDE，
> Joint Detection and Embedding。

## 1. JDE 是什么

JDE 不是一种新的关联算法，而是一种**网络架构约定**：检测头与 Re-ID 嵌入头共享同一
backbone，一次前向同时输出每框的 `(bbox, score, embedding)`。跟踪阶段因此可以
**一次性融合运动与外观**完成关联，不需要像 DeepSORT 那样分轮级联。

本框架只实现 JDE 的**跟踪侧**：约定调用方（JDE 风格检测网络或"检测器+轻量 ReID 头"）
在 `Object::feature` 里填好每框嵌入，本类负责关联与生命周期。

## 2. 设计要点（与 DeepSort 的差异）

| 维度 | DeepSort | JDE（本实现） |
|---|---|---|
| 关联轮数 | 级联多轮 + IoU 兜底 | **单轮融合** |
| 代价 | λ·运动 + (1−λ)·外观，按新鲜度分轮 | 同公式，一次全池求解 |
| 新轨迹初始化 | 无置信度门控 | **high_thresh 门控**（JDE 无低分回收，弱框直接丢弃） |
| 嵌入来源 | 独立 ReID 网络（后处理） | 检测头自带（同网联合） |
| 延迟 | 较高（多轮匈牙利） | 低（单轮） |
| 对嵌入质量的要求 | 可容忍较弱特征（有兜底） | 依赖较强判别性特征 |

## 3. 每帧流程

```
1. 过滤检测: rect 有效 且 prob >= track_thresh
2. 池化 tracked ∪ lost → KF predict()
3. 全池预计算 iou_cost 与 cosine_cost, 融合:
     cost = λ·iou + (1−λ)·cos, cos > appearance_thresh 的配对置 1 门控
4. linearAssignment 单轮匈牙利 (match_thresh)
5. 匹配对: Tracked→update / Lost→reActivate, 并 addFeature 入库
6. 未匹配 Tracked → markAsLost()
7. 未匹配检测: prob >= high_thresh 才 activate() 建新轨迹
8. 丢失 > max_age → markAsRemoved
9. 返回 tracked 集合
```

## 4. 融合代价与特征库

复用与 DeepSORT 完全相同的基础设施（`src/core/`）：

- `FeatureMetric::cosineDistance`：gallery 最小余弦距离；
- `FeatureMetric::fuseCost`：λ 混合 + 外观门控；
- `STrack::addFeature`：EMA 滑动代表，容量 `feature_budget`。

差异只在**策略编排**（`JDETracker.cpp`）：单轮、无兜底、置信度门控初始化。

## 5. 配置项

| 字段 | 用途 |
|---|---|
| `track_thresh` | 检测过滤 |
| `high_thresh` | 新轨迹初始化门控（比 DeepSORT 严格） |
| `match_thresh` | 分配门限 |
| `max_age` | 丢失存活帧数 |
| `lambda_weight` | 运动外观混合权重 |
| `appearance_thresh` | 余弦距离门控 |
| `feature_budget` | gallery 容量上限 |

## 6. 调用方约定

- 每个检测必须携带同维度 `Object::feature`（JDE 网络输出）；
- 嵌入应为 L2 归一化后的向量（余弦距离假设单位球）；
- 若特征质量不稳定（如小目标），建议改用 `TrackerType::DeepSort`（有 IoU 兜底）。

## 7. 扩展方向

- **FairMOT / CSTrack**：同为 JDE 家族，嵌入维度与平衡损失不同，跟踪侧逻辑一致，
  直接复用本类即可，只需调用方替换网络；
- **低分检测回收**：如需 ByteTrack 式低分二次关联，可在步骤 7 前插入第二阶段；
- **相机运动补偿（CMC）**：在步骤 2 前对预测框做仿射修正，接口点已预留
  （可在 `predict()` 后统一处理池内轨迹）。
