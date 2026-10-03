#!/usr/bin/env python3
"""
ByteTrackLib 架构流程图生成脚本

使用 matplotlib 绘制项目架构图、算法流程图和状态机图。
输出 SVG 格式，可直接嵌入 Markdown 文档。

依赖: pip install matplotlib numpy

使用方法:
    python generate_diagrams.py
    # 所有 SVG 图片将输出到 figures/ 目录下
"""

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch
import numpy as np
import os

# ================================================================
# 全局配置
# ================================================================
BASE = os.path.join(os.path.dirname(__file__), "figures")
os.makedirs(BASE, exist_ok=True)

# 配色方案
COLORS = {
    "public_bg": "#E8F0FE",
    "public_border": "#4472C4",
    "public_box": "#D6E4F0",
    "public_facade": "#BDD7EE",
    "internal_bg": "#FFF2E6",
    "internal_border": "#ED7D31",
    "internal_box": "#FCE4D6",
    "python_box": "#E2EFDA",
    "state_new": "#DAEEF3",
    "state_tracked": "#E2EFDA",
    "state_lost": "#FFF2CC",
    "state_removed": "#F4CCCC",
    "note_box": "#F2F2F2",
    "arrow_gray": "#666666",
    "arrow_green": "#2E7D32",
    "arrow_red": "#C62828",
    "arrow_orange": "#E65100",
    "action_highlight": "#FFF9C4",
}


# ================================================================
# 绘图辅助函数
# ================================================================

def draw_rounded_box(ax, x, y, w, h, text, color, fontsize=9, bold=False, align='center'):
    """绘制圆角矩形框并居中写文本。"""
    box = FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.1",
                         facecolor=color, edgecolor='#333333', linewidth=1.2)
    ax.add_patch(box)
    ha = {'left': 'left', 'center': 'center', 'right': 'right'}[align]
    ax.text(x + w / 2, y + h / 2, text, ha=ha, va='center', fontsize=fontsize,
            fontweight='bold' if bold else 'normal', family='sans-serif')


def draw_arrow(ax, x1, y1, x2, y2, label='', color='#666666'):
    """绘制带标签的曲线箭头。"""
    ax.annotate('', xy=(x2, y2), xytext=(x1, y1),
                arrowprops=dict(arrowstyle='->', color=color, lw=1.2,
                                connectionstyle='arc3,rad=0.2'))
    if label:
        mx, my = (x1 + x2) / 2, (y1 + y2) / 2
        ax.text(mx + 0.15, my + 0.1, label, fontsize=7, color=color,
                ha='center', va='bottom', family='sans-serif')


def draw_straight_arrow(ax, x1, y1, x2, y2, label='', color='#666666'):
    """绘制带标签的直线箭头。"""
    ax.annotate('', xy=(x2, y2), xytext=(x1, y1),
                arrowprops=dict(arrowstyle='->', color=color, lw=1.2))
    if label:
        mx, my = (x1 + x2) / 2, (y1 + y2) / 2
        ax.text(mx + 0.1, my + 0.1, label, fontsize=7, color=color,
                ha='center', va='bottom', family='sans-serif')


def save_fig(name, fig, tight=True):
    """保存当前图形为 SVG。"""
    path = os.path.join(BASE, name)
    fig.savefig(path, bbox_inches='tight' if tight else None, dpi=150)
    plt.close(fig)
    print(f"✓ {name}")


# ================================================================
# 1. 整体架构 / 模块依赖图
# ================================================================

def draw_architecture():
    fig, ax = plt.subplots(1, 1, figsize=(10, 8))
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 8)
    ax.axis('off')
    ax.set_title('ByteTrackLib Architecture Overview', fontsize=14, fontweight='bold', pad=10)

    C = COLORS

    # 公有 API 区域（虚线框）
    ax.add_patch(FancyBboxPatch((0.5, 5.5), 9, 2.2, boxstyle="round,pad=0.2",
                                facecolor=C["public_bg"], edgecolor=C["public_border"],
                                linewidth=1.5, linestyle='--'))
    ax.text(0.5, 7.5, 'Public API  —  include/Motrack.h', fontsize=10,
            fontweight='bold', color=C["public_border"])

    draw_rounded_box(ax, 0.8, 5.8, 2.0, 1.5, "Rect\n{x, y, width, height}", C["public_box"], 8)
    draw_rounded_box(ax, 3.4, 5.8, 2.2, 1.5, "Object\n{prob, label, rect}", C["public_box"], 8)
    draw_rounded_box(ax, 6.2, 5.8, 2.2, 1.5,
                     "Track\n{b_activated, track_id,\n frame_id, object}", C["public_box"], 8)
    draw_rounded_box(ax, 3.8, 3.0, 2.4, 1.2, "ByteTracker\n{pimpl → Impl}",
                     C["public_facade"], 9, True)

    # 内部实现区域（虚线框）
    ax.add_patch(FancyBboxPatch((0.5, 0.5), 9, 2.2, boxstyle="round,pad=0.2",
                                facecolor=C["internal_bg"], edgecolor=C["internal_border"],
                                linewidth=1.5, linestyle='--'))
    ax.text(0.5, 2.5, 'Internal Implementation', fontsize=10,
            fontweight='bold', color=C["internal_border"])

    draw_rounded_box(ax, 0.8, 0.8, 2.4, 1.2, "ByteTrackerImpl\n{5-stage update()}",
                     C["internal_box"], 8, True)
    draw_rounded_box(ax, 3.8, 0.8, 2.0, 1.2, "STrack\n{state machine,\nKF, track_id}",
                     C["internal_box"], 8)
    draw_rounded_box(ax, 6.2, 0.8, 1.8, 1.2, "KalmanFilter\n{8-dim KF}",
                     C["internal_box"], 8)
    draw_rounded_box(ax, 8.3, 0.8, 1.4, 1.2, "lapjv\n{JV alg}",
                     C["internal_box"], 8)

    # Python 绑定
    draw_rounded_box(ax, 7.0, 3.6, 2.5, 1.0, "pymotrack (Python)\n{pybind11 module}",
                     C["python_box"], 8, True)

    # 连接线
    draw_straight_arrow(ax, 5.0, 4.2, 5.0, 3.0, 'pimpl')
    draw_straight_arrow(ax, 5.0, 2.0, 5.0, 1.2, 'owns', '#4472C4')
    draw_straight_arrow(ax, 5.8, 1.2, 6.2, 1.2, 'has KF', '#4472C4')
    draw_arrow(ax, 5.0, 1.2, 8.3, 1.2, 'linearAssignment →', '#4472C4')
    draw_arrow(ax, 8.3, 2.0, 8.3, 3.6, 'wraps', '#4472C4')
    draw_arrow(ax, 7.0, 4.0, 5.0, 4.0, '', '#4472C4')

    save_fig("architecture.svg", fig)


# ================================================================
# 2. ByteTrack 5 阶段算法流程图
# ================================================================

def draw_bytetrack_flow():
    fig, ax = plt.subplots(1, 1, figsize=(10, 12))
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 12)
    ax.axis('off')
    ax.set_title('ByteTrackerImpl::update() — Per-Frame 5-Stage Algorithm',
                 fontsize=14, fontweight='bold', pad=10)

    C = COLORS
    bx = 2.5
    bw = 5.0
    bh = 0.8
    gap = 0.7

    # 准备阶段
    y = 11.0
    draw_rounded_box(ax, bx, y, bw, bh, 'Frame Start: frame_id++', C["python_box"], 9, True)
    y -= bh + gap

    draw_rounded_box(ax, bx, y, bw, bh + 0.2,
                     'Split detections by confidence:\n'
                     'high = prob ≥ track_thresh  |  low = prob < track_thresh',
                     C["state_new"], 8)
    y -= bh + 0.2 + gap

    draw_rounded_box(ax, bx, y, bw, bh + 0.2,
                     'Split tracked_stracks:\n'
                     '→ active_stracks (activated)  |  → non_active (unconfirmed)',
                     C["state_new"], 8)
    y -= bh + 0.2 + gap

    draw_rounded_box(ax, bx, y, bw, bh + 0.2,
                     'Build strack_pool = active ∪ lost\n'
                     'KF-predict all tracks',
                     C["state_new"], 8)
    y -= bh + 0.2 + gap

    # 阶段 1
    draw_rounded_box(ax, bx, y, bw, bh + 0.2,
                     'Stage 1: High-Score Association\n'
                     'strack_pool × det_high  |  IoU threshold = match_thresh',
                     C["internal_box"], 8, True)
    y -= bh + 0.2 + gap

    # 阶段 2
    draw_rounded_box(ax, bx, y, bw, bh + 0.2,
                     'Stage 2: Low-Score Association\n'
                     'remain_tracked × det_low  |  IoU threshold = 0.5',
                     C["internal_box"], 8, True)
    y -= bh + 0.2 + gap

    # 阶段 3
    draw_rounded_box(ax, bx, y, bw, bh + 0.2,
                     'Stage 3: Unconfirmed Association\n'
                     'non_active × remain_det_high  |  IoU threshold = 0.7',
                     C["internal_box"], 8, True)
    y -= bh + 0.2 + gap

    # 阶段 4
    draw_rounded_box(ax, bx, y, bw, bh + 0.2,
                     'Stage 4: New Track Init\n'
                     'unmatched det with prob ≥ high_thresh → activate new track',
                     C["python_box"], 8)
    y -= bh + 0.2 + gap

    # 阶段 5
    draw_rounded_box(ax, bx, y, bw, bh + 0.3,
                     "Stage 5: State Update\n"
                     "• Remove lost tracks older than max_age\n"
                     "• Merge tracked / refound\n"
                     "• removeDuplicateStracks (IoU > 0.85)",
                     C["public_box"], 8)
    y -= bh + 0.3 + gap

    # 输出
    draw_rounded_box(ax, bx, y, bw, bh,
                     'Output: tracked_stracks (activated tracks only)',
                     C["python_box"], 9, True)

    # 竖直连接箭头
    ys_top = 11.0
    for i in range(8):
        y_curr = ys_top - i * (bh + gap if i == 0 else (bh + 0.2 + gap if i < 4 else bh + 0.2 + gap))
        # Simplified: just draw arrows between consecutive boxes
    # Redo arrows more carefully:
    y_positions = []
    yp = 11.0
    y_positions.append(yp)  # start
    yp -= bh + gap
    y_positions.append(yp)  # split dets
    yp -= bh + 0.2 + gap
    y_positions.append(yp)  # split tracks
    yp -= bh + 0.2 + gap
    y_positions.append(yp)  # build pool
    yp -= bh + 0.2 + gap
    y_positions.append(yp)  # stage 1
    yp -= bh + 0.2 + gap
    y_positions.append(yp)  # stage 2
    yp -= bh + 0.2 + gap
    y_positions.append(yp)  # stage 3
    yp -= bh + 0.2 + gap
    y_positions.append(yp)  # stage 4
    yp -= bh + 0.3 + gap
    y_positions.append(yp)  # stage 5
    yp -= bh + gap
    y_positions.append(yp)  # output

    for i in range(len(y_positions) - 1):
        draw_straight_arrow(ax, 5.0, y_positions[i], 5.0, y_positions[i] - (y_positions[i] - y_positions[i+1]))

    save_fig("bytetrack_flow.svg", fig)


# ================================================================
# 3. STrack 状态机
# ================================================================

def draw_strack_sm():
    fig, ax = plt.subplots(1, 1, figsize=(10, 6))
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 6)
    ax.axis('off')
    ax.set_title('STrack State Machine', fontsize=14, fontweight='bold', pad=10)

    C = COLORS

    draw_rounded_box(ax, 0.5, 3.5, 2.5, 1.5,
                     "New\n(is_activated = false)\nframe_id_, track_id_ = 0",
                     C["state_new"], 8)
    draw_rounded_box(ax, 3.8, 3.5, 3.0, 1.5,
                     "Tracked\n(is_activated = true)\n"
                     "• KF predict each frame\n"
                     "• KF update on match\n"
                     "• tracklet_len++",
                     C["state_tracked"], 8)
    draw_rounded_box(ax, 7.2, 3.5, 2.5, 1.5,
                     "Lost\n• KF predict only\n"
                     "• can reActivate()\n"
                     "• max_time_lost_ TTL",
                     C["state_lost"], 8)
    draw_rounded_box(ax, 4.0, 0.5, 2.5, 1.5,
                     "Removed\n(dropped from\ntracked_ / lost_ pools)",
                     C["state_removed"], 8)

    # New -> Tracked
    draw_straight_arrow(ax, 3.0, 4.2, 3.8, 4.2,
                        'activate()\nprob ≥ high_thresh', C["arrow_green"])
    # New -> Removed
    draw_arrow(ax, 3.0, 3.5, 4.0, 2.0,
               'markAsRemoved()\nunmatched in stage 3', C["arrow_red"])
    # Tracked -> Lost
    draw_straight_arrow(ax, 6.8, 3.5, 7.2, 3.5,
                        'markAsLost()\nunmatched in stage 2', C["arrow_orange"])
    # Lost -> Tracked
    draw_arrow(ax, 7.2, 4.5, 6.8, 5.0,
               'reActivate()\nmatched in stage 1', C["arrow_green"])
    # Lost -> Removed
    draw_straight_arrow(ax, 8.5, 3.5, 8.5, 2.0,
                        'frame_id diff > max_age', C["arrow_red"])

    save_fig("strack_sm.svg", fig)


# ================================================================
# 4. 关联阶段细节
# ================================================================

def draw_association_detail():
    fig, ax = plt.subplots(1, 1, figsize=(10, 6))
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 6)
    ax.axis('off')
    ax.set_title('Association Stage Detail — IoU Distance + Linear Assignment',
                 fontsize=14, fontweight='bold', pad=10)

    C = COLORS

    draw_rounded_box(ax, 0.5, 4.0, 2.0, 1.0, "Track Set A\n(N tracks)", C["state_new"], 9, True)
    draw_rounded_box(ax, 0.5, 2.0, 2.0, 1.0, "Detection Set B\n(M detections)", C["state_new"], 9, True)

    draw_rounded_box(ax, 3.5, 3.0, 3.0, 1.5,
                     "calcIouDistance()\n\n"
                     "For each pair (i,j):\n"
                     "cost[i][j] = 1 - IoU\n"
                     "(skip different labels)",
                     C["internal_box"], 8)

    draw_rounded_box(ax, 7.0, 3.0, 2.5, 1.5,
                     "linearAssignment()\n\n"
                     "→ execLapjv()\n"
                     "Jonker-Volgenant\nalgorithm",
                     C["state_lost"], 8)

    draw_rounded_box(ax, 1.0, 0.3, 2.5, 1.0,
                     "Matched pairs\n• Tracked → update()\n• Lost → reActivate()",
                     C["python_box"], 8)
    draw_rounded_box(ax, 4.0, 0.3, 2.5, 1.0,
                     "Unmatched tracks\n→ next stage / mark Lost",
                     C["state_lost"], 8)
    draw_rounded_box(ax, 7.0, 0.3, 2.5, 1.0,
                     "Unmatched detections\n→ next stage / init new",
                     C["state_lost"], 8)

    draw_straight_arrow(ax, 2.5, 4.5, 3.5, 3.8, '', '#666')
    draw_straight_arrow(ax, 2.5, 2.5, 3.5, 3.8, '', '#666')
    draw_straight_arrow(ax, 6.5, 3.8, 7.0, 3.8, 'cost matrix', '#666')
    draw_straight_arrow(ax, 8.25, 3.0, 8.25, 1.3, 'rowsol[i] ≥ 0', C["arrow_green"])
    draw_straight_arrow(ax, 6.0, 3.0, 2.5, 1.0, 'rowsol[i] == -1', C["arrow_red"])
    draw_straight_arrow(ax, 8.25, 3.0, 8.25, 1.3, 'colsol[i] == -1', C["arrow_red"])

    # 代价矩阵示意图标注
    ax.plot([3.5, 6.5, 6.5, 3.5, 3.5], [3.0, 3.0, 4.5, 4.5, 3.0], 'gray', lw=0.5, ls=':')
    ax.text(5.0, 3.3, 'N×M matrix', fontsize=7, ha='center', color='gray', style='italic')

    save_fig("association_detail.svg", fig)


# ================================================================
# 5. 卡尔曼滤波器
# ================================================================

def draw_kalman_filter():
    fig, ax = plt.subplots(1, 1, figsize=(10, 7))
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 7)
    ax.axis('off')
    ax.set_title('Kalman Filter — 8-Dim Constant-Velocity Model',
                 fontsize=14, fontweight='bold', pad=10)

    C = COLORS

    draw_rounded_box(ax, 3.0, 5.5, 4.0, 1.0,
                     "State: 1×8  [cx, cy, aspect, height, vx, vy, va, vh]\n"
                     "Covariance: 8×8",
                     C["state_new"], 8)

    draw_rounded_box(ax, 1.0, 3.5, 3.0, 1.2,
                     "initiate(mean, cov, measurement)\n\n"
                     "mean = [cx,cy,ar,h, 0,0,0,0]\n"
                     "cov = diag(std²)",
                     C["python_box"], 8)

    draw_rounded_box(ax, 5.0, 3.5, 3.0, 1.2,
                     "predict(mean, cov)\n\n"
                     "mean = F · mean\n"
                     "cov = F·cov·Fᵀ + Q",
                     C["internal_box"], 8)

    draw_rounded_box(ax, 5.0, 1.5, 3.0, 1.2,
                     "update(mean, cov, measurement)\n\n"
                     "K = Kalman gain\n"
                     "mean += K · innovation\n"
                     "cov -= K·H·cov",
                     C["python_box"], 8)

    draw_rounded_box(ax, 8.5, 5.5, 1.5, 0.6, "F: motion_mat\n(8×8)", C["note_box"], 7)
    draw_rounded_box(ax, 8.5, 4.5, 1.5, 0.6, "H: update_mat\n(4×8)", C["note_box"], 7)
    draw_rounded_box(ax, 8.5, 3.5, 1.5, 0.6, "Q: motion\ncov", C["note_box"], 7)

    draw_straight_arrow(ax, 4.0, 6.0, 4.0, 4.7, 'new track', '#666')
    draw_straight_arrow(ax, 4.0, 4.7, 5.0, 4.2, '→ predict', '#666')
    draw_straight_arrow(ax, 8.0, 4.2, 8.0, 2.7, '→ update', '#666')
    draw_arrow(ax, 5.0, 3.5, 5.0, 2.7, 'match', C["arrow_green"])
    draw_arrow(ax, 8.5, 5.5, 7.0, 5.5, 'F', '#999')
    draw_arrow(ax, 8.5, 4.5, 7.0, 4.5, 'H', '#999')
    draw_arrow(ax, 8.5, 3.5, 7.0, 3.5, 'Q', '#999')

    # 噪声参数说明
    ax.text(2.0, 2.0,
            'Noise scales with detection height:\n'
            '  std_weight_position = 1/20\n'
            '  std_weight_velocity = 1/160',
            fontsize=7, color='#666', ha='center', family='sans-serif',
            bbox=dict(boxstyle='round,pad=0.3', facecolor=C["action_highlight"],
                      edgecolor='#F0E68C'))

    save_fig("kalman_filter.svg", fig)


# ================================================================
# 6. IoU 距离计算
# ================================================================

def draw_iou_distance():
    fig, ax = plt.subplots(1, 1, figsize=(8, 5))
    ax.set_xlim(0, 8)
    ax.set_ylim(0, 5)
    ax.axis('off')
    ax.set_title('calcIouDistance()', fontsize=14, fontweight='bold', pad=10)

    C = COLORS

    draw_rounded_box(ax, 0.5, 3.5, 2.5, 0.8, "a_tracks: N STrackPtr", C["state_new"], 9, True)
    draw_rounded_box(ax, 0.5, 2.5, 2.5, 0.8, "b_tracks: M STrackPtr", C["state_new"], 9, True)

    draw_rounded_box(ax, 4.0, 2.5, 3.5, 1.5,
                     "For each pair (i, j):\n\n"
                     "1. Skip if labels differ\n"
                     "2. Compute IoU\n"
                     "3. cost[i][j] = 1 - IoU",
                     C["internal_box"], 9)

    draw_rounded_box(ax, 2.0, 0.5, 4.0, 1.0,
                     "Output: N×M cost matrix (range [0, 1])\n"
                     "empty if a_tracks or b_tracks is empty",
                     C["python_box"], 9, True)

    draw_rounded_box(ax, 0.5, 0.5, 1.5, 0.6,
                     "IoU formula:\n"
                     "inter/(area_a\n"
                     "+area_b-inter)",
                     C["state_lost"], 7)

    draw_straight_arrow(ax, 3.0, 3.9, 4.0, 3.5, '', '#666')
    draw_straight_arrow(ax, 3.0, 2.9, 4.0, 3.0, '', '#666')
    draw_straight_arrow(ax, 5.5, 2.5, 5.5, 1.5, '', '#666')
    draw_arrow(ax, 2.0, 1.0, 1.5, 1.0, '', '#666')

    save_fig("iou_distance.svg", fig)


# ================================================================
# 7. 去重策略
# ================================================================

def draw_remove_duplicate():
    fig, ax = plt.subplots(1, 1, figsize=(8, 4))
    ax.set_xlim(0, 8)
    ax.set_ylim(0, 4)
    ax.axis('off')
    ax.set_title('removeDuplicateStracks — Deduplication Logic',
                 fontsize=14, fontweight='bold', pad=10)

    C = COLORS

    draw_rounded_box(ax, 0.5, 2.5, 2.0, 0.8, "tracked_stracks", C["state_new"], 9, True)
    draw_rounded_box(ax, 0.5, 1.5, 2.0, 0.8, "lost_stracks", C["state_new"], 9, True)

    draw_rounded_box(ax, 3.5, 1.5, 2.5, 1.5,
                     "calcIouDistance(a, b)\n→ N×M cost matrix",
                     C["internal_box"], 9)

    draw_rounded_box(ax, 6.5, 1.5, 1.5, 1.5,
                     "cost < 0.15\n→ IoU > 0.85\n→ overlap!",
                     C["state_lost"], 8, True)

    draw_rounded_box(ax, 1.5, 0.3, 2.0, 0.8, "tracked_out\n(no duplicates)",
                     C["python_box"], 9, True)
    draw_rounded_box(ax, 4.5, 0.3, 2.0, 0.8, "lost_out\n(no duplicates)",
                     C["python_box"], 9, True)

    draw_straight_arrow(ax, 2.5, 2.9, 3.5, 2.8, '', '#666')
    draw_straight_arrow(ax, 2.5, 1.9, 3.5, 2.0, '', '#666')
    draw_straight_arrow(ax, 6.0, 2.2, 6.5, 2.2, '', '#666')
    draw_arrow(ax, 4.0, 1.5, 2.5, 1.0, 'keep longer tracklet', C["arrow_green"])
    draw_arrow(ax, 4.0, 1.5, 5.5, 1.0, 'drop shorter', C["arrow_red"])

    save_fig("remove_duplicate.svg", fig)


# ================================================================
# 主函数
# ================================================================

def main():
    print("=" * 60)
    print("ByteTrackLib 架构流程图生成器")
    print(f"输出目录: {BASE}")
    print("=" * 60)

    draw_architecture()
    draw_bytetrack_flow()
    draw_strack_sm()
    draw_association_detail()
    draw_kalman_filter()
    draw_iou_distance()
    draw_remove_duplicate()

    print("\n" + "=" * 60)
    print("全部 SVG 图片已生成完毕！")
    print("=" * 60)


if __name__ == "__main__":
    main()