---
type: paper
node_id: paper:bai2024_trajectory_tracking_control
title: "Trajectory Tracking Control of Underactuated AUV Based on Ocean Current Observer"
authors: ["Jianjun Bai", "Zhiyao Liu", "Yun Chen"]
year: 2024
venue: "IEEE ICUS 2024"
external_ids:
  arxiv: null
  doi: "10.1109/ICUS61736.2024.10840048"
  s2: null
tags: ["current-observer", "RBFNN", "backstepping", "kinematic"]
added: 2026-05-26T07:21:29Z
---

# Trajectory Tracking Control of Underactuated AUV Based on Ocean Current Observer

## One-line thesis
RBFNN-based ocean current observer combined with backstepping achieves planar AUV trajectory tracking under current disturbances.

## Problem / Gap
欠驱动 AUV 在水平面轨迹跟踪中海流扰动导致跟踪误差增大，传统运动学观测器收敛慢。需要一种能在线学习海流特性的观测器。

## Method
- **RBF 神经网络海流观测器**：利用径向基函数网络在线逼近海流速度分量，输入为 AUV 位置和速度信息
- **Backstepping 控制器**：基于观测器输出的海流估计进行前馈补偿
- **视线法制导（LOS）**：生成期望航向角
- 仅考虑水平面（surge/sway/yaw）3-DOF

## Key Results
- 仿真验证（REMUS 100 模型）
- RBFNN 观测器能有效估计恒定和时变海流
- Backstepping + 前馈补偿优于无补偿基线

## Assumptions
- 海流在水平面内均匀分布
- 欠驱动 AUV（无侧向推进器）
- 海流变化速率有界
- RBFNN 需要离线训练数据或在线学习

## Limitations / Failure Modes
- RBFNN 需要训练数据，泛化到未见海流场景存疑
- 仅水平面，未扩展到 3D
- 计算复杂度高（神经网络在线推理）
- 会议论文，缺少海试验证

## Reusable Ingredients
- RBFNN 观测器结构可作为学习型观测器的基线
- 海流-动力学解耦思路（观测器 + 独立控制器）

## Open Questions
- RBFNN 在线学习 vs 离线训练的性能差异？
- 能否推广到时变剪切流？
- 与 CFD 物理模型相比的优势？

## Claims
_TODO._

## Connections
_Edges are recorded in `graph/edges.jsonl`; summarize here for human readers._

## Relevance to This Project
**中等相关**。RBFNN 海流观测器属于数据驱动范式，与 EG-UCCO 的 CFD 物理先验范式形成对比。RQ：能否用 CFD 先验替换 RBFNN 的训练数据需求？EG-UCCO 论文中可作为数据驱动观测器的代表性引用。
