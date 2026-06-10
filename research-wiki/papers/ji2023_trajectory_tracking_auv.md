---
type: paper
node_id: paper:ji2023_trajectory_tracking_auv
title: "Trajectory Tracking of AUV under Current Disturbance Based on Adaptive LADRC"
authors: ["Daxiong Ji", "Xinwei Wang", "Minghui Xu", "Qiang Zhang", "Sanjay Sharma", "Robert Sutton"]
year: 2023
venue: "ICARCE 2023"
external_ids:
  arxiv: null
  doi: "10.1109/ICARCE59252.2024.10492523"
  s2: null
tags: ["adaptive-LADRC", "ESO", "online-tuning", "current-disturbance"]
added: 2026-05-26T07:21:28Z
---

# Trajectory Tracking of AUV under Current Disturbance Based on Adaptive LADRC

## One-line thesis
Adaptive LADRC with online ESO bandwidth tuning effectively estimates and rejects ocean current disturbances in AUV 3D trajectory tracking.

## Problem / Gap
传统 LADRC（线性自抗扰控制）的 ESO 带宽固定，无法适应时变海流场景。固定高带宽带来噪声放大，固定低带宽导致估计滞后。

## Method
- **自适应 LADRC**：利用估计误差在线自动调整 ESO 参数（带宽 ω₀）
- 根据扰动估计精度动态调节观测器增益
- 标准 3D AUV 轨迹跟踪框架

## Key Results
- 仿真验证（与传统 LADRC 对比）
- 自适应 LADRC 有效估计外部海流扰动
- 3D 轨迹跟踪精度优于固定参数 LADRC
- 国际合作（浙江大学 + 普利茅斯大学）

## Assumptions
- 海流扰动可被 ESO 扩展状态捕获
- 集总扰动假设（不区分海流和模型误差）
- 在线调谐律需梯度信息

## Limitations / Failure Modes
- 在线调谐增加计算量
- 调谐律收敛需要持续激励
- 会议论文，篇幅有限，缺乏系统实验
- 仍为集总扰动框架

## Reusable Ingredients
- 自适应带宽调谐策略（可对比 EG-UCCO 的 Gramian 门控）
- ω₀ 在线调谐的梯度准则

## Open Questions
- 自适应 LADRC vs EG-UCCO 在模型失配下的对比？
- 带宽调谐与 Gramian 门控的等价/互补关系？

## Claims
_TODO._

## Connections
_Edges are recorded in `graph/edges.jsonl`; summarize here for human readers._

## Relevance to This Project
**高度相关**。自适应 LADRC 是 EG-UCCO 的"激励自适应"维度的最接近对比方法。关键差异：LADRC 调的是 ESO 带宽（基于估计误差），EG-UCCO 门控的是更新与否（基于 Fisher 信息）。论文中可作为自适应 ESO 的代表引用。
