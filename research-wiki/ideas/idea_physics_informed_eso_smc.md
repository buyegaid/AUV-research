---
type: idea
node_id: idea:physics_informed_eso_smc
title: "Physics-Informed ESO-SMC for Spatially Correlated Ocean Currents"
stage: proposed
outcome: unknown
based_on:
  - paper:ji2023_trajectory_tracking_auv
  - paper:kang2020_antidisturbance_control_auv
target_gaps:
  - gap:G1_physical_current_modeling
  - gap:G3_complex_current_validation
added: 2026-05-26
---

# 物理信息 ESO-SMC（针对空间相关海流）

## 假设
将低频、空间相关的海流物理模型（如一阶 Gauss-Markov 流）嵌入 ESO，比通用集总扰动 ESO 具有更高的估计精度和更低的 SMC 增益需求。

## 方法
- 在 ESO 状态方程中加入海流动力学模型（低频、空间相关）
- ESO 估计海流扰动，前馈补偿到 SMC
- SMC 仅处理残余不确定性，增益可减小

## 预期结果
物理信息 ESO 在低频海流下估计误差降低 30-50%，SMC 增益减小，抖振减轻。

## 最小实验
XHY MATLAB 仿真，4 种海流场景，对比纯 SMC / 经典 ESO+SMC / 物理信息 ESO+SMC。

## 风险
MEDIUM — 物理模型参数选择影响结果，需敏感性分析。

## 目标期刊
IEEE Journal of Oceanic Engineering 或 Ocean Engineering
