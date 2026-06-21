---
type: paper
node_id: paper:jaulin2009_set_membership
title: "Robust Set-Membership State Estimation; Application to Underwater Robotics"
authors: ["Luc Jaulin"]
year: 2009
venue: "Automatica"
external_ids:
  arxiv: null
  doi: "10.1016/j.automatica.2008.06.013"
  s2: null
tags: ["set-membership", "bounded-error", "interval-analysis", "underwater"]
added: 2026-06-09T00:00:00Z
---

# Robust Set-Membership State Estimation; Application to Underwater Robotics

## One-line thesis
Nonlinear observer using interval analysis and unknown-but-bounded error model guarantees that the true state lies within the computed feasible set, even with outliers.

## Problem / Gap
传统 Kalman 滤波假设高斯噪声，但在水下环境中传感器异常值（声学多路径、DVL 底跟踪丢失）经常违反此假设，导致估计发散。

## Method
- **区间分析**：用区间向量（boxes）而非点估计表示状态
- **约束传播**：前向和后向传播测量和动力学约束，收缩可行集
- **无线性化/近似**：精确处理非线性
- 假设测量误差为"未知但有界"（UBB），而非高斯

## Key Results
- 游泳池水下机器人定位和控制验证
- 在异常值存在下保持状态包含保证
- EKF 在有偏测量下发散时，set-membership 方法保持有效

## Assumptions
- 测量和模型误差边界已知
- 非线性函数可表示为包含函数（inclusion function）

## Limitations / Failure Modes
- 区间传播的包络效应（wrapping effect）导致保守性
- 计算量随状态维数指数增长（维度诅咒）
- 仅提供集合，不提供点估计
- AUV 6-DOF 高维问题计算挑战大

## Reusable Ingredients
- UBB 误差模型（vs 高斯假设）的思想
- Set-membership 更新规则：预测 + 校正（交集）的结构
- 状态包含保证（guaranteed containment）的概念

## Open Questions
- 能否将区间分析降维（如仅对海流分量用 UBB）降低计算量？

## Claims
_TODO._

## Connections
_Edges are recorded in `graph/edges.jsonl`._

## Relevance to This Project
**高度相关**。Set-membership 估计是 EG-UCCO 不确定性界的理论基础。EG-UCCO 的"模型不确定性界 Δ_model"和"死区机制"直接借鉴了 UBB 思想：残差在界内→归因于模型误差；残差超出界→归因于海流。论文方法部分可引用 Jaulin 作为 bounded-error 范式的源头。
