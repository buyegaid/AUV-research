---
type: paper
node_id: paper:he2025_threedimensional_trajectory_tracking
title: "Three-Dimensional Trajectory Tracking Control of Underactuated AUV Based on Fractional-Order PID and Super-Twisting Extended State Observer"
authors: ["Long He", "Ya Zhang", "Mengting Xie", "Zehui Yuan", "Chenrui Bai"]
year: 2025
venue: "Fractal and Fractional"
external_ids:
  arxiv: null
  doi: "10.3390/fractalfract9090580"
  s2: null
tags: ["super-twisting-ESO", "fractional-order-PID", "finite-time", "3D-tracking"]
added: 2026-05-26T07:20:46Z
---

# Three-Dimensional Trajectory Tracking Control of Underactuated AUV Based on Fractional-Order PID and Super-Twisting Extended State Observer

## One-line thesis
Super-twisting ESO provides finite-time disturbance estimation for fractional-order PID controller in 3D AUV trajectory tracking.

## Problem / Gap
欠驱动 AUV 在复杂海洋环境（动态扰动 + 模型不确定性）中，传统 PID 和线性 ESO 存在：(1) PID 频域调谐灵活性不足；(2) 线性 ESO 收敛速度依赖高增益带来噪声放大。

## Method
- **超螺旋扩张状态观测器（STESO）**：二阶滑模结构，有限时间收敛，自适应增益避免过估计，无需扰动先验知识
- **分数阶 PID 控制器（FOPID）**：利用分数阶微积分（额外的积分/微分阶次参数）扩大频域调谐空间，提升鲁棒性
- **Lyapunov 稳定性分析**：证明扰动估计误差消失时跟踪误差渐近收敛
- 多场景数值仿真验证

## Key Results
- 4 种控制方案对比仿真（多种扰动场景）
- STESO 有限时间估计优于传统线性 ESO
- FOPID 在扰动抑制和跟踪精度上优于整数阶 PID
- 分数阶微积分的额外自由度带来更平滑的控制信号

## Assumptions
- 海流扰动满足 Lipschitz 条件
- AUV 速度可测
- 欠驱动配置（舵面 + 单推进器）

## Limitations / Failure Modes
- 超螺旋算法在低激励场景可能收敛慢
- 分数阶 PID 参数整定复杂（5-6 个参数 vs 3 个）
- 仅仿真验证，无水池/海试
- STESO 仍为集总扰动框架
- 中北大学团队，非海洋工程传统强校

## Reusable Ingredients
- STESO 的有限时间收敛特性可对比 EG-UCCO 的激励门控
- FOPID 作为分数阶控制基线
- Lyapunov 分析框架

## Open Questions
- STESO vs 速度层面预测-校正（EG-UCCO 方案）的 SNR 对比？
- 分数阶 PID vs SMC 在 AUV 场景下的对比？

## Claims
_TODO._

## Connections
_Edges are recorded in `graph/edges.jsonl`; summarize here for human readers._

## Relevance to This Project
**中等相关**。STESO 是 ESO 的滑模变体，代表 ESO 方法的"高阶化"趋势。EG-UCCO 走的是另一条路（CFD 先验 + 门控），可在论文中作为 ESO 类方法的最新代表引用。同作者 He et al. (2024) 运动学观测器更相关。
