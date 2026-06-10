---
type: paper
node_id: paper:wang2024_accurate_trajectory_tracking
title: "Accurate trajectory tracking control for AUV under state constraints with a rapid stability dimensionality-augmented state observer"
authors: ["Jianhui Wang", "Haoyuan Wang", "Zikai Hu", "Jiarui Liu", "Kairui Chen"]
year: 2024
venue: "International Journal of Robust and Nonlinear Control"
external_ids:
  arxiv: null
  doi: "10.1002/rnc.7591"
  s2: null
tags: ["fixed-time-ESO", "state-constraints", "event-triggered", "dimensionality-augmented"]
added: 2026-05-26T07:21:27Z
---

# Accurate trajectory tracking control for AUV under state constraints with a rapid stability dimensionality-augmented state observer

## One-line thesis
Fast fixed-time ESO combined with fixed-time SMC achieves AUV trajectory tracking under state constraints and unknown disturbances.

## Problem / Gap
AUV 在实际运行中面临：(1) 状态约束（速度、角速度不能无限大）；(2) 通信资源有限（水声通信带宽窄）；(3) 模型不确定性和时变扰动。现有方法难以同时处理这三者。

## Method
- **快速稳定降维增强状态观测器（RSDASO）**：固定时间收敛的 ESO，估计误差在固定时间内收敛（与初始误差无关）
- **固定时间轨迹跟踪控制器**：跟踪误差固定时间收敛
- **事件触发机制（ETM）**：减少控制器更新和通信频次
- **障碍 Lyapunov 函数（BLF）**：处理状态约束

## Key Results
- 理论证明：固定时间收敛 + 状态约束满足
- 仿真验证（与传统固定时间控制和有限时间控制对比）
- ETM 有效减少控制更新次数（>40%）
- 发表于 IJRNC（控制领域权威期刊，IF~5.0）

## Assumptions
- 扰动满足 Lipschitz 条件
- 状态约束边界已知
- 事件触发阈值需预先设定

## Limitations / Failure Modes
- 固定时间收敛参数整定复杂
- 事件触发在高扰动下可能频繁触发（退化到连续控制）
- RSDASO 仍是集总扰动框架
- 仅仿真，无水试/海试

## Reusable Ingredients
- 固定时间收敛的理论框架（可对比分析 EG-UCCO 的收敛特性）
- 事件触发机制（可启发 EG-UCCO 的按需更新）
- 障碍 Lyapunov 函数（约束处理通用工具）

## Open Questions
- RSDASO 是否能分离海流和模型误差？
- 事件触发 vs 激励门控的异同？

## Claims
_TODO._

## Connections
_Edges are recorded in `graph/edges.jsonl`; summarize here for human readers._

## Relevance to This Project
**中等相关**。RSDASO 的固定时间收敛和事件触发机制与 EG-UCCO 的激励门控在"何时更新"问题上有共通之处。差异：RSDASO 关注减少通信（ETM），EG-UCCO 关注 Fisher 信息充足性。论文中可作为固定时间 ESO 和事件触发机制的代表引用。
