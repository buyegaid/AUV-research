---
type: paper
node_id: paper:fossen2015_adaptive_los
title: "Line-of-Sight Path Following for Dubins Paths with Adaptive Sideslip Compensation of Drift Forces"
authors: ["Thor I. Fossen", "Kristin Y. Pettersen", "Roberto Galeazzi"]
year: 2015
venue: "IEEE Transactions on Control Systems Technology"
external_ids:
  arxiv: null
  doi: "10.1109/TCST.2014.2338354"
  s2: null
tags: ["ALOS", "sideslip-compensation", "path-following", "USGES"]
added: 2026-06-09T00:00:00Z
---

# Line-of-Sight Path Following for Dubins Paths with Adaptive Sideslip Compensation of Drift Forces

## One-line thesis
Adaptive LOS guidance law estimates unknown sideslip angle (crab angle) due to ocean currents, achieving USGES convergence for path following on Dubins paths.

## Problem / Gap
传统 LOS 制导假设无侧滑角，海流引起的 crab angle 导致稳态横向误差。积分 LOS（ILOS）处理恒定侧滑，但对时变侧滑响应慢。

## Method
- **自适应 LOS（ALOS）**：自适应律估计 crab angle β̂，加性注入航向指令
- 关键创新：β̂ 直接加到航向角（扰动匹配），而非通过 arctan 饱和（ILOS 的做法）
- 均匀半全局指数稳定（USGES）证明
- 扩展到 Dubins 路径（直线 + 圆弧）

## Key Results
- ALOS 在时变侧滑场景下优于 ILOS
- USGES 保证了快速收敛
- REMUS 100 AUV 案例验证

## Assumptions
- 侧滑角缓变（自适应律带宽内）
- 路径曲率连续

## Limitations / Failure Modes
- 仅水平面，不处理垂向流
- 自适应律对快速方向变化的流可能滞后
- 需要精确的航向测量

## Reusable Ingredients
- 扰动匹配（disturbance matching）范式：估计值加性注入控制律
- USGES 稳定性框架
- ALOS 可作为 EG-UCCO 的前端制导模块

## Open Questions
- ALOS 的 crab angle 估计 vs EG-UCCO 的 2D 海流估计的关系？

## Claims
_TODO._

## Connections
_Edges are recorded in `graph/edges.jsonl`._

## Relevance to This Project
**中等相关**。ALOS 代表了制导层面的海流补偿（通过 crab angle），EG-UCCO 是动力学层面（通过力/力矩）。两者互补：ALOS 处理运动学效应（侧滑），EG-UCCO 处理动力学效应（阻力和力矩扰动）。论文引言中可引用作为运动学+动力学分层处理的论据。
