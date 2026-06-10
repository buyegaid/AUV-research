---
type: paper
node_id: paper:kang2020_antidisturbance_control_auv
title: "Antidisturbance Control for AUV Trajectory Tracking Based on Fuzzy Adaptive Extended State Observer"
authors: ["Song Kang", "Yongfeng Rong", "Wusheng Chou"]
year: 2020
venue: "Sensors"
external_ids:
  arxiv: null
  doi: "10.3390/s20247084"
  s2: null
tags: ["fuzzy-ESO", "dynamic-surface-control", "actuator-fault", "output-feedback"]
added: 2026-05-26T07:20:44Z
---

# Antidisturbance Control for AUV Trajectory Tracking Based on Fuzzy Adaptive Extended State Observer

## One-line thesis
Fuzzy adaptive ESO combined with dynamic surface control achieves AUV trajectory tracking robust to disturbances, parameter uncertainties, and actuator faults.

## Problem / Gap
AUV 轨迹跟踪面临多重不确定性：(1) 外部海流扰动；(2) 水动力参数不确定性；(3) 执行器故障。传统 ESO 固定增益无法同时应对这三类不确定性。

## Method
- **模糊自适应 ESO（FAESO）**：模糊逻辑在线调节 ESO 带宽，依据估计残差自适应增益
- **动态面控制（DSC）**：模糊调节低通滤波器时间常数，避免 backstepping 的"微分爆炸"
- **输出反馈架构**：无需全状态测量
- **Lyapunov 稳定性证明**：渐近稳定
- 执行器故障容错：部分失效场景下仍保持稳定

## Key Results
- 多场景对比仿真（与传统 ESO+DSC、PID 对比）
- FAESO 在扰动估计精度上优于固定增益 ESO
- 执行器故障下仍保持可接受的跟踪性能
- 控制能量效率更高
- 已获 30+ 引用，MDPI 开源论文

## Assumptions
- 执行器故障模式已知（部分失效）
- 模糊规则库需预先设计
- 集总扰动框架

## Limitations / Failure Modes
- 模糊规则依赖专家经验，通用性受限
- 仅仿真验证，无实物实验
- 隶属度函数和规则库的设计未给出系统方法
- 仍不区分海流和模型误差

## Reusable Ingredients
- 模糊自适应增益策略（可对比 EG-UCCO 的 Gramian 门控）
- DSC 避免微分爆炸的技术
- 输出反馈架构（可降低传感器需求）

## Open Questions
- 模糊自适应 vs Gramian 门控的本质差异？
- FAESO 在模型失配（非执行器故障）下的表现？

## Claims
_TODO._

## Connections
_Edges are recorded in `graph/edges.jsonl`; summarize here for human readers._

## Relevance to This Project
**高度相关**。FAESO 代表了 ESO 参数自适应的模糊逻辑路线，与 EG-UCCO 的 Fisher 信息门控形成方法对比。论文中可作为自适应 ESO 的另一代表，并讨论启发式（模糊规则）vs 原则性（Fisher 信息）的优劣。
