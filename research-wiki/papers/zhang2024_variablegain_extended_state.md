---
type: paper
node_id: paper:zhang2024_variablegain_extended_state
title: "Variable-Gain Extended State Observer-Based Data-Driven Trajectory Tracking Control of Unmanned Marine Vehicles Under Disturbances"
authors: ["Xuqi Zhang", "Lihua Hao", "Huiying Liu"]
year: 2024
venue: "IEEE ONCON 2024"
external_ids:
  arxiv: null
  doi: "10.1109/ONCON62778.2024.10931469"
  s2: null
tags: ["variable-gain-ESO", "data-driven", "peaking-phenomenon", "marine-vehicle"]
added: 2026-05-26T07:21:30Z
---

# Variable-Gain Extended State Observer-Based Data-Driven Trajectory Tracking Control of Unmanned Marine Vehicles Under Disturbances

## One-line thesis
Variable-gain ESO mitigates the peaking phenomenon while maintaining disturbance estimation accuracy for marine vehicle trajectory tracking.

## Problem / Gap
传统线性 ESO 在初始阶段因高增益产生"peaking 现象"（估计值大幅超调），可能导致控制饱和甚至失稳。固定增益 ESO 无法在收敛速度和 peaking 抑制之间兼顾。

## Method
- **变增益 ESO（Variable-Gain ESO）**：初始阶段低增益抑制 peaking，随后增益递增至稳态值保证精度
- **数据驱动轨迹跟踪控制**：结合无模型自适应控制（MFAC）或迭代学习控制
- 针对无人海洋航行器（UMV）——比 AUV 更宽泛的类别

## Key Results
- 变增益 ESO 有效抑制 peaking（超调降低 >50%）
- 稳态估计精度与固定高增益 ESO 相当
- 数据驱动部分减少对动力学模型的依赖
- IEEE ONCON 2024 会议论文

## Assumptions
- peaking 主要由初始估计误差引起
- 增益变化律需预先设计（时变函数）
- 数据驱动部分需要持续激励

## Limitations / Failure Modes
- 变增益律设计缺乏通用准则（启发式）
- 数据驱动方法对传感器噪声敏感
- 会议论文，篇幅和实验有限
- 仍为集总扰动框架
- UMV 范围宽泛，AUV 针对性不足

## Reusable Ingredients
- 变增益策略（可对比 EG-UCCO 的激励门控：都是"选择性更新"，但方法不同）
- Peaking 问题的系统性识别（EG-UCCO 也需考虑初始协方差大时的 peaking）

## Open Questions
- 变增益 vs 激励门控在 peaking 抑制上的对比？
- 能否将 Gramian 门控与变增益结合？

## Claims
_TODO._

## Connections
_Edges are recorded in `graph/edges.jsonl`; summarize here for human readers._

## Relevance to This Project
**高度相关**。变增益 ESO 的 peaking 问题与 EG-UCCO 在初始阶段的门控行为直接相关。差异点：变增益是时变标量增益（启发式），EG-UCCO 是 Fisher 信息驱动的二元门控 + 协方差管理。可在论文中作为 peaking 问题的引出文献，并讨论 EG-UCCO 如何通过激励门控自然避免 peaking（无激励=不更新=不 peaking）。
