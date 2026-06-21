---
type: paper
node_id: paper:xu2024_robust_prescribedtime_esobased
title: "Robust Prescribed-Time ESO-Based Practical Predefined-Time SMC for Benthic AUV Trajectory-Tracking Control with Uncertainties and Environment Disturbance"
authors: ["Yufei Xu", "Ziyang Zhang", "Lei Wan"]
year: 2024
venue: "Journal of Marine Science and Engineering"
external_ids:
  arxiv: null
  doi: "10.3390/jmse12061014"
  s2: null
tags: ["prescribed-time-ESO", "predefined-time-SMC", "benthic-AUV", "adaptive-law"]
added: 2026-05-26T07:20:42Z
---

# Robust Prescribed-Time ESO-Based Practical Predefined-Time SMC for Benthic AUV Trajectory-Tracking Control with Uncertainties and Environment Disturbance

## One-line thesis
Prescribed-time ESO combined with predefined-time SMC achieves robust AUV trajectory tracking under model uncertainties and environmental disturbances, with convergence time set as explicit parameters.

## Problem / Gap
传统 ESO 和 SMC 的收敛时间依赖于初始条件和控制器增益，无法预先指定。海底 AUV（benthic AUV）在近底作业时需要精确的时间约束（如避障、对接）。

## Method
- **鲁棒规定时间 ESO（RPTESO）**：收敛时间作为显式参数直接设置，不依赖初始条件或复杂调谐，含自适应律增强鲁棒性
- **非奇异实用预定义时间 SMC（RPPSMC）**：收敛时间预定义，非奇异设计避免控制奇异，考虑 AUV 水动力特性
- **自适应律**：降低 RPTESO 的超调
- 理论分析 + 仿真验证

## Key Results
- RPTESO 在规定时间内准确估计集总扰动
- RPPSMC 轨迹跟踪误差在预定义时间内收敛
- 与传统有限时间和固定时间控制对比优势明显
- JMSE 开源论文，哈尔滨工程大学（国内 AUV 强校）

## Assumptions
- 集总扰动有界
- 规定时间作为参数显式给出
- AUV 速度可测（或通过 RPTESO 重构）

## Limitations / Failure Modes
- 仍为集总扰动框架
- 规定时间参数选择影响控制能量（时间越短力越大）
- 海底 AUV 场景特殊（近底效应），泛化性存疑
- 仅仿真，无近底海试
- 非奇异设计增加复杂度

## Reusable Ingredients
- 规定时间收敛的理论框架（可对比 EG-UCCO 的收敛分析）
- 自适应律抑制超调的技术
- 海底 AUV 的特殊动力学考虑

## Open Questions
- 规定时间 ESO 是否能嵌入物理海流模型？
- 规定时间 vs 激励门控的收敛速度对比？

## Claims
_TODO._

## Connections
_Edges are recorded in `graph/edges.jsonl`; summarize here for human readers._

## Relevance to This Project
**中等相关**。代表 ESO 收敛时间可设计化的最新趋势。关键区分：RPTESO 关注"多快收敛"，EG-UCCO 关注"何时更新"。论文中作为指定时间 ESO 的代表引用，可在讨论中对比两方法的适用场景（时间关键 vs 鲁棒关键）。
