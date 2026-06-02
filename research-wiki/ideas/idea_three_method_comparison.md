---
type: idea
node_id: idea:three_method_comparison
title: "Quantitative Comparison: SMC vs ISMC vs ESO-SMC Under Identical Current Fields"
stage: proposed
outcome: unknown
based_on:
  - paper:kang2020_antidisturbance_control_auv
  - paper:xu2024_robust_prescribedtime_esobased
target_gaps:
  - gap:G6_quantitative_comparison
added: 2026-05-26
---

# SMC vs ISMC vs ESO-SMC 三方法定量对比

## 假设
ESO+SMC 在海流缓变且可部分观测时优势最大；ISMC 在匹配有界扰动下性能相近但调参更简单。

## 方法
- 在相同 XHY 模型、相同路径、相同执行器限制下对比三种方法
- 报告跟踪精度、能量、抖振、调参敏感性、模型失配鲁棒性

## 预期结果
明确各方法的适用场景边界，为工程选型提供依据。

## 最小实验
与现有 `main_loop_remus` 框架直接对接，工作量最小。

## 风险
LOW — 主要挑战是确保公平调参。

## 目标期刊
IEEE Access 或 Ocean Engineering
