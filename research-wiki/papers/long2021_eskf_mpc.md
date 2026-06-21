---
type: paper
node_id: paper:long2021_eskf_mpc
title: "Trajectory Tracking Control of ROVs Considering External Disturbances and Measurement Noises Using ESKF-Based MPC"
authors: ["Chengqi Long", "Xiaohui Qin", "Yougang Bian", "Manjiang Hu"]
year: 2021
venue: "Ocean Engineering"
external_ids:
  arxiv: null
  doi: "10.1016/j.oceaneng.2021.110012"
  s2: null
tags: ["ESKF", "MPC", "current-estimation", "EKF-baseline"]
added: 2026-06-09T00:00:00Z
---

# Trajectory Tracking Control of ROVs Considering External Disturbances and Measurement Noises Using ESKF-Based MPC

## One-line thesis
Extended State Kalman Filter (ESKF) jointly estimates system states and external current disturbances, then feeds into MPC for constrained trajectory tracking.

## Problem / Gap
海流扰动和测量噪声同时存在时，传统 MPC 假设完美状态反馈不成立。需要同时估计状态和海流。

## Method
- **扩展状态卡尔曼滤波（ESKF）**：将海流速度增广为状态向量，联合估计
- 海流建模为速度（非力），避免力的不可测问题
- **ESKF 基 MPC**：估计状态 + 海流 → MPC 预测模型 → 前馈补偿
- 处理传感器噪声和海流扰动的统一框架

## Key Results
- ESKF 有效估计恒定和缓变海流
- ESKF+MPC 在噪声下优于标准 MPC 和无海流估计的 MPC
- 仿真验证（ROV 平台）

## Assumptions
- 海流缓变（状态增广的准静态假设）
- 噪声为高斯分布
- 过程噪声和测量噪声协方差已知

## Limitations / Failure Modes
- EKF 线性化误差在强非线性区域增大
- 状态增广增加计算量（状态维数翻倍）
- 准静态海流假设在快速变化海流下失效
- 需要精确的噪声统计信息

## Reusable Ingredients
- ESKF 是 EG-UCCO 论文中 EKF 基线的方法参考
- 海流作为速度状态增广的思路（vs EG-UCCO 的 CFD 预测残差）
- ESKF+MPC 框架可与 UCCO+SMC 框架对比

## Open Questions
- ESKF vs EG-UCCO 在模型失配下的对比？
- 能否将激励门控融入 EKF 框架？

## Claims
_TODO._

## Connections
_Edges are recorded in `graph/edges.jsonl`._

## Relevance to This Project
**高度相关**。ESKF 是 EG-UCCO 论文中 EKF 基线的方法论基础。论文中 EKF-tuned 和 EKF-nom 对比组的设置参考了 ESKF 框架。关键差异：EKF 依赖噪声统计假设，EG-UCCO 利用 CFD 确定性先验 + 模型不确定性界。
