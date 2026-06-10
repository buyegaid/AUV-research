---
type: paper
node_id: paper:zhao2024_pinn_auv_current
title: "Physics-Informed AUV Modeling Method and Application Under Current Disturbances"
authors: ["Zhao et al."]
year: 2024
venue: "Ocean Engineering"
external_ids:
  arxiv: null
  doi: null
  s2: null
tags: ["PINN", "MPC", "current-modeling", "data-driven"]
added: 2026-06-09T00:00:00Z
---

# Physics-Informed AUV Modeling Method and Application Under Current Disturbances

## One-line thesis
Physics-informed neural network (PINN) combined with MPC achieves AUV control under current disturbances, representing the data-driven alternative to analytical current observers.

## Problem / Gap
AUV 水动力模型难以同时捕获海流诱导力和参数不确定性，传统 CFD 标定精度高但工况覆盖有限。

## Method
- **PINN 物理信息神经网络**：将 Navier-Stokes 方程作为损失函数的正则化项，训练神经网络近似 AUV 动力学和海流效应
- **模型预测控制（MPC）**：基于 PINN 的预测模型进行滚动优化
- 数据 + 物理双驱动

## Key Results
- PINN 模型在有数据区域精度高
- MPC 在 PINN 模型基础上实现约束满足

## Assumptions
- 需要大量训练数据（CFD 仿真或实验）
- 训练后模型固定，不进行在线更新

## Limitations / Failure Modes
- PINN 训练计算量大（GPU 小时级）
- 泛化到训练域外场景可能失败
- 黑箱模型，无法分离海流和模型误差
- 仅离线使用，不能在线适应

## Reusable Ingredients
- PINN 可作为基准验证 CFD 物理先验的替代价值
- MPC 框架可对比 SMC 的鲁棒性

## Open Questions
- PINN vs CFD 解析模型在观测器中的信息价值对比？

## Claims
_TODO._

## Connections
_Edges are recorded in `graph/edges.jsonl`._

## Relevance to This Project
**高度相关**。Zhao 2024 是 PI-ESO（负面结果）最接近的对比工作。关键差异：EG-UCCO 使用 CFD 解析先验 + 在线更新，PINN+MPC 使用数据驱动黑箱 + 离线训练。在 EG-UCCO 论文讨论中体现"CFD 物理先验 vs 数据驱动先验"的对比。
