---
type: paper
node_id: paper:wei2021_distributed_lyapunovbased_model
title: "Distributed Lyapunov-Based Model Predictive Formation Tracking Control for Autonomous Underwater Vehicles Subject to Disturbances"
authors: ["Henglai Wei", "Chao Shen", "Yang Shi"]
year: 2021
venue: "IEEE Transactions on Systems, Man, and Cybernetics: Systems"
external_ids:
  arxiv: null
  doi: "10.1109/TSMC.2019.2946127"
  s2: null
tags: ["distributed-MPC", "formation-control", "ESO", "Lyapunov", "multi-AUV"]
added: 2026-05-26T07:21:28Z
---

# Distributed Lyapunov-Based Model Predictive Formation Tracking Control for Autonomous Underwater Vehicles Subject to Disturbances

## One-line thesis
ESO-based auxiliary controller combined with distributed MPC achieves AUV formation tracking robust to ocean current disturbances.

## Problem / Gap
多 AUV 编队控制中，海流扰动同时影响编队保持和轨迹跟踪。集中式 MPC 计算成本高且单点故障脆弱，分布式方案需保证稳定性。

## Method
- **分布式 Lyapunov 基模型预测控制（DLMPC）**：每个 AUV 独立求解局部 MPC，通过通信交换编队信息
- **ESO 辅助控制器**：在 MPC 优化问题中嵌入 ESO 估计的扰动补偿项
- **Lyapunov 约束**：MPC 优化中加入 Lyapunov 下降约束保证闭环稳定
- **3D 编队跟踪**：同时跟踪参考轨迹和维持编队构型

## Key Results
- 编队误差有界且收敛
- 分布式 MPC 计算量远低于集中式（复杂度 O(N) vs O(N²)）
- ESO 补偿有效抑制海流对编队的扰动
- 已获 133+ 引用，发表于 IEEE TSMC（顶刊，IF~11）

## Assumptions
- AUV 间通信无丢包、无延迟
- 编队构型预先定义且固定
- 海流对所有 AUV 相同（空间均匀）
- ESO 估计误差有界

## Limitations / Failure Modes
- ESO 仍为集总扰动框架
- 空间均匀海流假设在真实海洋中不成立
- 通信假设在水下声学环境中过于理想
- 未考虑避碰（collision avoidance）
- 编队构型固定，不适应动态任务需求

## Reusable Ingredients
- 分布式 MPC + ESO 的架构（可启发 EG-UCCO 的多 AUV 扩展）
- Lyapunov 约束保证 MPC 稳定性的方法
- 编队控制中的扰动处理框架

## Open Questions
- EG-UCCO 替换 ESO 后能否提升编队估计精度？
- 空间非均匀海流下的分布式估计？

## Claims
_TODO._

## Connections
_Edges are recorded in `graph/edges.jsonl`; summarize here for human readers._

## Relevance to This Project
**中等相关**。代表 ESO 在多 AUV 编队中的应用，EG-UCCO 的远期扩展方向（多 AUV 协同流场估计）。论文中作为 ESO 应用广度的佐证引用，说明 ESO 从单体控制到多体协同的延伸。
