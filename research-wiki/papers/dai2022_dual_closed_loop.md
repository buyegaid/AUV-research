---
type: paper
node_id: paper:dai2022_dual_closed_loop
title: "Dual closed loop AUV trajectory tracking control based on finite time and state observer"
authors: ["Xiaoqiang Dai", "Hewei Xu", "Hong-xu Ma", "Jianjun Ding", "Qiang Lai"]
year: 2022
venue: "Mathematical Biosciences and Engineering"
external_ids:
  arxiv: null
  doi: "10.3934/mbe.2022517"
  s2: null
tags: ["reduced-order-ESO", "finite-time-control", "dual-closed-loop", "lake-trial"]
added: 2026-05-26T07:21:29Z
---

# Dual closed loop AUV trajectory tracking control based on finite time and state observer

## One-line thesis
Reduced-order ESO combined with finite-time controller and auxiliary saturation system achieves robust 3D AUV trajectory tracking, validated through lake trials.

## Problem / Gap
AUV 在 3D 轨迹跟踪中面临三个问题：(1) 控制器收敛速度慢；(2) 执行器输出饱和；(3) 模型不确定性和外部扰动。需要同时解决这三个问题的统一框架。

## Method
- **降阶扩展状态观测器（Reduced-order ESO）**：估计模型不确定性和外部扰动，维度低于全阶 ESO 降低计算量
- **有限时间位置控制器**：加速位置环收敛
- **滤波积分滑模姿态控制器**：在 Serret-Frenet 坐标系下基于"虚拟导引"建立 3D 误差模型，抑制抖振
- **辅助动态系统**：补偿执行器饱和，防止积分器 windup
- **双闭环架构**：外环位置 + 内环姿态

## Key Results
- **海探 II 号 AUV 苏州湖试验证**
- 俯仰角/航向角误差均值 < 8°
- 深度误差均值 < 0.1 m
- 降阶 ESO 有效估计扰动，辅助动态系统成功缓解饱和

## Assumptions
- 扰动变化速率有界
- AUV 速度可测（DVL + IMU）
- 欠驱动配置（舵面控制）

## Limitations / Failure Modes
- 降阶 ESO 仍为集总扰动框架，不区分海流和模型误差
- 湖试条件温和（弱流），海试海流场景未验证
- 有限时间收敛对初始误差敏感

## Reusable Ingredients
- 降阶 ESO 结构可适配 XHY 的 6-DOF 模型
- 辅助动态系统抗饱和机制通用性强
- 双闭环架构清晰，易于模块化替换

## Open Questions
- 降阶 ESO 在强海流下的估计精度？
- 有限时间控制 vs SMC 的对比？

## Claims
_TODO._

## Connections
_Edges are recorded in `graph/edges.jsonl`; summarize here for human readers._

## Relevance to This Project
**高度相关**。降阶 ESO 是 EG-UCCO 的重要基线方法之一。双闭环架构中的扰动观测器可替换为 EG-UCCO 形成改进方案。湖试验证增加了方法的可信度，EG-UCCO 论文可引用其实验范式。
