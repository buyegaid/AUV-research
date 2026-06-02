---
type: idea
node_id: idea:coupled_multidof_eso_smc
title: "Coupled Multi-DOF ESO-SMC for Underactuated AUV"
stage: proposed
outcome: unknown
based_on:
  - paper:dai2022_dual_closed_loop
  - paper:wang2024_accurate_trajectory_tracking
target_gaps:
  - gap:G5_multidof_coupled_eso
added: 2026-05-26
---

# 欠驱动 AUV 多自由度耦合 ESO-SMC

## 假设
针对纵向-偏航-俯仰耦合设计的 MIMO ESO 比独立通道 ESO 在欠驱动 AUV 轨迹跟踪中表现更好。

## 方法
- 设计 MIMO ESO 同时估计 surge/yaw/pitch 通道的耦合扰动
- 与 XHY 5 推进器推力分配模块结合
- 对比解耦 ESO+SMC 和耦合 MIMO ESO+SMC

## 预期结果
耦合 ESO 在横向海流下偏航-纵向耦合误差减小 20-40%。

## 最小实验
XHY MATLAB 仿真，3D 螺旋/割草机/变深路径，包含推力分配饱和。

## 风险
MEDIUM-HIGH — 耦合观测器稳定性和调参难度较高。

## 目标期刊
IEEE Transactions on Control Systems Technology
