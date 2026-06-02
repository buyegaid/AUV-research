---
type: paper
node_id: paper:zhao2024_pinn_auv_current
title: "Physics-Informed AUV Modeling Method and Application Under Current Disturbances"
authors: "Zhao, Yifeng; Hu, Zhiqiang; Geng, Lingbo; Zhang, Shaoze; Li, Xinmao"
year: 2024
venue: "2024 6th International Conference on Robotics and Computer Vision (ICRCV), IEEE"
doi: "10.1109/ICRCV62709.2024.10758602"
zotero_key: "8WVAIHMG"
added: 2026-05-26
---

# Physics-Informed AUV Modeling Method and Application Under Current Disturbances

## 核心贡献

用 PINN（物理信息神经网络）建立 AUV 静水动力学模型，通过比较静水模型输出与实际导航数据估计海流速度，再将估计的海流补偿到 MPC 预测模型中，实现海流环境下的轨迹跟踪。

## 方法

1. PINN 建模：将 AUV 动力学方程嵌入神经网络损失函数（物理约束项 + 数据拟合项）
2. 海流估计：`V_CE = R_EB^{-1} * (V_AUV - V_PINN_output)`
3. MPC 控制：基于修正后的 PINN 模型进行轨迹跟踪

## 与 Idea 1 的关键区别

| 维度 | Zhao 2024 (PINN+MPC) | Idea 1 (PI-ESO+SMC) |
|------|---------------------|---------------------|
| 物理信息嵌入方式 | PINN 损失函数中的物理约束 | Gauss-Markov 模型嵌入 ESO 状态方程 |
| 控制器 | MPC（采样优化） | SMC（解析控制律） |
| 海流模型 | 无显式时间动力学 | 一阶 Gauss-Markov 过程 |
| 计算复杂度 | 高（神经网络推理+MPC优化） | 低（线性ESO+SMC） |
| 实时性 | 受限于 MPPI 采样 | 适合嵌入式实时控制 |

## 局限性（可作为 Idea 1 的动机）

- 需要大量训练数据（20条轨迹×200s）
- PINN 推理计算量大，实时性受限
- 未利用海流的时间相关性（Gauss-Markov 结构）
- 未与 SMC 等鲁棒控制方法结合

## 引用

Zhao Y, Hu Z, Geng L, et al. Physics-Informed AUV Modeling Method and Application Under Current Disturbances[C]. ICRCV 2024, IEEE.
