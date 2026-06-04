# 论文规划：基于CFD先验的激励门控海流观测器

**标题**: Excitation-Gated CFD-Prior Current Observer for AUVs: Robust Estimation under Hydrodynamic Model Mismatch
**中文标题**: 基于CFD先验的激励门控海流观测器：水动力模型失配下的鲁棒估计
**目标期刊**: Ocean Engineering (IF~5.0)
**语言**: 中文
**日期**: 2026-06-04

---

## 一、论文结构

### 1. 引言 (Introduction)
**篇幅**: ~2页

**内容**:
- AUV轨迹跟踪中海流估计的重要性
- 现有方法的两类范式：
  - 运动学观测器 (Liang 2018): 收敛慢，需要持续位置误差
  - 集总扰动观测器 (ESO): 响应快，但不能分离海流与模型不确定性
- 核心问题: 水动力模型不确定性普遍存在（CFD误差、制造公差、未建模耦合），现有方法缺乏对模型失配的鲁棒性
- 本文贡献:
  1. 提出EG-UCCO：利用CFD预标定动力学模型作为确定性先验
  2. 灵敏度Gramian激励门控：Fisher信息驱动的自适应更新
  3. 系统验证：3场景×4失配水平下的鲁棒性优势
  4. 门控消融：Gramian门控在高噪声下提升23%

**关键引用**: Børhaug 2007, Kim 2018, Liang 2018, He 2024

### 2. 问题描述 (Problem Formulation)
**篇幅**: ~1.5页

**内容**:
- XHY AUV 6-DOF动力学模型
- 海流的参数化形式: ν_c = [Vc·cos(βc-ψ), Vc·sin(βc-ψ), w_c, 0, 0, 0]^T
- CFD标定阻力模型 (R² > 0.99)
- 惯性系海流分量 c = [cN, cE]^T
- 模型失配的有界表征
- 传感器配置: INS+DVL+深度计

**公式**:
- 动力学方程 Mν̇ + C(νr)νr + D(νr)νr + g = τ
- 相对速度 νr = ν - νc
- 模型不确定性界 Δ

### 3. EG-UCCO观测器设计 (Observer Design)
**篇幅**: ~3页

**内容**:

**3.1 速度预测架构**
- 一步前向速度预测: ν̂_pred = ν_prev + dt·a_model(ĉ)
- 速度新息: e_vel = ν_meas - ν̂_pred
- 与加速度残差方案的对比（SNR分析）

**3.2 灵敏度分析**
- 灵敏度矩阵 Φ_c = ∂(ν̂_pred)/∂c（数值扰动CFD模型）
- 灵敏度Gramian: W_c = Φ_c' Φ_c
- Fisher信息解释

**3.3 激励门控**
- 门控条件: λ_min(W_c) > μ_gate
- 可辨识性分析: 稳态直航时W_c奇异
- 门控将不可辨识性转化为自动保护机制

**3.4 梯度更新律**
- ĉ += γ·Φ_c'·e_vel（自适应步长）
- GM时间传播（无激励时）
- 稳定性分析（Lyapunov UUB）

**3.5 前馈补偿**
- 补偿消融实验: surge补偿损害航向（surge-yaw耦合）
- 仅yaw前馈策略

**公式**:
- Φ_c 数值计算
- Gramian特征值条件
- Lyapunov候选函数

### 4. 仿真验证 (Simulation Validation)
**篇幅**: ~3页

**4.1 实验设置**
- XHY AUV MATLAB仿真平台
- 7种对比方法: SMC, KIN, LESO, PIESO, EKF-nom, EKF-tuned, UCCO
- 3种海流场景: 圆形(持续机动)、直线(低激励)、海流阶跃(突变)
- 4种模型失配水平: 0%, 10%, 20%, 30% CFD阻力系数扰动
- 评估指标: RMSE_Vc, MAE_Vc, 收敛时间

**4.2 海流估计精度（无噪声）**
- 表格: 3场景×4失配的RMSE_Vc
- 关键发现: UCCO在12/12配置中估计精度最优
- 平均比KIN/EKF提升42%

**4.3 模型失配鲁棒性**
- 退化曲线图: RMSE_Vc vs 失配水平
- EKF从0.27退化到0.29 (+8%)，UCCO从0.14退化到0.15 (+7%)
- EKF未调优时退化694%，UCCO仅3%

**4.4 门控消融实验**
- 表格: 3种门控×3种噪声水平
- Gramian门控在高噪声下比无门控提升23.3%
- 高阈值门控完全失效（excite=0%）

**4.5 传感器噪声鲁棒性**
- Monte Carlo结果（10 seeds）
- 噪声下UCCO优势收窄但鲁棒性规律保持
- 模型失配20%时UCCO仍优于EKF 5-8%

### 5. 讨论 (Discussion)
**篇幅**: ~1页

- UCCO的优势场景: 模型失配存在时
- UCCO的局限: 传感器噪声下优势收窄
- 与ESO方法的互补性: ESO用于控制补偿，UCCO用于物理海流感知
- CFD先验的价值: 预标定 vs 在线辨识
- 实际部署考虑: 计算复杂度、DVL失效、参数调节

### 6. 结论 (Conclusion)
**篇幅**: ~0.5页

- 主要贡献总结
- 未来工作: 海试验证、多AUV协同估计、流场建图

---

## 二、图表计划

| 编号 | 类型 | 内容 | 数据来源 |
|------|------|------|---------|
| Fig.1 | 框图 | EG-UCCO架构图 | 手动/figurespec |
| Fig.2 | 示意图 | XHY AUV + 传感器配置 | 手动 |
| Fig.3 | 曲线图 | 灵敏度Gramian特征值 vs 时间（机动vs直航） | gate_ablation.m |
| Fig.4 | 表格 | 海流估计精度（3场景×4失配） | full_experiment.m |
| Fig.5 | 曲线图 | 模型失配退化曲线（5方法×4水平） | mismatch_sweep.m |
| Fig.6 | 表格 | 门控消融（3门控×3噪声） | gate_ablation.m |
| Fig.7 | 曲线图 | Monte Carlo误差棒图 | monte_carlo.m |
| Fig.8 | 曲线图 | 海流估计时程（阶跃响应） | full_experiment.m |

---

## 三、引用计划

| 论文 | 引用位置 | 用途 |
|------|---------|------|
| Børhaug et al. (2007) | 引言 | 动力学海流观测器先驱 |
| Kim et al. (2018) | 引言+讨论 | HGO+在线辨识基线 |
| Liang et al. (2018) | 引言+方法 | 运动学海流观测器基线 |
| He et al. (2024) | 引言 | 运动学+NDO双层架构 |
| Fossen (2011) | 问题描述 | 海洋器动力学手册 |
| Aguiar & Pascoal (2007) | 引言 | 指数海流观测器 |

---

## 四、Claims-Evidence矩阵

| Claim | Evidence | 章节 |
|-------|----------|------|
| CFD先验提升估计精度 | UCCO 42%优于运动学/KIN基线 | 4.2 |
| 模型失配鲁棒性 | 30%失配下UCCO退化<8%, EKF退化16% | 4.3 |
| 激励门控有效 | 高噪声下gate vs no-gate提升23% | 4.4 |
| 多场景泛化 | 圆形/直线/阶跃12/12全胜 | 4.2 |
| 补偿消融 | surge补偿损害航向，yaw-only最优 | 3.5 |
| 噪声鲁棒性 | Monte Carlo统计验证(10 seeds) | 4.5 |

---

## 五、论文提纲（LaTeX章节）

```
\section{引言}
\section{问题描述}
  \subsection{AUV动力学模型}
  \subsection{海流参数化与CFD先验}
  \subsection{模型不确定性表征}
\section{EG-UCCO观测器设计}
  \subsection{速度预测架构}
  \subsection{灵敏度分析与激励门控}
  \subsection{梯度更新律}
  \subsection{前馈补偿策略}
\section{仿真验证}
  \subsection{实验设置}
  \subsection{海流估计精度}
  \subsection{模型失配鲁棒性}
  \subsection{门控消融实验}
  \subsection{传感器噪声鲁棒性}
\section{讨论}
\section{结论}
```
