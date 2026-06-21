# Research Proposal: PC-RCO — Prediction-Correction Robust Current Observer

## Problem Anchor

- **Bottom-line problem**: 任何水动力模型（CFD或水池辨识）与真实AUV之间必然存在误差。现有模型基海流观测器在模型-实物失配下精度退化或发散，而运动学观测器收敛太慢。
- **Must-solve bottleneck**: 如何利用动力学模型的信息优势，同时不被模型误差污染海流估计？
- **Non-goals**: 不是追求最高的绝对精度（EKF可做到），不是替换CFD方法学，不是设计新的水动力模型
- **Constraints**: XHY AUV MATLAB仿真平台，5推进器，水池校准后的动力学参数
- **Success condition**: 在30%模型失配下，PC-RCO的退化幅度显著低于EKF（<50% vs >70%），且在低激励场景不崩塌

## Technical Gap

现有海流观测器在面对模型-实物失配时呈现两极分化：
1. **运动学方法(KIN)**: 完全不用模型 → 免疫失配 → 但精度受限(0.19 m/s)且收敛慢
2. **概率方法(EKF)**: 用模型+协方差 → 精度最高(0.035) → 但失配下退化77%
3. **朴素模型基方法(Børhaug类)**: 用模型但无保护 → 低激励下崩塌(2.28)

**缺失的机制**: 一个既能利用模型信息、又能在模型失配下自我保护的估计架构。

## Method Thesis

- **One-sentence thesis**: 三个轻量级保护机制——速度层预测校正、Gauss-Markov时间衰减、单步限幅——联合作用，使模型基海流观测器在模型失配下保持鲁棒，无需激励门控或协方差调参。
- **Why this is the smallest adequate intervention**: 三个机制都不引入新状态或新参数族，仅对更新过程施加结构性约束

## Contribution Focus

- **Dominant contribution**: 预测校正+GM衰减+限幅三机制的联合鲁棒性——证明这三个简单机制足以防止模型基观测器发散，且失配退化显著低于EKF
- **Optional supporting contribution**: Gramian灵敏度分析作为可观测性诊断工具（非性能开关）
- **Explicit non-contributions**: 不是CFD方法，不是新水动力模型，不是激励门控（已证明无效）

## Proposed Method

### Complexity Budget
- 无新增状态变量（EKF需要8-14个状态，PC-RCO仅2个海流状态）
- 无协方差矩阵维护
- 无在线参数辨识
- 三个保护机制都是结构性的（架构+衰减+限幅），不需要额外调参

### System Overview
```
ν_meas(DVL) → [速度预测: ν̂_pred = ν_prev + dt·a_model(ν_prev,τ,ĉ)]
            → [新息: e = ν_meas(1:2) - ν̂_pred(1:2)]
            → [梯度更新: Δĉ = γ·Φ_c^T·e / (1+λ_min·κ)]  ← Gramian诊断
            → [限幅: |Δĉ| ≤ Δc_max]
            → [GM衰减: ĉ ← α·ĉ + (1-α)·c̄]
            → ĉ → [Yaw前馈补偿]
```

### Core Mechanism — 三保护机制
1. **速度层预测校正**: 在速度层做一步预测（ν̂_pred），新息天然O(10⁻⁴ m/s)，避免加速度差分噪声放大
2. **GM时间衰减**: τ_c=100s的慢衰减持续将估计拉向先验均值，防止漂移
3. **单步限幅**: Δc_max=0.1 m/s限制任何单次更新的最大影响
4. **Gramian诊断** (辅助): λ_min(W_c)量化可辨识性，用于自适应正则化

### Novelty Argument
- vs EKF: 不需要协方差调参，状态维度低(2 vs 8)，失配退化小(42% vs 77%)
- vs Børhaug类: 有三保护机制，不会崩塌
- vs KIN: 利用了模型信息，精度更高(0.064 vs 0.192)

## Experiment Summary (已完成)

| 场景 | 失配 | KIN | CFD-Luenberger | EKF-nom | PC-RCO |
|------|------|-----|----------------|---------|--------|
| 圆形 | 0% | 0.192 | 0.430 | 0.035 | 0.064 |
| 圆形 | 30% | 0.194 | 0.481 | 0.062 | 0.091 |
| 直线 | 0% | 0.186 | 2.284 | 0.038 | 0.043 |
| 阶跃 | 0% | 0.237 | 0.230 | 0.107 | 0.138 |

**关键发现**:
- PC-RCO失配退化42% vs EKF 77%
- CFD-Luenberger直线崩塌2.28 vs PC-RCO稳定0.043
- Gate消融: 硬门控在所有场景无效(<2%差异) → 三保护机制才是真正的鲁棒性来源
