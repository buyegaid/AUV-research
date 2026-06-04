# Codex GPT-5.5 xhigh 外部审阅记录

**日期**: 2026-06-04
**审阅模型**: gpt-5.5 (xhigh reasoning) via Codex MCP
**审阅标准**: Ocean Engineering 正刊投稿标准
**Pipeline 阶段**: Phase 4 — 外部批判性审阅

---

## 审阅结论摘要

| 维度 | 评分 | 评语 |
|------|:---:|------|
| 创新性 | 5/10 | CFD prior + gated adaptive observer 有一定组合价值，但各组件不新 |
| 理论严谨性 | 3/10 | 核心可辨识性和分离性没有证明 |
| 实验说服力 | 2/10 | simulation-only + 传感器理想化 |
| 工程意义 | 6/10 | XHY 有高质量 CFD 和池试数据，方向有潜力 |
| **综合** | **4/10** | **Reject** |
| 若弱化 claim + 强 baseline + 模型失配验证 | — | Major Revision |
| 若再加水池/海试数据 | — | 有机会 Accept |

---

## 四个致命缺陷

### F1: Deadzone ≠ Separation
- Deadzone 只是启发式归因（"误差大于阈值才更新"），不是可辨识分离
- 大的模型误差、推进器误差、耦合水动力误差都可能越过 deadzone
- 小海流也可能落在 deadzone 内
- Claim 2 "explicit separation" 是当前最危险的表述

### F2: CFD R² 被过度解释
- 单自由度 R² > 0.99 ≠ 6-DOF 耦合模型准确
- 没有 coupled sway-yaw、heave-pitch、大漂角等耦合工况的 CFD 验证
- 耦合模型误差完全未知

### F3: 双模型残差 ≠ 海流特征
- `ν̂_nom - ν̂_obs` 混有 observer gain 注入、初值误差、模型误差、推进器误差、数值积分误差
- 海流和阻尼系数误差都通过相对流速进入阻力项，结构上高度共线
- 需要 sensitivity matrix 分析来证明残差到海流的映射

### F4: Inverse Crime
- 仿真若使用同一个 CFD 模型生成 plant 和 observer，算法只是反演自己写进去的模型
- 必须让 plant 和 estimator model 有结构性不一致

---

## 精化建议

### 1. 弱化 Claim，重构论文故事

**旧核心 claim**: "首次显式分离海流和模型误差"
**新核心 claim**: "在有界 CFD 模型误差下，利用激励条件和灵敏度分析实现鲁棒海流估计，并输出保守置信界"

建议论文定位 **C + B**（工程方法论 + 理论保证）：
> A CFD-calibrated, uncertainty-aware current observer for AUVs, with excitation-conditioned update and bounded-error confidence under model mismatch.

### 2. 方法改进

#### a. 状态表示：Vc,βc → cN,cE
- 用惯性系水平流速分量 `c = [cN, cE]^T` 替代 Vc,βc
- 避免 Vc≈0 时方向角奇异性
- 避免角度 wrap 导致的不连续

#### b. 激励门控：启发式 → Fisher Information / Sensitivity Gramian
- 用滑动窗口内灵敏度矩阵 Φc 的 Gramian:
  ```
  Wc = Σ Φc(t)^T R^(-1) Φc(t) dt
  ```
- 只有 `λmin(Wc) > μ` 时才允许 dynamics update
- 从"传感器读数经验判断"提升为"基于模型的可辨识性数学条件"

#### c. Deadzone → Set-Membership Update
- 估计不是点估计，而是一个可行集合:
  ```
  Ck = {c : ||r - Φc·c|| ≤ Δ_model + Δ_noise}
  ```
- 集合缩小时更新，不缩小时只用 GM prior
- 将 deadzone 从核心创新降级为保护项

#### d. 多假设检验
- Hc: 残差由惯性系海流解释
- Hm: 残差由艇体系模型偏差解释
- 只有 Hc likelihood 显著优于 Hm 时才更新

#### e. 不确定性分量化
- 不用全局标量 deadzone，改为状态相关的分量化界:
  ```
  |δτ_i| ≤ Δ_single-DOF,i + Δ_coupling,i + Δ_thruster,i + Δ_sensor,i
  ```

### 3. 强制 Baseline: CFD-Augmented EKF

必须实现三个版本:

**EKF-1**: `x = [η(6), ν(6), cN, cE]` — 仅增广海流
**EKF-2**: `x = [η(6), ν(6), cN, cE, bτ(6)]` — 增广海流 + 慢变 model bias
**UKF-3**: 若 CFD dynamics 强非线性

过程模型必须用和 EG-DMAO 同等级的 CFD 6-DOF 模型。

### 4. Inverse Crime 避免：Plant-Estimator Mismatch

至少四类扰动族：
| 扰动族 | 范围 | 目的 |
|--------|------|------|
| 阻尼系数 | ±5%, ±10%, ±15%, ±25% | CFD/池试外推误差 |
| 附加质量/惯量 | ±5%, ±10% | 动力学惯性误差 |
| 未建模耦合 | sway-yaw, heave-pitch, surge-yaw | 打破单自由度模型假设 |
| 推进器模型 | 增益 ±5-15%, 滞后, 饱和 | 真实执行器 |
| 传感器 | DVL噪声/dropout, INS bias, depth noise | 实际测量链 |

实验设计：
1. Nominal sanity: plant=estimator
2. Single-family stress: 每次只扰动一类
3. Combined Monte Carlo: 随机组合，至少 100 次
4. **零流池试 replay**: 验证不误报海流（最有价值利用现有数据）

### 5. 最小可行改进
如果只做一件事：**把 deadzone 改成"灵敏度 Gramian + 有界误差 set-membership update"，并加入 current+bias CFD-EKF baseline。**

### 6. 方法更名
EG-DMAO → **EG-UCCO** (Excitation-Gated Uncertainty-Calibrated Current Observer)

---

## 新增参考文献
1. https://doi.org/10.1016/j.oceaneng.2014.09.013 — AUV EKF/UKF 水动力参数估计
2. https://doi.org/10.1016/j.oceaneng.2019.04.039 — 鲁棒 EKF/H∞ 参数估计
3. https://doi.org/10.1016/j.oceaneng.2024.116847 — current-aware dynamic model
4. https://doi.org/10.5772/60415 — model-aided sea current estimation
5. https://doi.org/10.1016/S1474-6670(17)32090-5 — 经典 6-DOF model-based current observer (Børhaug)
