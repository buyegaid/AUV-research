---
type: idea
node_id: idea:gauss_markov_eso_smc
title: "Gauss-Markov Ocean Current Model Embedded ESO for AUV Trajectory Tracking"
stage: completed
outcome: negative
based_on:
  - paper:ji2023_trajectory_tracking_auv
  - paper:kang2020_antidisturbance_control_auv
  - paper:zhao2024_pinn_auv_current
target_gaps:
  - gap:G1_physical_current_modeling
  - gap:G2_eso_smc_integration
added: 2026-05-26
novelty_score: 5
feasibility: high
---

# Gauss-Markov海流模型嵌入ESO用于AUV轨迹跟踪

## 核心假设

将可解释的海流载荷模型（Gauss-Markov动力学）显式嵌入扩展状态观测器（ESO）的状态方程中，可比集总扰动ESO更准确、更快速地估计复杂海流干扰。

## 方法概述

基于相对流速模型，将海流诱导的阻力、侧向力和偏航力矩表示为参数化物理项，构建"模型预测扰动 + ESO残差估计"的复合观测器：

```
ẋ = f(x,u) + d_current + d_residual
d_current = M * v_c  (海流诱导的力/力矩)
v̇_c = -Λv_c + σw  (Gauss-Markov动力学)
```

利用DVL/IMU/姿态/深度信息估计局部流速参数，控制端接入SMC进行补偿。

## 新颖性验证

**验证日期**: 2026-05-26
**验证方法**: 多源文献检索 + GPT-5.5交叉验证
**结论**: ✅ 严格意义上无等价工作

**最接近工作**:
- Zhao et al. 2024: PINN+MPC（数据驱动神经网络 vs 解析观测器）
- Yuan et al. 2022: GM用于仿真环境 vs GM嵌入ESO状态方程

**关键差异**: 将Gauss-Markov海流动力学显式嵌入ESO扩展状态转移方程，而非仅用于环境仿真或集总扰动估计。

## 预期结果

- 海流估计精度提升30-50%（相比LESO）
- SMC滑模增益减小，抖振减轻
- 复杂海流场景下跟踪误差降低

## 最小实验

XHY MATLAB仿真，4种海流场景（恒定流、GM缓变流、深度剪切流、流场切换），对比5种方法：
- SMC（基线）
- ISMC（积分滑模）
- LESO+SMC（集总扰动ESO）
- PI-ESO+SMC（物理信息ESO）
- PI-ESO+ASMC（物理信息ESO+自适应SMC）

## 风险

MEDIUM - Gauss-Markov参数选择影响结果，需敏感性分析。

## 目标期刊

IEEE Journal of Oceanic Engineering 或 Ocean Engineering

## 实施时间线

- 月1-2: 基础实现和初步验证
- 月3-4: 完整对比实验
- 月5-6: 复杂场景验证
- 月7-8: 论文撰写和投稿

## 相关想法

- [[idea_adaptive_boundary_smc]] - 自适应边界滑模（配套控制器）
- [[idea_current_field_switching]] - 流场切换鲁棒性（验证场景）
- [[idea_shear_flow_gradient_eso]] - 剪切流梯度ESO（扩展方向）

## 实施日志

### 2026-05-26: 基础实现完成

**完成项**:
- ✅ 创建 `eso/vec_pieso_update.m` - 物理信息ESO实现
- ✅ 关键创新: `Z3_dot = -Lambda * Z(:,3) + beta3 * f3` (嵌入Gauss-Markov衰减)
- ✅ 参数配置: `params.pieso.tau_c = 50s` (海流相关时间常数)
- ✅ 集成到 `xhy_simulator.m` (usePIESO开关)
- ✅ 单元测试通过: Lambda = 0.02, 自适应带宽 = 5.0
- ✅ 集成测试通过: 30s仿真正常运行

### 2026-05-27: 初步对比实验完成

**实验配置**:
- 仿真时间: 300s
- 海流: Vc=0.5m/s, 方向45°
- 轨迹: 300m半径圆形
- 对比方法: SMC / LESO+SMC / PI-ESO+SMC

**关键修复**:
- 修正`a_known`计算：使用绝对速度而非相对速度
- ESO估计值从0.1N增至1.3N（12倍提升）

**实验结果**:
| 方法 | 航向误差RMSE (rad) | 横向误差RMSE (m) | ESO估计均值 (N) |
|------|-------------------|-----------------|----------------|
| SMC | 0.395 | 173.1 | 0.0 |
| LESO | 0.403 (+2.0%) | 172.7 (-0.2%) | 1.26 |
| PI-ESO | 0.403 (+2.0%) | 172.7 (-0.2%) | 1.24 |

**发现问题**:
1. ⚠️ ESO方法在航向误差上反而增大2%
2. ⚠️ 横向误差仅改善0.2%（不显著）
3. ⚠️ LESO与PI-ESO性能几乎相同（1.26N vs 1.24N）
4. ⚠️ Gauss-Markov动力学未体现优势
5. ⚠️ 基础跟踪性能较差（173m误差 on 300m半径轨迹）

**可能原因**:
- ESO带宽参数需要调优
- 控制增益与ESO补偿不匹配
- 海流场景过于简单（恒定流）
- tau_c参数选择不当（50s可能过大）

**下一步**:
- [x] 调整ESO带宽参数（omega0_base, omega0_max）
- [x] 优化SMC控制增益
- [x] 测试时变海流场景（Gauss-Markov过程）
- [x] 调整tau_c参数（尝试多种值）
- [x] 分析ESO估计精度（与真实海流对比）

### 2026-05-27: 完整实验完成 — 结论为负面

**实验规模**: 5阶段实验，20种子统计验证，tau_c/omega0参数扫描

**最终数值结果**:
| 场景 | LESO均值 | PIESO均值 | 改善% |
|------|---------|----------|-------|
| 标准(omega0=3, tau_c=50, Vc=0.8m/s, 20种子) | 3.944m | 3.931m | 0.8% |
| 快速海流(tau_c=2s, 20种子) | 2.283m | 2.553m | -16.6% |
| 正常工况(Vc=0.3m/s, tau_c=2-50s, 20种子) | ~0.19m | ~0.19m | <2% |
| 阶跃海流消失(omega0=0.5-3.0) | — | — | -0.5~-1.1% |

**根本原因（理论验证）**:
- `omega0=3 >> 1/tau_c=0.02`，比值=150
- `beta3/Lambda = omega0^3 * tau_c = 27 * 50 = 1350`，Lambda项可忽略
- tau_c=0.5s时PIESO Z3欠估计58%，但SMC鲁棒性(Ks=3.0)掩盖差异
- 闭环控制下LESO的Z3也通过误差驱动自然衰减，无需显式Lambda项

**关键发现**:
1. PI-ESO与LESO在SMC+ESO前馈架构下等价（当omega0 >> 1/tau_c时）
2. 快速海流(tau_c=2-10s)导致系统不稳定率35-50%，与ESO类型无关
3. gauss_markov_current.m存在硬编码rng(42)导致多种子测试失效（已修复）

**研究价值重新定位**:
- 理论贡献：PI-ESO稳定性证明和参数设计准则（omega0与tau_c的匹配条件）
- 实用价值：防止Z3无界增长（积分器饱和问题），在ESO带宽受限场景下有优势
- 建议方向：理论分析 > 仿真验证（仿真无法体现优势）
