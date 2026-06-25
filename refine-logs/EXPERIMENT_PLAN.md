# Experiment Plan: 轨迹对比 — 基线方法 RMSE 评估

**问题**: 不同轨迹激励条件下，各基线方法的精度-鲁棒性表现如何？
**目标**: 圆形（持续激励）vs 直线（低激励）轨迹下，4种基线 × 4种失配水平的系统 RMSE 对比
**日期**: 2026-06-23

---

## Claim Map

| Claim | 为何重要 | 最小令人信服的证据 | 关联块 |
|-------|---------|-------------------|--------|
| **C1** PC-RCO在低激励直线轨迹不崩塌，而朴素模型基方法崩塌 | 证明连续GM正则化在弱激励下的保护作用 | 直线RMSE: CFD-Luenberger >> PC-RCO ≈ EKF | B1, B2 |
| **C2** EKF在直线轨迹精度最高但失配退化显著 | 确认EKF精度-鲁棒性权衡在低激励下同样存在 | 直线30%失配下EKF退化 >> PC-RCO退化 | B2 |
| **C3** 轨迹激励水平是区分方法鲁棒性的关键因子 | 为论文增加一个表格维度的对比维度 | 圆形 vs 直线对比表 | B1, B2 |

---

## 实验矩阵

### 实验因子

| 因子 | 水平 | 说明 |
|------|------|------|
| 轨迹类型 | 圆形, 直线 | 圆形=持续激励, 直线=低激励 |
| 基线方法 | KIN, CFD-Luenberger, EKF, PC-RCO | 4种 |
| 模型失配 | 0%, 10%, 20%, 30% | 4水平 |
| 噪声 | low (σ=0.02) | 基线噪声水平 |
| Seeds | 5 | 统计稳定性 |
| 海流 | Vc=0.3 m/s, β=45° 恒流 | 统一条件 |

**总计**: 2轨迹 × 4方法 × 4失配 × 5 seeds = 160 次仿真运行

### 已有数据

| 轨迹 | 失配 | KIN | CFD-Luenberger | EKF | PC-RCO | 来源 |
|------|------|-----|----------------|-----|--------|------|
| 圆形 | 0% | 0.192 | 0.430 | 0.035 | 0.064 | 论文 Tab 1 |
| 圆形 | 10% | 0.192 | 0.438 | 0.041 | 0.074 | 论文 Tab 1 |
| 圆形 | 20% | 0.193 | 0.453 | 0.048 | 0.081 | 论文 Tab 1 |
| 圆形 | 30% | 0.194 | 0.481 | 0.062 | 0.091 | 论文 Tab 1 |
| 直线 | 0% | 0.186 | 2.284 | 0.038 | 0.043 | proposal (需重新验证) |
| 直线 | 10% | ❌ | ❌ | ❌ | ❌ | 需运行 |
| 直线 | 20% | ❌ | ❌ | ❌ | ❌ | 需运行 |
| 直线 | 30% | ❌ | ❌ | ❌ | ❌ | 需运行 |

---

## Experiment Blocks

### Block 1: 圆形轨迹基线对比（已有，复核）

- **Claim tested**: C1, C2 — 持续激励下各方法表现
- **Status**: ✅ 已完成 — 论文 Tab 1 数据
- **Action**: 仅需用当前代码重新运行确认数据一致性
- **Compared systems**: KIN, CFD-Luenberger, EKF, PC-RCO
- **Metrics**: RMSE_Vc (主要), max Δc, final bias
- **Priority**: 复核

### Block 2: 直线轨迹基线对比（新运行）

- **Claim tested**: C1, C2, C3 — 低激励下各方法表现
- **Why this block exists**: 论文当前主表仅有圆形数据，缺少轨迹维度的系统对比。直线轨迹是区分方法鲁棒性最关键的场景：CFD-Luenberger崩塌(2.284) vs PC-RCO保持稳定(0.043)
- **Setup**: 直线轨迹，恒流 Vc=0.3 m/s, β=45°, T=100s, u_d=1 m/s
- **Mismatch levels**: 0%, 10%, 20%, 30%
- **Noise**: low (σ=0.02 m/s)
- **Seeds**: 5
- **Compared systems**: KIN, CFD-Luenberger, EKF, PC-RCO
- **Metrics**: RMSE_Vc, max Δc, final bias, overshoot
- **Success criterion**: 直线下 PC-RCO << CFD-Luenberger (至少 10× 差距)，PC-RCO 退化 < 50% at 30% mismatch
- **Failure interpretation**: 如果 PC-RCO 在直线下也崩塌 → 连续 GM 正则化无效 → 需重新审查 τ_c 参数
- **Table target**: 新论文表格 — 圆形 vs 直线对比表

### Block 3: 轨迹对比综合表（数据汇总）

- **Claim tested**: C3 — 轨迹激励水平是关键区分因子
- **Data source**: B1 + B2
- **Output**: 2×4×4 矩阵表，行=轨迹，列=方法×失配
- **Table target**: 论文新增 `tab:trajectory_comparison`

---

## 论文目标表格

```
Table X: RMSE_Vc across trajectory types and mismatch levels (5 seeds, σ=0.02)

圆形轨迹 (持续激励):
Mismatch | KIN    | CFD-Luen | EKF    | PC-RCO
0%       | 0.192  | 0.430    | 0.035  | 0.064
10%      | 0.192  | 0.438    | 0.041  | 0.074
20%      | 0.193  | 0.453    | 0.048  | 0.081
30%      | 0.194  | 0.481    | 0.062  | 0.091

直线轨迹 (低激励):
Mismatch | KIN    | CFD-Luen | EKF    | PC-RCO
0%       | TBD    | TBD      | TBD    | TBD
10%      | TBD    | TBD      | TBD    | TBD
20%      | TBD    | TBD      | TBD    | TBD
30%      | TBD    | TBD      | TBD    | TBD
```

---

## Run Order

| 里程碑 | 目标 | 运行内容 | 决策门 | 预计时间 |
|--------|------|---------|--------|---------|
| M0 | 代码验证 | 直线轨迹 1 seed × 4方法, 确认无崩溃 | 所有方法运行正常 | 5 min |
| M1 | 直线批量 | 直线 × 4失配 × 4方法 × 5 seeds = 80 runs | 结果趋势合理 (CFD-Luen 崩塌, PC-RCO 稳定) | 20 min |
| M2 | 圆形复核 | 圆形 × 4失配 × 4方法 × 5 seeds = 80 runs | 与论文 Tab 1 一致 | 20 min |
| M3 | 数据汇总 | 生成对比表 + 写入论文 | 对比表完成 | 10 min |

---

## 实现方案

### 方案: 扩展 `run_ablation_v2.m`

当前 `run_ablation_v2.m` 已支持:
- Scenario `B1`-`B4`，基于 `'circle'` 和 `'intermittent'` 两种 `traj_type`
- PC-RCO 变体: PCRCO, NoGM, NoClamp, NoPredCorr
- 需要新增: (1) `'straight'` 轨迹类型; (2) 完整基线方法 (KIN, CFD-Luenberger, EKF); (3) 失配扫描

### 新建 `run_trajectory_comparison.m`

统一脚本，一次运行所有配置:
```
参数空间: traj ∈ {circle, straight} × baseline ∈ {KIN, CFD-Luenberger, EKF, PC-RCO} × mismatch ∈ {0,10,20,30}%
噪声: low (σ=0.02)
海流: constant Vc=0.3, β=45°
Seeds: 5
T: 100s, dt: 0.01
```

输出: `results/trajectory_comparison.mat` + 汇总表

---

## 风险与缓解

| 风险 | 缓解 |
|------|------|
| 直线轨迹下 EKF 协方差发散 | 限定 mismatch 上限 30%, 监控 P 矩阵 |
| 直线下 ALOS 制导产生小幅度转弯 → 名义直线非纯直线 | 确认这个影响不大——反馈控制下纯直线不存在，正是论文论点 |
| 直线 100s 下 KIN 未充分收敛 | 论文已说明 KIN 收敛慢，可加注释 |
| 重复运行与论文 Tab 1 的不完全一致 | 固定 seed，记录在结果文件中 |

## Nice-to-Have (暂不运行)

- 直线 × high noise (σ=0.10) — 额外压力测试
- 直线 × 40%/50% mismatch — 极端失配
- zigzag / lawnmower 轨迹 — 扩展轨迹库
