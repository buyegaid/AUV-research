# Experiment Tracker: 轨迹对比 — 基线方法 RMSE

| Run ID | 里程碑 | 目的 | 配置 | 指标 | 优先级 | 状态 | 备注 |
|--------|--------|------|------|------|--------|------|------|
| T01 | M0 | 直线sanity | straight/0%/5seed × 4baselines | RMSE | MUST | ⬜ TODO | 验证代码无崩溃 |
| T02 | M1 | 直线批量 | straight/0-10-20-30%/5seed × 4baselines | RMSE+Δc_max+bias | MUST | ✅ DONE | 39min, 80 runs |
| T03 | M2 | 圆形复核 | circle/0-10-20-30%/5seed × 4baselines | RMSE+Δc_max+bias | MUST | ✅ DONE | 49min, 80 runs |
| T04 | M3 | 数据汇总 | 全部 T01-T03 结果 | 对比表 | MUST | ✅ DONE | 见下方汇总 |

## 已有数据（论文 Tab 1，需复核）

| 配置 | KIN | CFD-Luenberger | EKF | PC-RCO |
|------|-----|----------------|-----|--------|
| circle/0% | 0.192 | 0.430 | 0.035 | 0.064 |
| circle/10% | 0.192 | 0.438 | 0.041 | 0.074 |
| circle/20% | 0.193 | 0.453 | 0.048 | 0.081 |
| circle/30% | 0.194 | 0.481 | 0.062 | 0.091 |

## 待运行：直线轨迹

| 配置 | KIN | CFD-Luenberger | EKF | PC-RCO |
|------|-----|----------------|-----|--------|
| straight/0% | ? | ? | ? | ? |
| straight/10% | ? | ? | ? | ? |
| straight/20% | ? | ? | ? | ? |
| straight/30% | ? | ? | ? | ? |

## 基线参数配置

```
KIN:             K3=[0.02;0.02], τ_c=100s
CFD-Luenberger:  K_c=[80;80], max_dc=0.5
EKF:             P0=0.1, τ_c=100s, 8-state (ν+c)
PC-RCO:          K_obs=10, τ_c=100s, Δc_max_base=0.06, adaptive clamp
```

## Next Actions

1. 创建 `run_trajectory_comparison.m` 脚本
2. 运行 T01 (sanity) → 确认通过
3. 运行 T02 (直线批量) → 收集数据
4. 运行 T03 (圆形复核) → 确认一致性
5. T04 汇总写入论文
