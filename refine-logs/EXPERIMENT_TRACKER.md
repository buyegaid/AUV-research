# Experiment Tracker: EG-UCCO

| Run ID | Milestone | Purpose | System / Variant | Split | Metrics | Priority | Status | Notes |
|--------|-----------|---------|------------------|-------|---------|----------|--------|-------|
| R001 | M0 | sanity | ALL 4 observers | circle/0% | RMSE | MUST | ✅ PASSED | T=20s验证 |
| R002 | M0 | mismatch机制验证 | ALL 4 observers | circle/0%+30% | RMSE | MUST | ✅ PASSED | EKF退化77%确认失配生效 |
| R003 | M1 | 精度表batch | KIN/Børhaug/EKF/UCCO | 3场景×4失配×2seed | RMSE+MAE | MUST | ✅ DONE | K_obs=2.0, KIN未工作 |
| R004 | M1 | KIN诊断修复 | KIN only | circle | c_hat时程 | MUST | ✅ DONE | KIN=0.300=完全不收敛 |
| R005 | M1 | KIN重写+参数优化 | ALL 4 observers | circle/0%+30% | RMSE | MUST | ✅ DONE | KIN速率残差型,K3=0.02→0.192 |
| R006 | M1 | K_obs扫描 | UCCO | circle/0%+30% | RMSE | MUST | ✅ DONE | K_obs=6,8,10,12,16,20扫描 |
| R007 | M1 | 最终精度表batch | ALL 4 | 3×4×2 | RMSE | MUST | ✅ DONE | K_obs=10, UCCO 0.064 |
| R008 | M2 | 退化分析 | ALL 4 | 圆形4失配 | deg% | MUST | ✅ DONE | 从R007提取 |
| R009 | M3 | 门控消融 | UCCO variants | 3门控×3噪声×2seed | RMSE | MUST | ✅ DONE | 圆形Gramian≈NoGate; 高阈值=0.300 |
| R010 | M4 | Monte Carlo | ALL 4 | 2场景×2失配×10seed | Mean±Std | NICE | ✅ DONE | UCCO直线上方差大 |
| R011 | M4 | 填入论文+编译 | — | 中英文4节 | 编译 | MUST | ⏳ NEXT | 填入R007/R009/R010数据 |

## 当前参数配置

```
UCCO:   K_obs=10.0, max_dc=0.10, gate_mu=1e-8
Børhaug: K_c=[80;80], max_dc=0.5, eps_vel=1e-5
KIN:    K3=[0.02;0.02]
EKF:    P0=0.1, Q_nu=0.001, Q_c=0.0005, R0=0.01, use_bias=false
```

## Next Actions

1. **R009**: 门控消融 (gate vs no-gate vs high-threshold, 3噪声水平)
2. **R010**: Monte Carlo验证 (10 seeds)
3. **R011**: 填入论文数据 + xelatex编译

## 关键发现

- EKF精度最高(0.035)但模型失配下退化77%
- UCCO平衡精度(0.064)与鲁棒性(退化42%)
- Børhaug直线崩塌(2.284)完美验证激励门控必要性
- KIN免疫失配(退化<2%)但精度有限(0.192)
