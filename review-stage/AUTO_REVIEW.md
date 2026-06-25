# Auto Review: PC-RCO 消融实验 V2

**开始**: 2026-06-22
**方法**: PC-RCO (Prediction-Correction Robust Current Observer) 海流观测器
**审查后端**: Codex GPT-5.5 xhigh
**难度**: medium

---

## Round 1 (2026-06-22)

### Assessment
- **Score**: 4/10
- **Verdict**: not ready
- **Reviewer**: Codex GPT-5.5 xhigh

### Key Criticisms
1. GM decay机制未被验证 — gate永远100%打开
2. excitation gate设计失败 — gate_mu与实际λ_min差300-600倍
3. Prediction-correction主张不稳 — B3中NoPredCorr反而更好
4. RMSE不是足够指标 — B3证明RMSE会奖励物理不可接受的行为
5. Clamp物理解释需收紧 — 不是严格海流物理模型
6. Full在B3的final bias=0.935极差 — 对0.3m/s海流来说不可接受

### Reviewer Raw Response
Score: 4/10. Verdict: 暂不建议投稿。目前结果已足够说明原始ablation有严重问题，也验证了clamp的必要性。但以"PC-RCO三机制均有效"作为论文主张站不住。更像是一篇有潜力的算法调试后研究。

主要弱点：GM decay未被验证（gate=100%）、gate设计失败（λ_min永超阈值）、Prediction-correction主张不稳（B3反例）、RMSE非充分指标、clamp物理包装过度、Full在B3高bias需解释。

最低修改：重新设计gate使B2中gate_open%<100%、增加NoGate/GM-only对照、多目标指标、重写claims、解释B3高bias、clamp描述为数值保护。

### Actions Taken
1. 诊断确认: λ_min gate在圆形轨迹永不关闭 (λ_min≈3e-6 >> 1e-8)
2. 诊断确认: 即使在直线轨迹, ALOS制导产生航向修正 → r≠0 → gate永开
3. 测试yaw-rate替代门控: |r|>0.0005对应circle=52.6%, straight=73.1% → 仍有大量时间"开"
4. 根本结论: 在反馈控制AUV场景中, gate机制(无论哪种判据)都无法可靠区分激励/非激励阶段

### Status
→ Continuing to Round 2

---

## Round 2 (2026-06-22)

### Assessment
- **Score**: 6/10 (raised from 4/10)
- **Verdict**: 有投稿潜力，但还不是submission-ready

### Key Updates
1. 接受论文pivot：主贡献从"三机制PC-RCO" → "以clamp为核心的鲁棒估计框架"
2. GM/gate降级为limitation/future work
3. Prediction-correction重新定位为trade-off而非绝对优势

### Reviewer's Minimum Requirements for 7-7.5/10
1. Clamp阈值敏感性实验 (max_dc sweep: 0.01-Inf)
2. RMSE-physical plausibility Pareto图
3. C1压力矩阵 (mismatch × noise grid)
4. C2 trade-off两面报告
5. 与KIN/CFD-Luenberger/EKF baseline同场比较
6. GM/gate limitation诚实表述

### New Claim Structure
**C1 (PRIMARY)**: Update-rate clamping prevents physically impossible estimation jumps under model mismatch and sensor noise
**C2 (SUPPORTING)**: Prediction-correction trades convergence speed for transient smoothness
**C3 (DESIGN)**: GM decay is a design provision for unactuated drift phases (not empirically validated)

### Status
→ Implementing Round 2 fixes: clamp sensitivity + stress grid + baseline Pareto
