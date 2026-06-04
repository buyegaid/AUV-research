# Auto Review Loop — EG-UCCO 海流观测器

**开始**: 2026-06-04 | **难度**: medium | **审阅模型**: gpt-5.5 (Codex MCP xhigh)

---

## Round 3 (2026-06-04)

### Assessment: Score 6.5/10, Almost

### Actions: Full experiment matrix (3 scenarios × 4 mismatch × 5 methods × 2 seeds)
- UCCO 12/12 cells best Vc estimate
- Degradation at 30% mismatch: UCCO +3%, EKF +9%

---

## Round 4 — FINAL (2026-06-04)

### Assessment: Score 7.5/10, Almost ready

### Actions: Gate ablation + sensor noise Monte Carlo (10 seeds)

**Gate ablation (3 configs × 3 noise levels):**
| Gate | Noiseless | Low Noise | High Noise |
|------|:---:|:---:|:---:|
| Gramian (mu=1e-8) | 0.195 | 0.263 | **0.512** |
| No gate | 0.202 | 0.278 | 0.668 |
| High thresh | 0.334* | 0.334* | 0.334* |

Gate benefit: +3.6% → +5.6% → +23.3% with increasing noise

**Monte Carlo (10 seeds, low noise):**
| Scenario | Mismatch | KIN | EKF-tuned | UCCO | UCCO advantage |
|----------|:---:|:---:|:---:|:---:|:---:|
| Circle | 0% | 0.272 | 0.276 | 0.279 | -1% |
| Circle | 20% | 0.272 | 0.292 | 0.270 | +7.5% |
| Straight | 0% | 0.278 | 0.282 | 0.306 | -8% |
| Straight | 20% | 0.280 | 0.290 | **0.276** | +4.8% |

**Honest assessment with noise:**
- UCCO advantage is strongest under MODEL MISMATCH (not in ideal conditions)
- Under noise + ideal model: methods are comparable (within 1-8%)
- Under noise + mismatch: UCCO maintains accuracy while EKF degrades 3-6%
- UCCO variance (std~0.05) > EKF variance (std~0.03) → needs more seeds

**Revised paper claim:**
> "UCCO provides robust current estimation specifically under hydrodynamic model mismatch, maintaining accuracy where EKF degrades. Under ideal sensor conditions, UCCO substantially outperforms; with realistic sensor noise, the advantage narrows but the robustness to model mismatch persists."

### Verdict: Almost ready. Key refinements for submission:
1. 20+ seeds for statistical power (current: 10 seeds)
2. Clear "robustness under mismatch" narrative (not absolute accuracy)
3. Gate ablation as mechanism evidence
4. Honest discussion of noise sensitivity

---

## Method Description

**EG-UCCO (Excitation-Gated Uncertainty-Calibrated Current Observer)**

The core innovation is using a CFD-pre-calibrated 6-DOF dynamics model as a deterministic prior for ocean current estimation. The observer operates at the velocity level (not noisy acceleration level):

1. **Velocity prediction**: One-step forward prediction using CFD dynamics with estimated current [cN, cE]
2. **Sensitivity Gramian**: Numerical perturbation of CFD model computes ∂(ν_pred)/∂c  
3. **Excitation gate**: Updates only when λmin(Φ'Φ) > μ_gate — mathematically grounded (Fisher information) rather than heuristic
4. **Gradient update**: c_hat += γ · Φ' · e_vel with adaptive step size
5. **Yaw-only feedforward**: Compensation ablation showed surge compensation damages heading control due to thruster-yaw coupling

**Data flow**: INS+DVL+Depth → ν_meas → [EG-UCCO] → [cN, cE] → Body-frame conversion → Yaw moment compensation → SMC controller → Thrust allocation → XHY AUV

---

## Score Progression

| Round | Score | Key |
|:---:|:---:|------|
| 1 | 4.0 | Baseline prototype |
| 2 | 5.5 | Comp ablation + mismatch |
| 3 | 6.5 | Full experiment matrix |
| 4 | 7.5 | Gate ablation + MC |

**Status: Completed at MAX_ROUNDS. Ready to begin paper writing.**
