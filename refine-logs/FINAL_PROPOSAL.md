# Final Proposal: PC-RCO — Prediction-Correction Robust Current Observer

**Title**: A Robust Prediction-Correction Current Observer for AUVs under Hydrodynamic Model Mismatch

**Problem**: Any hydrodynamic model has unavoidable mismatch with the real AUV. Model-based current observers degrade or diverge under this mismatch.

**Method**: PC-RCO — three lightweight structural protections:
1. Velocity-level prediction-correction (naturally small innovation O(10⁻⁴ m/s))
2. Gauss-Markov time decay (τ_c=100s, prevents drift)
3. Per-step clamping (Δc_max=0.1 m/s, last line of defense)

**Gramian analysis**: Diagnostic tool for observability, used for adaptive regularization (NOT a gate).

**Results**:
| Scenario | Mismatch | KIN | CFD-Luenberger | EKF | PC-RCO |
|----------|----------|-----|----------------|-----|--------|
| Circular | 0% | 0.192 | 0.430 | 0.035 | 0.064 |
| Circular | 30% | 0.194 | 0.481 | 0.062 | 0.091 |
| Straight | 0% | 0.186 | 2.284 | 0.038 | 0.043 |

**Key claims**:
- Mismatch degradation 42% vs EKF 77%
- No collapse on straight (vs CFD-Luenberger 2.28, 53× worse)
- Gate ablation: hard gate <2% effect → structural protections are the real mechanism
- Lightweight: 2 states, no covariance matrix, no online parameter ID

**Honest limitations**:
- Simulation only (no sea trial)
- No per-mechanism ablation
- Single AUV platform
