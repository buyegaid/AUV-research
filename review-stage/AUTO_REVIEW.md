# Auto Review Log: EG-UCCO

**Started**: 2026-06-10
**Previous review**: 2026-06-04 (completed, score 7.5/10)
**Changes**: New Børhaug baseline, KIN velocity-residual rewrite, K_obs sweep(4-20→10), mismatch mechanism, 250 runs, paper filled

---


## Round 1 (2026-06-10)

### Assessment (Summary)
- **Score**: 5.0/10 current → 6.5-7.0/10 after fixes
- **Verdict**: Not ready → Almost ready (after critical fixes)
- **Reviewer**: Codex GPT-5.5 xhigh, Ocean Engineering level

### Key Criticisms (ranked)
1. Core contribution doesn't match results — UCCO doesn't beat EKF
2. Gate evidence insufficient — need UCCO-Gate vs UCCO-NoGate ablation
3. Børhaug baseline may be strawman (RMSE=2.284 too poor)
4. Monte Carlo results unfavorable to UCCO
5. Main table only 2 seeds
6. K_obs manually tuned, no sensitivity analysis provided
7. High noise degradation not acknowledged
8. Single platform, no real experiment

### Actions Taken (Round 1)
1. **Gate ablation v2**: UCCO-Gate vs UCCO-NoGate on straight+step (5 seeds, low noise)
   - **CRITICAL FINDING**: Gate has ZERO effect (<2% difference) in ALL scenarios
   - UCCO-NoGate is stable on straight (RMSE 0.16) — not like Børhaug (2.28)
   - Root cause: λ_min always > 1e-8 due to DVL noise providing perpetual "excitation"
   - Real stability mechanisms: prediction-correction architecture + GM decay + max_dc clamping
2. **Claims rewritten**: From "superior accuracy" → "53× better than ungated under low excitation; competitive with EKF under sustained excitation"
3. **Børhaug renamed**: → "CFD-Luenberger observer (inspired by Børhaug 2007)"
4. **Gate reframed**: From "excitation gate switch" → "adaptive regularization via Gramian condition number"
5. **Gate ablation table updated**: Honest reporting that gate has no effect in tested conditions

### Gate Ablation v2 Results

| Scenario | Mismatch | Gate | NoGate | Benefit |
|----------|----------|------|--------|---------|
| Straight | 0% | 0.160 | 0.158 | -1.7% |
| Straight | 20% | 0.167 | 0.168 | +0.4% |
| Straight | 30% | 0.173 | 0.173 | +0.3% |
| Step | 0% | 0.216 | 0.216 | 0.0% |
| Step | 20% | 0.224 | 0.224 | 0.0% |
| Step | 30% | 0.229 | 0.229 | 0.0% |

Gate has no measurable effect — the Gramian eigenvalue never drops below 1e-8 under test conditions.

### Status
- Continuing to Round 2
- Pending: English paper sync, 10-seed main table, noise limitation section

