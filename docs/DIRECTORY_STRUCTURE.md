# 目录结构说明

本仓库仍以项目根目录作为 MATLAB 运行基准。进入仓库后先运行：

```matlab
project_root = setup_paths();
```

## 顶层目录

| 目录 | 用途 |
| --- | --- |
| `src/matlab/` | 核心 MATLAB 代码，包括模型、控制器、ESO、制导、后处理和 RL 组件 |
| `apps/` | 常用仿真入口和主循环 |
| `experiments/` | 对比实验、站位/轨迹实验、参数敏感性和批处理脚本 |
| `experiments/diagnostics/` | 论文诊断图、消融、Monte Carlo 和模型失配实验 |
| `analysis/` | 水池实验分析、系统辨识、标定、timeline 提取和 Python 绘图 |
| `tests/` | 模型、ESO 和集成快速测试 |
| `data/raw/` | 原始实验 CSV 数据 |
| `assets/figures/` | 已保留的关键历史图和分析图 |
| `paper/` | 论文源码、参考文献和保留版 PDF |
| `docs/agent/` | 分线程/分角色 agent 说明文档 |

## 运行约定

- 所有 MATLAB 脚本从项目根目录运行。
- `startup.m` 会自动调用 `setup_paths.m`；已有 MATLAB 会话可手动运行 `setup_paths`。
- 自动生成结果默认写入 `results/`、`assets/figures/` 或 `paper/figures/`。
- `results/`、`.mat`、LaTeX 中间文件和 RL 训练产物不提交；`data/raw/*.csv`、`assets/figures/*.png`、`paper/main.pdf` 作为关键产物保留。

## 常用入口

```matlab
project_root = setup_paths();
xhy_simulator
run_comparison_smc_pieso
test_comparison_quick
test_xhy_dynamics
test_eso_performance
```
