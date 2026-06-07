# 2026-06-07 目录迁移记录

本次迁移只调整目录边界和路径初始化，不改变控制算法、模型参数、函数签名或 `hist` 数据结构。

## 主要迁移

- `Lib/ controller/ eso/ guidance/ model/ post/ traj/ RL/` 迁入 `src/matlab/`。
- 根目录仿真入口迁入 `apps/`。
- 对比实验和批处理脚本迁入 `experiments/`。
- 水池分析、标定、系统辨识和 Python 绘图迁入 `analysis/`。
- `model/test`、`eso/test` 和根目录 quick test 迁入 `tests/`。
- 原始 CSV 迁入 `data/raw/`，历史图片迁入 `assets/figures/`。
- `AGENTS_control.md`、`AGENTS_eso.md`、`CLAUDE_control.md`、`CLAUDE_eso.md` 迁入 `docs/agent/`；与根文档完全重复的 dynamics 副本删除。
- `paper/main.aux`、`paper/main.bbl`、`paper/main.blg`、`paper/main.out` 删除并加入忽略规则。

## 兼容方式

旧式手动 `addpath('.', './Lib', ... )` 已替换为：

```matlab
project_root = setup_paths();
```

如需从脚本中构造路径，请使用 `fullfile(project_root, ...)`。
