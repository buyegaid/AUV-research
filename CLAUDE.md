# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

这是一个6自由度AUV（自主水下航行器）轨迹跟踪仿真项目，基于自研的5推进器XHY平台。项目同时保留了REMUS 100模型用于对比研究。

**控制架构：** 3D ALOS制导 → SMC控制律 + ESO扰动补偿 → 推力分配 → 动力学模型

**两套控制方案：**
1. **单通道独立控制**（推荐）：surge/yaw/pitch/heave各自独立SMC控制器
2. **6-DOF统一控制**（实验性）：统一计算6维力/力矩指令

## 运行仿真

```matlab
% XHY平台仿真（推荐）
xhy_simulator
% 参数在脚本内修改: useESO, TrajMode, DepthMode

% XHY 6-DOF统一控制（实验性）
params = get_params;
hist = main_loop_xhy(1, 1, 1, params);
% 参数: useESO, TrajMode, CurrentModel, params

% REMUS平台仿真（旧模型）
Remus_simlulator
% 或: hist = main_loop_remus(useESO, TrajMode, CurrentModel, ControlFlag, HeadingMode, KinematicsFlag, params);

% 测试动力学模型
cd model
test_xhy_dynamics
```

**无需编译** — MATLAB直接解释.m文件。确保路径已添加：
```matlab
addpath('.', './Lib', './guidance', './controller', './model', './eso', './post', './traj');
```

## 代码架构

### 主仿真入口
- **`xhy_simulator.m`** — XHY平台主入口（**推荐使用**）
  - 单通道SMC控制架构（surge/yaw/pitch或heave独立控制）
  - 支持两种深度控制模式：俯仰角控深(DepthMode=1) 或 直接Z力控深(DepthMode=2)
  - 1000秒仿真，时间步长0.01s
  
- **`main_loop_xhy.m`** — XHY 6-DOF统一控制（实验性）
  - 使用`smc_6dof.m`统一计算6维力/力矩
  - 500秒仿真，时间步长0.01s
  
- **`Remus_simlulator.m`** / **`main_loop_remus.m`** — REMUS平台仿真（旧模型）

### 动力学模型
- **`model/xhy.m`** — XHY模型：5推进器（主推+2垂直+2侧向），33kg
  - 输入：`[n_main, n_vert1, n_vert2, n_side1, n_side2]'` (RPM)
  - 输出：`[xdot, U, M, C, D, g, tau]`
  - 状态：`x = [u v w p q r x y z phi theta psi]'` (12维)
  
- **`model/remus100.m`** — REMUS模型：3推进器（主推+2垂直），31.9kg

### 控制器

**单通道SMC控制器（当前在用，xhy_simulator.m）：**
- **`controller/smc_surge_xhy.m`** — 纵荡速度控制 → X力
- **`controller/smc_yaw_xhy.m`** — 航向角控制 → N力矩
- **`controller/smc_pitch_xhy.m`** — 俯仰角控制 → M力矩
- **`controller/smc_heave_xhy.m`** — 深度控制 → Z力

**6-DOF统一控制器（实验性，main_loop_xhy.m）：**
- **`controller/smc_6dof.m`** — 6自由度滑模控制器
  - 输入：当前速度、期望速度/加速度、动力学矩阵、ESO扰动估计
  - 输出：6维力/力矩指令 `tau_cmd`

**REMUS平台控制器（旧架构）：**
- `controller/my_integralSMCheading.m` — 航向角控制
- `controller/my_integralSMCpitch.m` — 俯仰角控制
- `controller/my_SMCsurge.m` / `my_PIDsurge.m` — 纵荡速度控制

### 推力分配
- **`controller/thrust_allocation_xhy.m`** — 6-DOF力/力矩 → 5推进器RPM
  - 使用伪逆求解（横滚通道不可控）
  - 推力模型：T = ρ·D_prop^4·KT·|n|·n
  - 包含饱和限幅和诊断输出

### 扰动观测器
- **`eso/vec_leso_update_adv.m`** — 向量化扩展状态观测器
  - 输入：6x3状态矩阵Z、6x1测量y、已知加速度a_known
  - 输出：更新后的Z（Z3列为扰动估计）
  - 特性：自适应带宽、RK4积分、测量值与输出低通滤波

### 制导
- **`guidance/my_ALOS3D.m`** — 3D自适应视线制导
  - 输入：当前位置、航点序列
  - 输出：期望航向角psi_ref、俯仰角theta_ref、横向/垂向误差

### 轨迹生成
- **`traj/line_traj.m`** — 直线轨迹
- **`traj/traj.m`** — 圆形/螺旋轨迹

### 后处理与可视化
- **`post/plot_xhy_results.m`** — XHY仿真结果绘图
- **`post/plot_compare_single.m`** — 单次仿真结果绘图
- **`post/compare_results.m`** — 对比两次仿真结果

### 参数配置
- **`Lib/get_params.m`** — 所有可调参数的中心配置
  - 海流参数、SMC增益、ESO带宽、ALOS前视距离、AUV物理参数

### 强化学习（实验性）
- **`RL/`** — 强化学习控制器训练与测试
  - `train_*.m` / `test_*.m` — 训练/测试脚本
  - `env/` — 仿真环境
  - `lib/` — 辅助函数

## 项目状态

**已完成：**
1. ✅ XHY动力学模型 (`model/xhy.m`)
2. ✅ 推力分配逆映射 (`controller/thrust_allocation_xhy.m`)
3. ✅ 单通道SMC控制器 (`smc_surge_xhy.m`, `smc_yaw_xhy.m`, `smc_pitch_xhy.m`, `smc_heave_xhy.m`)
4. ✅ 6-DOF统一SMC控制器 (`controller/smc_6dof.m`)
5. ✅ ESO扰动观测器 (`eso/vec_leso_update_adv.m`)
6. ✅ XHY主仿真循环 (`xhy_simulator.m`, `main_loop_xhy.m`)
7. ✅ 3D ALOS制导 (`guidance/my_ALOS3D.m`)

**待完成：**
- [ ] 参数调优（SMC增益、ESO带宽、推力分配权重）
- [ ] 性能验证（直线/圆形轨迹、海流扰动鲁棒性）
- [ ] 对比分析（SMC vs SMC+ESO性能提升）
- [ ] 横滚稳定控制（当前横滚通道不可控）
- [ ] 推进器动态模型（当前假设瞬时响应）

**两套控制架构对比：**

| 特性 | 单通道控制 (xhy_simulator) | 6-DOF统一控制 (main_loop_xhy) |
|------|---------------------------|-------------------------------|
| 状态 | ✅ 当前推荐使用 | ⚠️ 实验性 |
| 控制器 | 独立SMC（surge/yaw/pitch/heave） | 统一smc_6dof |
| 深度控制 | 俯仰角控深 或 直接Z力控深 | 仅俯仰角控深 |
| 代码复杂度 | 低，易调试 | 高，耦合性强 |
| 适用场景 | 工程实现、快速验证 | 理论研究、多通道协同 |

## 代码约定

- **坐标系：** NED（北东地）惯性系，船体坐标系（前右下）
- **状态向量：** `[u v w p q r x y z phi theta psi]'`（速度+位置+姿态）
- **控制输入：** RPM（转/分钟），正值为正推力
- **角度单位：** 弧度（rad）
- **注释语言：** 中文

## 文件命名模式

- `my_*` — 自定义实现（区别于标准库）
- `*_params.m` — 参数配置文件
- `test_*.m` — 单元测试脚本
- `*_control.m` — 控制器实现
- `*_update.m` — 状态更新函数（如ESO）

## 调试技巧

- 使用`timebar()`显示仿真进度（在主循环中）
- 历史数据存储在`hist`结构体：
  - `hist.x` — 状态历史 [u v w p q r x y z phi theta psi]
  - `hist.ui` — 控制输入历史（推进器RPM）
  - `hist.tau` 或 `hist.tau_cmd` — 力/力矩指令
  - `hist.Z` — ESO状态（如果使用ESO）
  - `hist.guidance` — 制导信号（psi_ref, theta_ref等）
- 可视化：
  - `plot_xhy_results(hist)` — XHY仿真结果
  - `plot_compare_single(hist)` — 单次仿真结果
  - `compare_results(hist1, hist2)` — 对比两次仿真
- ESO诊断信息在`aux`结构体：
  - `aux.e` — 误差
  - `aux.omega0` — 自适应带宽
  - `aux.z3_filt` — 滤波后扰动估计
- 推力分配诊断：`thrust_allocation_xhy`返回`info`结构体包含饱和状态

## 常见问题

**Q: 如何切换深度控制模式？**  
A: 在`xhy_simulator.m`中修改`DepthMode`：
- `DepthMode = 1` — 俯仰角控深（ALOS给theta_ref → SMC pitch → M力矩）
- `DepthMode = 2` — 直接Z力控深（深度误差 → SMC heave → Z力）

**Q: 如何调整SMC参数？**  
A: 
- 单通道控制器：修改`Lib/get_params.m`中的`params.xhy.surge/yaw/pitch/heave`
- 6-DOF控制器：修改`main_loop_xhy.m`第49-54行的`smc_params`

**Q: 如何启用/禁用ESO？**  
A: 在仿真入口脚本中设置`useESO = 1`（启用）或`useESO = 0`（禁用）

**Q: 横滚通道为什么不可控？**  
A: XHY的5推进器配置（1主推+2垂直+2侧向）无法产生独立的横滚力矩。推力分配矩阵B_thr的第4行（K通道）全为0。

**Q: 仿真运行缓慢怎么办？**  
A: 
- 减少仿真时长（修改`T`变量）
- 增大时间步长（修改`h`，但注意数值稳定性）
- 注释掉`timebar()`调用
