# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

> **当前线程：控制方法研究** | 切换：`/worktree control`

## 工作目录

**所有MATLAB程序都从项目根目录运行：** `C:\Users\sixuh\Documents\B_matlab_ws\AUV\research`

确保路径已添加：
```matlab
addpath('.', './Lib', './guidance', './controller/xhy', './controller/remus', './model', './eso', './post', './traj');
```

## 研究目标

**AUV 抗流控制**：用 ESO（扩展状态观测器）估计海流扰动，并前馈补偿到 SMC 控制律中，与纯 SMC 和 ISMC（积分滑模）进行对比。

**三种对比方法：**
- **SMC**：标准滑模控制，无积分项（`controller/remus/SMCheading.m`，HeadingMode=2）
- **ISMC**：积分滑模控制，含积分项消除稳态误差（`controller/remus/my_integralSMCheading.m`，HeadingMode=1）
- **SMC+ESO**：SMC + ESO 扰动前馈补偿（useESO=1，HeadingMode=2）

**两个平台的角色：**
- **XHY**：自研 5 推进器 AUV，目标平台，使用推力分配实现力/力矩控制
- **REMUS 100**：标准库模型（Fossen MSS Toolbox），用于方法验证，使用舵面控制

---

## 线程 2：控制方法研究

### XHY 平台（5 推进器推力分配）

- 4 通道 SMC 控制器：`smc_surge_xhy`（纵荡）、`smc_yaw_xhy`（转艏）、`smc_pitch_xhy`（俯仰）、`smc_heave_xhy`（垂荡）
- 深度控制双模式：`DepthMode=1`（俯仰角控深）vs `DepthMode=2`（直接 Z 力控深）
- Station-keeping 定点控制测试：`run_sk.m`、`run_sk_omega0.m`、`run_sk_tauc.m` 等

### REMUS 平台（舵面控制，方法验证）

- SMC/ISMC/SMC+ESO 三种方法对比框架
- 3D ALOS 制导 + 航向/俯仰/速度三通道控制

### 控制器参数调整

- ALOS 自适应增益 `gamma_h/gamma_v` 从 0.002 → 0（关闭自适应，由 ESO 补偿海流）
- SMC 纵荡通道从 d1/d2 参数化 → T1 时间常数参数化

### XHY 单通道 SMC 控制器接口

所有控制器使用 `persistent` 变量保存积分状态，**切换仿真前必须 `clear` 对应函数**（`xhy_simulator.m:6` 已处理）。

| 控制器 | 输入 | 输出 | 关键参数 |
|--------|------|------|----------|
| `smc_surge_xhy` | u, u_d, u_d_dot | X (N) | m_eff, T1, lambda, Kd, Ks（T1 时间常数参数化） |
| `smc_yaw_xhy` | psi, r, psi_d, r_d | N (N·m) | Iz_eff, lambda, Kd, Ks |
| `smc_pitch_xhy` | theta, q, theta_d, q_d | M (N·m) | Iy_eff, lambda, Kd, Ks |
| `smc_heave_xhy` | zn, w, z_d, w_d | Z (N) | m_eff_z, d_w, g_z |

### 深度控制模式（XHY）

在 `xhy_simulator.m` 中修改 `DepthMode`：
- `DepthMode = 1` — 俯仰角控深：ALOS → theta_ref → SMC pitch → M 力矩
- `DepthMode = 2` — 直接 Z 力控深：深度误差 → SMC heave → Z 力（俯仰保持水平）

---

## 运行仿真

```matlab
% XHY平台（推荐，目标平台）
xhy_simulator   % 参数在脚本内修改: useESO, TrajMode, DepthMode

% REMUS平台对比实验（三种方法）
params = get_params;
hist_smc  = main_loop_remus(0, 2, 1, 1, 2, 1, params);  % 纯SMC
hist_ismc = main_loop_remus(0, 2, 1, 1, 1, 1, params);  % ISMC
hist_eso  = main_loop_remus(1, 2, 1, 1, 2, 1, params);  % SMC+ESO

% 对比分析
compare_results(hist_smc, hist_eso, 'SMC vs SMC+ESO');
% 指标：航向误差RMSE、横向误差RMSE、舵角RMS、滑模面RMS
```

`main_loop_remus` 参数顺序：`(useESO, TrajMode, CurrentModel, ControlFlag, HeadingMode, KinematicsFlag, params)`

## 控制架构

```
3D ALOS制导 → 期望姿态/速度 → SMC控制律 → 推力分配 → 5推进器AUV
                                  ↑
                    ESO/PI-ESO 扰动估计（hat_d = M * Z(:,3)）
```

**ESO 补偿方式：**
```matlab
hat_d = M * Z(:, 3);          % 加速度单位 → 力/力矩单位
X_cmd = smc_surge_xhy(...) - hat_d(1);   % 前馈补偿
N_cmd = smc_yaw_xhy(...)   - hat_d(6);
```

**REMUS 平台 ESO 补偿方式（`controller/remus/SMCheading.m`）：**
```matlab
delta_eso = -(T_nomoto / K_nomoto) * rho_eso * hat_dr;  % 舵角补偿
```

## 参数配置

所有可调参数集中在 `Lib/get_params.m`：
- `params.current.*` — 海流速度/方向（Vc=0 关闭海流）
- `params.xhy.surge/yaw/pitch/heave` — XHY 各通道 SMC 增益
- `params.eso.*` — ESO 带宽（omega0_base/max）、滤波截止频率、RK4 开关
- `params.pieso.*` — PI-ESO 参数，继承标准 ESO + Gauss-Markov τ_c 时间常数
- `params.alos.*` — ALOS 前视距离、自适应增益（gamma=0 关闭，由 ESO 补偿）、航点切换半径
- `params.heading/pitch/surge` — REMUS 平台控制器参数

## 关键模型文件

| 文件 | 功能 | 备注 |
|------|------|------|
| `model/xhy.m` | XHY 6-DOF 动力学主模型 | 阻力用 `xhy_drag_cfd`（CFD 数据），推进器用推力分配 |
| `model/xhy_drag_cfd.m` | CFD 标定的阻力模型 | Surge/Sway/Heave 二次阻力系数来自 Fluent 仿真 |
| `model/drag.m` | 通用线性阻尼矩阵 | 时间常数法（T1=20s surge, T2=20s sway, T6=1s yaw） |
| `model/thrust_main.m` | 主推进器推力模型 | KT_f=0.019（等效系数，含实际损耗） |
| `model/thrust_aux.m` | 辅助推进器推力模型 | 4 垂推/侧推，含死区补偿 |
| `model/gauss_markov_current.m` | 时变海流生成器 | 4 种场景：均匀/GM缓变/剪切/空间相关 |
| `eso/vec_leso_update_adv.m` | 标准 LESO（自适应带宽） | 6-DOF 扰动观测，前馈补偿 |
| `eso/vec_pieso_update.m` | 物理信息 ESO（PI-ESO） | 嵌入 GM 海流模型，Z3 含衰减项 -Λ·Z3 |

## 调试

历史数据结构体 `hist`：
- `hist.x` — 状态 [u v w p q r x y z phi theta psi]，12列
- `hist.ui` — 控制输入（XHY: 5推进器RPM；REMUS: [delta_r, delta_s, n]）
- `hist.tau` / `hist.tau_cmd` — 力/力矩指令
- `hist.Z` — ESO 状态（6×3 展开为 18 列）
- `hist.e_psi`, `hist.e_y`, `hist.sigma_heading` — REMUS 误差诊断量

推力分配诊断：`[ui, info] = thrust_allocation_xhy(tau_cmd, thr_params)`，`info` 含饱和状态。

## 代码约定

- **坐标系：** NED 惯性系，船体坐标系（前右下）
- **状态向量：** `[u v w p q r x y z phi theta psi]'`
- **XHY 控制输入：** RPM（正值为正推力）；**REMUS 控制输入：** 舵角 rad + 转速 RPM
- **角度单位：** 弧度（rad）
- **注释语言：** 中文

## 横滚通道不可控

XHY 的 B_thr 矩阵第 4 行（K 通道）全为 0，5 推进器配置无法产生独立横滚力矩，推力分配使用伪逆求解。

## 飞书文档更新

使用 `lark-doc` skill 将进展写入飞书文档"论文工作"。

**文档信息：**
- Wiki URL：`https://my.feishu.cn/wiki/GKuswOjxIi7kBAktIumcy14Vnxg`
- "进展记录及规划"章节 block ID：`D14EdBR8Aol39txvpVhcpg5en4e`

**追加内容到进展章节：**
```bash
lark-cli docs +fetch --api-version v2 \
  --doc "https://my.feishu.cn/wiki/GKuswOjxIi7kBAktIumcy14Vnxg" \
  --as user --scope outline

lark-cli docs +update --api-version v2 \
  --doc "https://my.feishu.cn/wiki/GKuswOjxIi7kBAktIumcy14Vnxg" \
  --as user --command append \
  --block-id "D14EdBR8Aol39txvpVhcpg5en4e" \
  --content '<h2>标题</h2><p>内容</p>'
```

**注意：** `--command append --block-id <章节id>` 将内容追加到该章节末尾。写入前先用 `+fetch --scope section --start-block-id` 读取现有内容避免重复。

## 飞书通知

使用 `/feishu-notify-auv` skill 或 `notify_feishu.sh` 脚本向用户发送实验/仿真完成通知。

```bash
bash notify_feishu.sh "标题" "内容"
```

<!-- ARIS:BEGIN -->
## ARIS Skill Scope
ARIS skills are installed **globally** at `~/.claude/skills/` (107 entries).
Global manifest: `~/.aris/installed-skills.txt`.
Do not modify or delete files inside any skill directory (content comes from `/c/Users/sixuh/aris_repo`).
Update with: `bash /c/Users/sixuh/aris_repo/tools/install_aris.sh /c/Users/sixuh/ --reconcile`
<!-- ARIS:END -->
