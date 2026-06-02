# AGENTS.md

This file provides guidance to Codex (Codex.ai/code) when working with code in this repository.

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

% 测试动力学模型
cd model/test && test_xhy_dynamics
```

`main_loop_remus` 参数顺序：`(useESO, TrajMode, CurrentModel, ControlFlag, HeadingMode, KinematicsFlag, params)`

## 控制架构

```
3D ALOS制导 → 期望姿态/速度 → SMC控制律 → 推力分配 → 5推进器AUV
                                  ↑
                              ESO扰动估计（hat_d = M * Z(:,3)）
```

**ESO 补偿方式（`xhy_simulator.m:89-93`）：**
```matlab
hat_d = M * Z(:, 3);          % 加速度单位 → 力/力矩单位
X_cmd = smc_surge_xhy(...) - hat_d(1);   % 前馈补偿
N_cmd = smc_yaw_xhy(...)   - hat_d(6);
```

**REMUS 平台 ESO 补偿方式（`controller/remus/SMCheading.m`）：**
```matlab
delta_eso = -(T_nomoto / K_nomoto) * rho_eso * hat_dr;  % 舵角补偿
```

## XHY 单通道 SMC 控制器接口

所有控制器使用 `persistent` 变量保存积分状态，**切换仿真前必须 `clear` 对应函数**（`xhy_simulator.m:6` 已处理）。

| 控制器 | 输入 | 输出 | 关键参数 |
|--------|------|------|----------|
| `smc_surge_xhy` | u, u_d, u_d_dot | X (N) | m_eff, T1, lambda, Kd, Ks |
| `smc_yaw_xhy` | psi, r, psi_d, r_d | N (N·m) | Iz_eff, lambda, Kd, Ks |
| `smc_pitch_xhy` | theta, q, theta_d, q_d | M (N·m) | Iy_eff, lambda, Kd, Ks |
| `smc_heave_xhy` | zn, w, z_d, w_d | Z (N) | m_eff_z, d_w, g_z |

## 参数配置

所有可调参数集中在 `Lib/get_params.m`：
- `params.current.*` — 海流速度/方向（Vc=0 关闭海流）
- `params.xhy.surge/yaw/pitch/heave` — XHY 各通道 SMC 增益
- `params.eso.*` — ESO 带宽（omega0_base/max）、滤波截止频率
- `params.alos.*` — ALOS 前视距离、自适应增益、航点切换半径
- `params.heading/pitch/surge` — REMUS 平台控制器参数

## 深度控制模式（XHY）

在 `xhy_simulator.m` 中修改 `DepthMode`：
- `DepthMode = 1` — 俯仰角控深：ALOS → theta_ref → SMC pitch → M 力矩
- `DepthMode = 2` — 直接 Z 力控深：深度误差 → SMC heave → Z 力（俯仰保持水平）

## 调试

历史数据结构体 `hist`：
- `hist.x` — 状态 [u v w p q r x y z phi theta psi]，12列
- `hist.ui` — 控制输入（XHY: 5推进器RPM；REMUS: [delta_r, delta_s, n]）
- `hist.tau` / `hist.tau_cmd` — 力/力矩指令
- `hist.Z` — ESO 状态（6×3 展开为 18 列）
- `hist.e_psi`, `hist.e_y`, `hist.sigma_heading` — REMUS 误差诊断量

ESO 诊断通过 `vec_leso_update_adv` 的第二个返回值 `aux`：`aux.e`（误差）、`aux.omega0`（自适应带宽）、`aux.z3_filt`（滤波后扰动估计）。

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

**使用 Skill（推荐）：**
```bash
/feishu-notify-auv
```

**直接使用脚本：**
```bash
bash notify_feishu.sh "标题" "内容"
```

**使用场景：**
- 仿真运行完成（XHY/REMUS）
- 对比实验结果生成
- 长时间运行任务完成
- 关键指标计算完成

**示例：**
```bash
# 仿真完成通知
bash notify_feishu.sh "XHY 仿真完成" "SMC+ESO vs SMC 对比完成
航向误差 RMSE: 0.05 rad
横向误差 RMSE: 0.12 m
仿真时间: 300s"

# 实验结果通知
bash notify_feishu.sh "三种方法对比完成" "SMC、ISMC、SMC+ESO 对比分析完成
图表已生成: pic/comparison_results.png"
```

**配置：**
- 用户 open_id：`ou_7464ad1f38b07ce9f32b39bfcdb5c9dc`（已配置在脚本中）
- 使用 bot 身份发送消息
- 消息格式：Markdown

<!-- ARIS:BEGIN -->
## ARIS Skill Scope
ARIS skills are installed **globally** at `~/.Codex/skills/` (107 entries).
Global manifest: `~/.aris/installed-skills.txt`.
Do not modify or delete files inside any skill directory (content comes from `/c/Users/sixuh/aris_repo`).
Update with: `bash /c/Users/sixuh/aris_repo/tools/install_aris.sh /c/Users/sixuh/ --reconcile`
<!-- ARIS:END -->
