# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

> **当前线程：海流扰动观测与物理模型** | 切换：`/worktree dynamics|control|eso`

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

## 线程 3：海流扰动观测与物理模型

### 标准 ESO（`eso/vec_leso_update_adv.m`）

- 6-DOF 线性扩展状态观测器，自适应带宽（omega0_base → omega0_max）
- Z3 状态估计总扰动（含海流 + 模型不确定性），前馈补偿到 SMC 控制律

### 物理信息 ESO / PI-ESO（`eso/vec_pieso_update.m`）

- 在标准 ESO 中嵌入 Gauss-Markov 海流模型：`Z3_dot = -Λ·Z3 + β3·fal(e)`，Λ = 1/τ_c
- τ_c 为海流相关时间常数（典型值 50-100s），体现低频海流的时间持续性
- 支持 RK4 积分、自适应带宽、Z3 低通滤波和饱和限制
- `xhy_simulator.m` 已集成：`usePIESO=1` 切换 PI-ESO

### Gauss-Markov 海流模型（`model/gauss_markov_current.m`）

- 4 种海流场景：均匀恒定流 / GM 缓变流 / 深度剪切流 / 空间相关流场
- 用于仿真验证 ESO/PI-ESO 对不同海流特性的估计能力

### ESO 诊断

ESO 诊断通过 `vec_leso_update_adv` 的第二个返回值 `aux`：`aux.e`（误差）、`aux.omega0`（自适应带宽）、`aux.z3_filt`（滤波后扰动估计）。

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
```

## 控制架构

```
3D ALOS制导 → 期望姿态/速度 → SMC控制律 → 推力分配 → 5推进器AUV
                                  ↑
                    ESO/PI-ESO 扰动估计（hat_d = M * Z(:,3)）
```

**两种 ESO 模式（`xhy_simulator.m`）：**
- `useESO=0`：不使用 ESO，纯 SMC 控制
- `useESO=1` + `usePIESO=0`：标准 LESO（`vec_leso_update_adv`），无海流先验
- `useESO=1` + `usePIESO=1`：物理信息 ESO（`vec_pieso_update`），嵌入 Gauss-Markov 海流模型

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
- `params.eso.*` — ESO 带宽（omega0_base/max）、滤波截止频率、RK4 开关
- `params.pieso.*` — PI-ESO 参数，继承标准 ESO + Gauss-Markov τ_c 时间常数
- `params.current.*` — 海流速度/方向（Vc=0 关闭海流）
- `params.xhy.surge/yaw/pitch/heave` — XHY 各通道 SMC 增益
- `params.alos.*` — ALOS 前视距离、自适应增益
- `params.heading/pitch/surge` — REMUS 平台控制器参数

## 关键模型文件

| 文件 | 功能 | 备注 |
|------|------|------|
| `eso/vec_leso_update_adv.m` | 标准 LESO（自适应带宽） | 6-DOF 扰动观测，前馈补偿 |
| `eso/vec_pieso_update.m` | 物理信息 ESO（PI-ESO） | 嵌入 GM 海流模型，Z3 含衰减项 -Λ·Z3 |
| `model/gauss_markov_current.m` | 时变海流生成器 | 4 种场景：均匀/GM缓变/剪切/空间相关 |
| `model/xhy.m` | XHY 6-DOF 动力学主模型 | CFD 阻力 + 推力分配 |
| `model/xhy_drag_cfd.m` | CFD 标定阻力模型 | Fluent 仿真系数 |
| `model/drag.m` | 线性阻尼矩阵 | 时间常数法 |

## 调试

历史数据结构体 `hist`：
- `hist.x` — 状态 [u v w p q r x y z phi theta psi]，12列
- `hist.Z` — ESO 状态（6×3 展开为 18 列）
- `hist.tau` / `hist.tau_cmd` — 力/力矩指令

ESO 诊断：`vec_leso_update_adv` 第二返回值 `aux`（e, omega0, z3_filt）。

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

## 飞书通知

使用 `/feishu-notify-auv` skill 或 `notify_feishu.sh` 脚本发送通知。

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
