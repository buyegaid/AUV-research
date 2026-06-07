# AGENTS.md

This file provides guidance to Codex (Codex.ai/code) when working with code in this repository.

> **当前线程：XHY 动力学模型搭建** | 切换：`/worktree dynamics`

## 工作目录

**所有MATLAB程序都从项目根目录运行：** `C:\Users\sixuh\Documents\B_matlab_ws\AUV\research`

确保路径已添加：
```matlab
project_root = setup_paths();
```

## 目录结构

- `src/matlab/`：核心 MATLAB 代码（`Lib/`、`controller/`、`model/`、`eso/` 等）
- `apps/`：仿真入口和主循环
- `experiments/`：对比实验、轨迹/站位实验、参数敏感性和批处理
- `analysis/`：水池数据分析、系统辨识、标定和绘图
- `tests/`：模型、ESO 和集成测试
- `data/raw/`：原始实验 CSV
- `assets/figures/`：关键历史图和分析图

详细说明见 `docs/DIRECTORY_STRUCTURE.md`。

## 研究目标

**AUV 抗流控制**：用 ESO（扩展状态观测器）估计海流扰动，并前馈补偿到 SMC 控制律中，与纯 SMC 和 ISMC（积分滑模）进行对比。

**三种对比方法：**
- **SMC**：标准滑模控制，无积分项（`src/matlab/controller/remus/SMCheading.m`，HeadingMode=2）
- **ISMC**：积分滑模控制，含积分项消除稳态误差（`src/matlab/controller/remus/my_integralSMCheading.m`，HeadingMode=1）
- **SMC+ESO**：SMC + ESO 扰动前馈补偿（useESO=1，HeadingMode=2）

**两个平台的角色：**
- **XHY**：自研 5 推进器 AUV，目标平台，使用推力分配实现力/力矩控制
- **REMUS 100**：标准库模型（Fossen MSS Toolbox），用于方法验证，使用舵面控制

---

## 线程 1：XHY 动力学模型搭建

### CFD 仿真

- 已完成：Surge/Sway/Heave 直航+斜航阻力 CFD（Fluent），结果写入 `src/matlab/model/xhy_drag_cfd.m`
- 进行中：转动阻尼 MRF CFD（横滚/俯仰/偏航，12工况），当前用时间常数法临时替代
- 待开展：斜航/旋转臂 CFD（交叉耦合阻尼 36工况）、附加质量交叉项辨识

### 水池实验验证与修正

- **2026-05-30 实验**（T1 前垂推故障）：完成动力学校准，推进器有效推力系数 KT 从理论值 0.33 → 实测等效值 0.019，阻力系数全部更新（详见 [[analysis_report_260530]]）
- **2026-06-01 实验**（T1 修复后）：验证 T1 恢复正常，Z-M 解耦恢复，推进器推力系数重新标定 k=0.001282 CAN-g→N，确认所有通道控制有效（详见 [[水池实验分析_20260603]]）

### 关键发现

- CAN 指令单位"g"为无量纲值，满指令 8000g 实际推力约 6-10 N（非 78.5 N）
- 推进系统总效率 ~2.5%（受待机功耗和低推力工况影响）
- Sway 和 Yaw 为最强控制通道（r≈0.64-0.66），Pitch 激励不足需专项测试
- 模型从 CFD 理想值 → 实测标定值，Surge/Heave/Yaw 稳态速度匹配度 >94%

### 模型更新（2026-05-31）

- `src/matlab/model/xhy.m`：阻力计算从 `forceLiftDrag+crossFlowDrag` → `xhy_drag_cfd`（基于 CFD 数据）
- `src/matlab/model/drag.m`：统一使用时间常数法（T1=T2=20s, T6=1s）
- `src/matlab/model/thrust_main.m`：KT_f 从 0.33 → 0.019（等效推力系数）
- `src/matlab/model/thrust_aux.m`：死区补偿参数更新
- `src/matlab/controller/xhy/smc_surge_xhy.m`：阻力补偿从 d1/d2 二次模型 → T1 时间常数线性模型

### Obsidian 参考文档（tag: 小黄鱼）

| 笔记 | 内容 |
|------|------|
| [[水池实验记录]] | 2026-06-01 水池实验原始记录（时间线、mode 说明、各测试段详情） |
| [[水池实验分析_20260603]] | 2026-06-01 实验完整分析（T1 修复验证、系统辨识、修复前后对比） |
| [[analysis_report_260530]] | 2026-05-30 实验分析（T1 故障诊断、动力学校准方法、CAN 协议） |
| [[CAN协议说明]] | 推进器控制 CAN 协议（帧格式、推力分配矩阵、电机标定、安全机制） |
| [[CFD]] | CFD 仿真执行方案（文献方法总结、P1 转动阻尼/P2 交叉耦合/P3 附加质量计划） |

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

% 测试动力学模型
project_root = setup_paths(); test_xhy_dynamics
```

`main_loop_remus` 参数顺序：`(useESO, TrajMode, CurrentModel, ControlFlag, HeadingMode, KinematicsFlag, params)`

## 控制架构

```
3D ALOS制导 → 期望姿态/速度 → SMC控制律 → 推力分配 → 5推进器AUV
                                  ↑
                    ESO/PI-ESO 扰动估计（hat_d = M * Z(:,3)）
```

## 参数配置

所有可调参数集中在 `src/matlab/Lib/get_params.m`：
- `params.current.*` — 海流速度/方向（Vc=0 关闭海流）
- `params.xhy.surge/yaw/pitch/heave` — XHY 各通道 SMC 增益
- `params.eso.*` — ESO 带宽（omega0_base/max）、滤波截止频率、RK4 开关
- `params.pieso.*` — PI-ESO 参数，继承标准 ESO + Gauss-Markov τ_c 时间常数
- `params.alos.*` — ALOS 前视距离、自适应增益（gamma=0 关闭，由 ESO 补偿）、航点切换半径
- `params.heading/pitch/surge` — REMUS 平台控制器参数

## 关键模型文件

| 文件 | 功能 | 备注 |
|------|------|------|
| `src/matlab/model/xhy.m` | XHY 6-DOF 动力学主模型 | 阻力用 `xhy_drag_cfd`（CFD 数据），推进器用推力分配 |
| `src/matlab/model/xhy_drag_cfd.m` | CFD 标定的阻力模型 | Surge/Sway/Heave 二次阻力系数来自 Fluent 仿真 |
| `src/matlab/model/drag.m` | 通用线性阻尼矩阵 | 时间常数法（T1=20s surge, T2=20s sway, T6=1s yaw） |
| `src/matlab/model/thrust_main.m` | 主推进器推力模型 | KT_f=0.019（等效系数，含实际损耗） |
| `src/matlab/model/thrust_aux.m` | 辅助推进器推力模型 | 4 垂推/侧推，含死区补偿 |
| `src/matlab/model/gauss_markov_current.m` | 时变海流生成器 | 4 种场景：均匀/GM缓变/剪切/空间相关 |
| `src/matlab/eso/vec_leso_update_adv.m` | 标准 LESO（自适应带宽） | 6-DOF 扰动观测，前馈补偿 |
| `src/matlab/eso/vec_pieso_update.m` | 物理信息 ESO（PI-ESO） | 嵌入 GM 海流模型，Z3 含衰减项 -Λ·Z3 |

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
ARIS skills are installed **globally** at `~/.Codex/skills/` (107 entries).
Global manifest: `~/.aris/installed-skills.txt`.
Do not modify or delete files inside any skill directory (content comes from `/c/Users/sixuh/aris_repo`).
Update with: `bash /c/Users/sixuh/aris_repo/tools/install_aris.sh /c/Users/sixuh/ --reconcile`
<!-- ARIS:END -->
