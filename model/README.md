# M080 推进器 MATLAB 仿真模型

本目录提供 M080 深水推进器的准静态模型，用于 AUV/ROV 动力学仿真。

## 文件

- `m080_thruster_model.m`: 主模型函数，输入 PWM 和电压，输出推力、扭矩、功率、转速。
- `m080_thruster_params.m`: 默认参数。
- `demo_m080_thruster.m`: 示例脚本和曲线绘制。

## 基本用法

```matlab
addpath('model');

params = m080_thruster_params();
state = m080_thruster_model(1700, 24.0, params);

state.thrust_n
state.shaft_torque_nm
state.reaction_torque_nm
```

## 输出字段

- `command`: 归一化油门，范围 [-1, 1]。
- `thrust_n`: 轴向推力，单位 N。
- `shaft_torque_nm`: 桨轴输出扭矩，单位 N*m。
- `reaction_torque_nm`: 载体受到的反扭矩，单位 N*m。
- `electrical_power_w`: 估计输入电功率，单位 W。
- `shaft_power_w`: 估计轴功率，单位 W。
- `rpm`: 估计负载转速，单位 rpm。
- `omega_rad_s`: 估计负载角速度，单位 rad/s。

## 建模公式

PWM 死区:

```text
1485 us <= PWM <= 1535 us -> u = 0
```

正向:

```text
u = (PWM - 1535) / (1850 - 1535)
```

反向:

```text
u = -(1485 - PWM) / (1485 - 1150)
```

静水推力:

```text
T = Tmax_forward(V) * u^2,   u > 0
T = -Tmax_reverse(V) * u^2,  u < 0
```

最大推力表:

| 电压 | 正推 | 反推 |
|---:|---:|---:|
| 12 V | 27.46 N | 16.67 N |
| 16 V | 39.23 N | 27.46 N |
| 24 V | 66.69 N | 46.09 N |

功率与扭矩估计:

```text
P_e = P_max * |u|^3 * min((V / 24)^2, 1)
P_shaft = P_e * eta_shaft
rpm = KV * V * speed_load_factor * |u|
Q_shaft = P_shaft / omega
Q_reaction = -Q_shaft
```

默认参数:

```text
P_max = 360 W
KV = 270 rpm/V
eta_shaft = 0.70
speed_load_factor = 0.75
```

## 注意

公开 M080 资料没有完整的推进效率、转速、电流和扭矩曲线，所以:

- 推力模型主要基于公开静推力表，适合作为初始仿真模型。
- 轴扭矩是基于功率和估计转速得到的工程估计值。
- 如果后续有实测数据，建议直接替换 `m080_thruster_params.m` 中的参数或拟合新的曲线。
