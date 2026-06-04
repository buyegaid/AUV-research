# M060 推进器 MATLAB 仿真模型

本目录提供 M060 深水推进器的准静态模型，用于 AUV/ROV 动力学仿真。

## 文件

- `m060_thruster_model.m`: 主模型函数，输入 PWM 和电压，输出推力、扭矩、功率、转速。
- `m060_thruster_params.m`: 默认参数。
- `demo_m060_thruster.m`: 示例脚本和曲线绘制。

## 参数来源

FullDepth M060 公开参数给出:

- 工作电压: 12-24 V
- 最大功率: 150 W
- 最大电流: 6 A
- 最大工作深度: 200 m
- 12 V 正推/反推: 1.0/0.7 kgf
- 16 V 正推/反推: 1.5/1.1 kgf
- 24 V 正推/反推: 3.0/2.2 kgf

FreeBoxHobby M060 页面给出的补充参数:

- KV: 350 rpm/V
- 额定功率: 140 W
- 最大电流: 6 A
- 24 V 正推/反推: 3.46/2.61 kgf
- 空气中重量: 约 200 g
- 长度: 约 90 mm
- 导流罩直径: 约 69 mm

默认模型采用 FullDepth 推力表作为静推力基准，采用 350 KV 和 150 W 估算轴扭矩。

## 基本用法

```matlab
addpath('model');

params = m060_thruster_params();
state = m060_thruster_model(1700, 24.0, params);

state.thrust_n
state.shaft_torque_nm
state.reaction_torque_nm
```

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
| 12 V | 9.81 N | 6.86 N |
| 16 V | 14.71 N | 10.79 N |
| 24 V | 29.42 N | 21.57 N |

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
P_max = 150 W
KV = 350 rpm/V
eta_shaft = 0.70
speed_load_factor = 0.75
```

## 注意

公开 M060 资料没有完整推进效率、转速、电流和扭矩曲线，所以:

- 推力模型主要基于公开静推力表，适合作为初始仿真模型。
- 轴扭矩是基于功率和估计转速得到的工程估计值。
- 如果后续有实测数据，建议直接替换 `m060_thruster_params.m` 中的参数或拟合新的曲线。

## 资料链接

- FullDepth M060: https://fulldepthrov.en.made-in-china.com/product/lfbYRdhrHScG/China-Compact-M060-Thruster-for-Underwater-Rov-and-Marine-Tasks.html
- FreeBoxHobby M060: https://freeboxhobby.com/product/rov-m060-underwater-thruster-brushless-motor/
