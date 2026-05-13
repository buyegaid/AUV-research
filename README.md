# XHY AUV 三维轨迹跟踪仿真

## 目标 (Goals)

1. 基于6-DOF动力学模型，推导并实现SMC（滑模控制）控制律
2. 使用扩展状态观测器(ESO)估计外部扰动，补偿SMC控制
3. 实现3D ALOS制导 + SMC + ESO的完整闭环轨迹跟踪
4. 验证在海流扰动下的跟踪性能

## 系统架构

```
3D ALOS制导 → 期望姿态/速度 → SMC控制律 → 推力分配 → 5推进器AUV
                                  ↑
                              ESO扰动估计
```

## 模型参数

- 质量: m = 33 kg
- 长度: L = 1.242 m, 直径: D = 0.24 m
- 5推进器: 1主推 + 2垂直 + 2侧向
- 附加质量: diag([11.28, 132.11, 47.10, 0.006, 0.043, 0.138])

## 控制通道

| 通道 | 控制方法 | 执行器 |
|------|---------|--------|
| 纵荡(surge) | SMC + ESO | 主推进器 |
| 俯仰(pitch) | SMC + ESO | 垂直推进器 x2 |
| 偏航(yaw) | SMC + ESO | 侧向推进器 x2 |

## 文件结构

- `model/xhy.m` — 6-DOF动力学模型
- `controller/smc_6dof.m` — 滑模控制器
- `vec_leso_update_adv.m` — 扩展状态观测器
- `main_loop.m` — 主仿真循环
- `heading/lib/my_ALOS3D.m` — 3D ALOS制导算法
- `heading/lib/get_params.m` — 参数配置

## 当前进度

- [x] 动力学模型搭建 (xhy.m)
- [x] SMC控制律框架 (smc_6dof.m)
- [x] ESO观测器实现 (vec_leso_update_adv.m)
- [x] 3D ALOS制导
- [ ] 主循环中接入SMC+ESO闭环控制
- [ ] 推力分配逆映射 (tau → RPM)
- [ ] 海流扰动下仿真验证
- [ ] 参数调优
