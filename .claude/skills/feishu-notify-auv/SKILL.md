---
name: feishu-notify-auv
version: 1.0.0
description: "向用户发送 AUV 实验/仿真完成通知到飞书。Use when experiment/simulation completes, or user says '发送通知', '通知我', 'notify me'."
---

# 飞书通知 - AUV 项目

向用户的飞书发送实验/仿真完成通知。

## 使用场景

- 仿真运行完成
- 实验结果生成
- 对比分析完成
- 用户明确要求发送通知

## 使用方法

```bash
bash notify_feishu.sh "标题" "内容"
```

## 参数

- **标题**：通知标题（如"仿真完成"、"实验结果"）
- **内容**：通知详细内容（支持多行，可包含关键指标）

## 示例

```bash
# 仿真完成通知
bash notify_feishu.sh "XHY 仿真完成" "SMC+ESO vs SMC 对比完成
航向误差 RMSE: 0.05 rad
横向误差 RMSE: 0.12 m"

# 实验结果通知
bash notify_feishu.sh "实验结果" "三种方法对比完成，图表已生成"
```

## 配置

用户 open_id 已配置在 `notify_feishu.sh` 中：
- `USER_OPEN_ID="ou_7464ad1f38b07ce9f32b39bfcdb5c9dc"`

## 注意事项

- 需要 lark-cli 已配置并认证
- 使用 bot 身份发送消息
- 消息格式为 Markdown
