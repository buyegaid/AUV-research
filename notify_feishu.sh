#!/bin/bash
# 飞书通知脚本 - 向用户发送实验/仿真完成通知

USER_OPEN_ID="ou_7464ad1f38b07ce9f32b39bfcdb5c9dc"

# 参数检查
if [ $# -lt 2 ]; then
    echo "用法: $0 <标题> <内容>"
    echo "示例: $0 '实验完成' '仿真结果：RMSE=0.05'"
    exit 1
fi

TITLE="$1"
CONTENT="$2"

# 构建消息
MESSAGE="🔔 **${TITLE}**

${CONTENT}

---
时间: $(date '+%Y-%m-%d %H:%M:%S')
项目: AUV 抗流控制研究"

# 发送消息
lark-cli im +messages-send \
  --as bot \
  --user-id "${USER_OPEN_ID}" \
  --markdown "${MESSAGE}"

if [ $? -eq 0 ]; then
    echo "✅ 通知发送成功"
else
    echo "❌ 通知发送失败"
    exit 1
fi
