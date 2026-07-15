#!/bin/sh
# ===== TurnSign OCR + Qianfan 环境变量配置 =====
# ar_receiver.py 会在未设置 API Token 时自动读取本文件。
# 该文件含密钥，不要提交到 Git 或发送给他人。
#
# Qianfan API Key（必须！）
# 控制台：https://console.bce.baidu.com/iam/#/iam/apikey/list
export AR_TURNSIGN_ACCESS_TOKEN="bce-v3/ALTAK-y1n9JwdvcvztnoOjNTTcQ/42a5d0861205f35a0362d3a816efcc92df72acd1"
export AR_TURNSIGN_API_BASE_URL="https://qianfan.baidubce.com/v2"
export AR_TURNSIGN_MODEL="ernie-4.5-turbo-32k"

# 关闭 OCR（可选，默认开启）
# export AR_TURNSIGN_OCR_ENABLED=0

if [ -n "${AR_TURNSIGN_ACCESS_TOKEN:-}" ]; then
    echo "OCR env loaded: Qianfan API token is set"
else
    echo "OCR env warning: API token is not set; OCR works but API interpretation is disabled"
fi
