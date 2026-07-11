#!/bin/sh
# ===== TurnSign OCR + AI Studio 环境变量配置 =====
# ar_receiver.py 会在未设置 AISTUDIO_ACCESS_TOKEN 时自动读取本文件。
# 该文件含密钥，不要提交到 Git 或发送给他人。
#
# AI Studio access token（必须！）
# 获取地址：https://aistudio.baidu.com/account/accessToken
export AISTUDIO_ACCESS_TOKEN="c3da7b3b23ecb2719fd47c66cff8aa63833ab703"

# 关闭 OCR（可选，默认开启）
# export AR_TURNSIGN_OCR_ENABLED=0

if [ -n "${AISTUDIO_ACCESS_TOKEN:-}" ]; then
    echo "OCR env loaded: AISTUDIO_ACCESS_TOKEN is set"
else
    echo "OCR env warning: AISTUDIO_ACCESS_TOKEN is not set; OCR works but API interpretation is disabled"
fi
