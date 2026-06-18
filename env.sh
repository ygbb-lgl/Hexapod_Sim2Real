#!/bin/bash
# 激活 conda 环境并注入代理
source /home/hexapod/anaconda3/bin/activate hexapod_lgl
export HTTP_PROXY="http://127.0.0.1:7890"
export HTTPS_PROXY="http://127.0.0.1:7890"
