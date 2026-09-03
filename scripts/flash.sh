#!/bin/sh
# 刷写 DlaBoard_B。（契约 B-2 的最后一块。）
#
# 用法:  sh scripts/flash.sh <口>              # 刷 Debug/ 里刚构建的那份
#        sh scripts/flash.sh <口> <hex 文件>   # 刷指定的
#
# ⚠⚠ 口不要记。COM 号 2026-08-28 全变过一次。
#     用 `python tools/ports.py`（在 FOC_Double_C/）按 MCU UID 认板。
#
# ⚠⚠ 这条命令会把板子踢进 ROM BootLoader。要把它弄回来，**顺序不能反**：
#     复位必须在 RTS 还高的时候断言，松开复位那一刻 RTS 才放低（BOOT0 由那一刻决定）。
#     ⚠ 先把电平恢复成"安全值"再跳 DTR 是**一次复位都不会发生** —— 现象是
#       什么都没变，人会以为板子坏了。2026-08-30 台架实测，两个方向都验过。
#     详见 CONTRACT_串口_控制板_2026-08-30.md §1。
#     ⇒ 对一块装在车上的控制板，这不是一次"重启"，是一次**停发 0x210**：
#       驱动板 300 ms 后超时。刷之前请确认车是停的、使能是断的。
set -eu

root=$(cd "$(dirname "$0")/.." && pwd)

if [ $# -lt 1 ]; then
  echo "用法: sh scripts/flash.sh <口> [hex 文件]" >&2
  echo "口不要记 —— 用 python tools/ports.py（在 FOC_Double_C/）按 UID 认板。" >&2
  exit 2
fi

port=$1
image=${2:-"$root/Debug/DlaBoard_B.hex"}

if [ ! -f "$image" ]; then
  echo "找不到镜像：$image —— 先跑 sh scripts/build.sh" >&2
  exit 1
fi

python "$root/Debug/ISP.py" "$port" "$image" \
  || python3 "$root/Debug/ISP.py" "$port" "$image"
