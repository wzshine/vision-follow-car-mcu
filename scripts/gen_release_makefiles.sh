#!/bin/sh
# 从 CubeIDE 生成的 Debug/ 派生出 Release/ 的构建文件。
#
# 为什么是派生，不是手写一份
# --------------------------
# Debug/*.mk 是 CubeIDE 按 .cproject 生成的（每个文件头上都写着
# "Automatically-generated file. Do not edit!"），在 IDE 里加一个源文件，
# 它自己会重写。Release/ 这份要是手写的、独立的，加源文件的那天它不会跟着变 ——
# ⚠⚠ 而后果**不是编译失败**，是少编一个文件然后照样链接成功、照样能烧进板子。
# ⇒ 每次都从 Debug/ 重新派生，不缓存、不增量。Debug/ 是唯一的事实源。
#
# 三个差别，2026-08-31 对着 .cproject 的 exe.release 配置抄的
# ---------------------------------------------------------
#   -O0     -> -Os     optimization.level.value.os
#   -g3     -> -g0     debuglevel.value.g0（汇编和 C 都是）
#   -DDEBUG -> 去掉    Release 的定义符号只有 USE_HAL_DRIVER / STM32F407xx
#
# ⚠ -DDEBUG 不是装饰：fw_version.h 的 FW_IS_DEBUG 读的就是它，板子在
#   `ver` 和 `=tlm build` 里报的 `debug=` 是它。上位机看到 debug=1 会警告，
#   而它**永远不会**把"没说"读成 Release。⇒ 这一版发出去之前，那个 0
#   要在**真板上**问出来，不是在这里推断出来。
#
# ⚠ 有人在 CubeIDE 里真构建一次 Release，CubeIDE 会用它自己生成的文件盖掉
#   这里的 —— 那是对的，两边的来源都是 .cproject。这个脚本每次跑又会盖回去。
#   两份必须一致；不一致说明这里抄错了 .cproject，去改这个脚本，不要改 Release/。
set -eu

root=$(cd "$(dirname "$0")/.." && pwd)
src="$root/Debug"
dst="$root/Release"

[ -d "$src" ] || { echo "没有 Debug/ —— 派生不出 Release/。" >&2; exit 1; }

# 只重建构建文件，**不删已有的 .o** —— 删了每次都是全量重编。
mkdir -p "$dst"
( cd "$src" && find . -name '*.mk' -o -name 'makefile' -o -name 'objects.list' ) | while read -r f; do
  mkdir -p "$dst/$(dirname "$f")"
  sed -e 's/-g3 -DDEBUG/-g0/g' -e 's/-O0 /-Os /g' "$src/$f" > "$dst/$f"
done

# 自检：抄错一处，这里就该红。
bad=0
if grep -rq -- '-DDEBUG' "$dst"; then
  echo "Release/ 里还有 -DDEBUG —— 板子会报 debug=1。" >&2; bad=1
fi
if grep -rq -- '-O0 ' "$dst"; then
  echo "Release/ 里还有 -O0 —— 那不是 Release。" >&2; bad=1
fi
if grep -rq -- '-g3' "$dst"; then
  echo "Release/ 里还有 -g3。" >&2; bad=1
fi
# 行数对不上 = sed 把整行吃了或者漏了一个子目录，比编译错更难发现。
a=$(grep -rc 'arm-none-eabi-gcc' "$src" --include='*.mk' | awk -F: '{s+=$2} END {print s}')
b=$(grep -rc 'arm-none-eabi-gcc' "$dst" --include='*.mk' | awk -F: '{s+=$2} END {print s}')
if [ "$a" != "$b" ]; then
  echo "编译规则条数对不上：Debug $a / Release $b —— 少编的文件照样能链接成功。" >&2; bad=1
fi
[ "$bad" = 0 ] || exit 1

echo "Release/ 已从 Debug/ 派生（-Os -g0，无 DEBUG；$b 条编译规则）"
