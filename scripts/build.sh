#!/bin/sh
# 构建 DlaBoard_B。（上位机契约 B-2 的一半 —— 在这之前只有 IDE 手动流程。）
#
# 用法:  sh scripts/build.sh [clean]
#         DLA_CONFIG=Release sh scripts/build.sh [clean]
#
# ⚠ 工具链路径可以用环境变量盖掉，不要改这个文件：
#     DLA_TOOLCHAIN_BIN=... DLA_MAKE_BIN=... sh scripts/build.sh
#
# ⚠ 默认出的是 **Debug** 构建。板子会在 `ver` 和 `=tlm build` 里
#   如实说 debug=1 —— 那不是装饰：上位机看到 debug=1 会警告，
#   而它**永远不会**把"没说"读成 Release。
#
# ⭐ DLA_CONFIG=Release 出不定义 DEBUG 的 -Os 构建（板子报 debug=0）。
#   Release/ 的构建文件是从 Debug/ 派生的，每次重新派生 ——
#   理由写在 scripts/gen_release_makefiles.sh 开头。
#   ⚠⚠ **发给客户的那一份要是 Release**：一块报 debug=1 的板子，
#   上位机每次烧录都会警告一句，而那句话是对的。
set -eu

root=$(cd "$(dirname "$0")/.." && pwd)

TC=${DLA_TOOLCHAIN_BIN:-"/c/ST/STM32CubeIDE_1.19.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.win32_1.0.0.202411081344/tools/bin"}
MK=${DLA_MAKE_BIN:-"/c/ST/STM32CubeIDE_1.19.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.make.win32_2.2.0.202409170845/tools/bin"}

if [ ! -x "$TC/arm-none-eabi-gcc.exe" ] && [ ! -x "$TC/arm-none-eabi-gcc" ]; then
  echo "找不到工具链：$TC" >&2
  echo "用 DLA_TOOLCHAIN_BIN=... 指一个，或装 STM32CubeIDE 1.19。" >&2
  exit 1
fi

PATH="$TC:$MK:$PATH"
export PATH

# 这两步 Debug/makefile 自己也会做（-include ../makefile.init），
# 这里显式再跑一次，是为了让**不经过 make 的调用者**也拿到新的标识。
sh "$root/scripts/gen_build_info.sh"
python "$root/tools/gen_cmd_table.py" --header "$root/Core/Inc/cmd_hash.h" \
  || python3 "$root/tools/gen_cmd_table.py" --header "$root/Core/Inc/cmd_hash.h"

cfg=${DLA_CONFIG:-Debug}
case "$cfg" in
  Debug) ;;
  Release) sh "$root/scripts/gen_release_makefiles.sh" ;;
  *) echo "DLA_CONFIG 只能是 Debug 或 Release（给的是：$cfg）" >&2; exit 1 ;;
esac

cd "$root/$cfg"
if [ "${1:-}" = "clean" ]; then
  make clean
fi
make all

# ⭐ 盖章：把长度和 CRC32 写进镜像自己的头（见 tools/stamp_image.py）。
# ⚠ 必须**在链接之后** —— 那两个字段编译期算不出来。
# ⚠ .hex 也要重写：ISP.py 烧的是 .hex，只盖 .bin 的章等于没盖。
python "$root/tools/stamp_image.py" "$root/$cfg/DlaBoard_B.bin" "$root/$cfg/DlaBoard_B.hex"   || python3 "$root/tools/stamp_image.py" "$root/$cfg/DlaBoard_B.bin" "$root/$cfg/DlaBoard_B.hex"

echo
echo "产物在 $root/$cfg/：DlaBoard_B.elf / .hex / .bin"
echo "要打成上位机认得的发布目录：sh scripts/release.sh"
