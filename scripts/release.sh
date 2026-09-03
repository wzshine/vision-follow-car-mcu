#!/bin/sh
# 构建 + 打成上位机 DlaStudio 认得的发布目录。（契约 B-2 的另一半。）
#
# 用法:  sh scripts/release.sh
#         DLA_CONFIG=Release sh scripts/release.sh
#
# ⚠⚠ **要发到服务器上给客户下载的，必须是 Release** —— Debug 构建的板子
#   在 `ver` 里报 debug=1，上位机每次烧录都会拦一句，而那句话是对的。
#   ⇒ DLA_CONFIG 一路传到 make_release.py，MANIFEST.txt 里那行 `config:`
#   说的就是这一份到底是哪种构建。
#
# 出 dist/DlaBoard_B-<版本>[-<commit>]/，里面 MANIFEST.txt / .bin / .hex /
# COMMANDS.json。形状照他们 packages/board-tools/src/release.ts 写，
# 每一条的理由在 tools/make_release.py 的开头。
set -eu

root=$(cd "$(dirname "$0")/.." && pwd)

sh "$root/scripts/build.sh"
cfg=${DLA_CONFIG:-Debug}
python "$root/tools/make_release.py" --out dist --config "$cfg" \
  || python3 "$root/tools/make_release.py" --out dist --config "$cfg"
