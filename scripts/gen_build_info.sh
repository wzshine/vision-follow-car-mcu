#!/bin/sh
# 生成 Core/Inc/build_info.h —— 只在内容真的变了的时候写，否则每次 make
# 都会碰一下 mtime，把整棵依赖树弄脏。
#
# 由 makefile.init 调用(CubeIDE 不会重新生成 makefile.init，那是它留给用户的
# 扩展点)。取不到 git 就写 "unknown" —— 一个陈旧的 hash 比 "unknown" 糟得多，
# 因为它看起来是个答案。
set -u
root=$(cd "$(dirname "$0")/.." && pwd)
out="$root/Core/Inc/build_info.h"

scm=$(cd "$root" && git rev-parse --short=7 HEAD 2>/dev/null || true)
[ -n "${scm:-}" ] || scm="unknown"
if [ -n "$(cd "$root" && git status --porcelain 2>/dev/null || true)" ]; then
  scm="$scm+"
fi

new='/* 自动生成，勿手改。见 scripts/gen_build_info.sh。未纳入版本控制。 */
#ifndef BUILD_INFO_H
#define BUILD_INFO_H
#define FW_SCM "'"$scm"'"
#endif'

old=$(cat "$out" 2>/dev/null || true)
[ "$old" = "$new" ] || printf '%s\n' "$new" > "$out"
