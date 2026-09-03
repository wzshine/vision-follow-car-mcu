# -*- coding: utf-8 -*-
"""把本仓脚本的 stdout/stderr 固定成 UTF-8。

⚠⚠ 这不是"顺手统一一下编码"。中文 Windows 的控制台是 GBK，而 GBK 编不出
`⚠` `⇒` `⭐` 这些字符 —— 本仓的脚本一旦要打印一条带 ⚠ 的话，
Python 抛 UnicodeEncodeError，脚本以非零退出，`set -e` 让整个 build.sh 停下。

⭐ 而要命的是**它们出现在哪些分支里**：

  · gen_cmd_table.py 的 ⚠ 只在「表和 foc_can.c 对不上」的失败分支里，
    成功那一行一个都没有；
  · make_release.py 的 ⚠ 只在「工作树是脏的」那一句里；
  · stamp_image.py 的那一句每次都打（所以这个缺陷 2026-08-31 才暴露出来）。

⇒ 也就是说：**这条链偏偏在有事要报的时候炸**，而炸出来的是一个
UnicodeEncodeError 堆栈，和它本来要说的那件事一个字都对不上。

⚠ errors='replace' 而不是 'strict'：真编不出来的字符打成 `?` 也比中断好 ——
一条被替换掉几个符号的消息仍然在说事，一个堆栈什么都不说。
"""
import sys


def use_utf8():
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except Exception:
            # 早于 3.7 的 Python，或者被重定向成一个不支持 reconfigure 的对象。
            # ⚠ 静默跳过是对的：这一步是让消息能打出来，它自己不该成为新的失败点。
            pass


use_utf8()
