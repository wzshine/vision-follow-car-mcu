#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""DlaBoard_B 的控制台命令表 —— 生成 COMMANDS.json 和 cmd_hash.h。

用法:
    python tools/gen_cmd_table.py --check            # 只对照 foc_can.c，不写文件
    python tools/gen_cmd_table.py --header Core/Inc/cmd_hash.h
    python tools/gen_cmd_table.py --json dist/COMMANDS.json --version 0.1.0 --scm abc1234

为什么表不在板上
----------------
上位机 DlaStudio 2026-08-29 上线了消费端（他们仓 `packages/board-tools/src/commands.ts`，
提交 3d58715）。形状是他们定的：表**随发布目录发**，不占 flash，
事实源仍然是固件源码（没有人转抄）。

而那个决定制造了它自己要解决的那个问题：**手上这块板和这台机器上的目录可以对不上。**
不是边缘情况，是开发期间的常态 —— 板子报 `scm=4dacba0+`，那个 `+` 的意思是
"工作树有未提交改动"，所以**一个版本号覆盖无数个不同的镜像**。

⇒ 唯一能回答"这张表配不配得上这块板"的是 cmdhash：固件把这张表的 CRC32 编进
`=tlm build cmdhash=`。⚠⚠ **相等才用；缺失或不等 ⇒ 整张表不用，没有部分采纳。**
他们的原话：一张描述别的构建的表比没有表更糟，因为照它画的每个按钮
看起来都和对的一样。

⚠ 没有表的后果不是"少个功能"：他们的规则是**表里没有的命令一律按 motion 处理**。
⇒ 本板每一条命令都会多一道安全勾选，而"一次标定问六遍，把真正要紧的那一步
稀释成六分之一"是驱动板那边记过的教训。

canonical() 为什么不是 JSON
---------------------------
逐字节抄的驱动板 `tools/gen_cmd_table.py`，**必须一模一样** —— 两个仓的工具
要能对同一张表算出同一个数。对格式化过的文档做校验和，格式化器一变，
板子和文件在每一条命令上都一致、指纹却对不上，而上位机会（正确地）
报"这张表不是这块板的"。⇒ 一个字段分隔符、一个行分隔符、按 id 排序，没别的。
"""
import argparse
import io
import json
import os
import re
import sys
import zlib

# ⚠ 中文 Windows 控制台是 GBK，编不出 ⚠/⇒ —— 不先做这一步，
#   下面每一条带符号的消息都会把脚本崩掉。理由见 tools/_console.py。
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _console  # noqa: E402,F401

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
CLI = os.path.join(ROOT, "Core", "Src", "foc_can.c")

SCHEMA = 1

# 上位机认得的三档，多一个字他们那一行就被丢掉（不是给默认值 —— 丢掉的行
# 落到"表里没有的一律按 motion"，那条规则本来就是为它写的）。
DANGER = ("none", "gates", "motion")
NEEDS = ("none", "idle", "bridge", "torque", "commissioned")

# ⚠⚠ needs 这套词是**驱动板的世界**（桥、力矩、已标定），本板一个都不成立。
#    本板真正的前提是"使能开关合 + 挂 D 档"，也就是 drive_block 那张位图，
#    而他们的词表说不出这件事。⇒ 全填 none，并且这是**已知的失真**，
#    已写进给他们的回执，不是忘了填。

# ⚠ id 只增不重用。和 cart_param.c 的参数 id、drive_block.h 的位号同一条规矩：
#   上位机把人话挂在 id 上，重编号会静默换掉一整屏解释。
#
# ⚠ danger 是上位机的**默认值**，不是他们的闸 —— 他们自己的 constraints.yaml
#   说了算，这里只填空白。而空白正是"客户端从没听说过的那条命令"落的地方。
COMMANDS = [
    # id, name,        args,                              scope,   needs,  danger
    # --- 问一句，什么都不改 ---------------------------------------------
    (  1, "FOC?",       "",                               "board", "none", "none"),
    (  2, "FOC HELP",   "",                               "board", "none", "none"),
    (  3, "ver",        "",                               "board", "none", "none"),
    (  4, "VER?",       "",                               "board", "none", "none"),
    (  5, "tlm",        "",                               "board", "none", "none"),
    (  6, "TLM?",       "",                               "board", "none", "none"),
    (  7, "param list", "",                               "board", "none", "none"),
    (  8, "PARAM?",     "",                               "board", "none", "none"),
    (  9, "WHY?",       "",                               "board", "none", "none"),
    ( 10, "ST?",        "",                               "board", "none", "none"),
    ( 11, "FOC ST?",    "",                               "board", "none", "none"),
    ( 12, "RC?",        "",                               "board", "none", "none"),
    ( 13, "IMU?",       "",                               "board", "none", "none"),
    ( 14, "GEAR?",      "",                               "board", "none", "none"),
    ( 15, "BRK?",       "",                               "board", "none", "none"),
    ( 16, "JET?",       "",                               "board", "none", "none"),
    # ⚠ 往 UART5 写一行标记。分级 none：它不改板子的任何状态，
    #   写出去的那行对面认不得就丢掉。台架认口用的。
    ( 17, "JET TX",     "",                               "board", "none", "none"),
    ( 18, "param get",  "name",                           "board", "none", "none"),
    # ⚠ 这两条今天**一定被拒**（整张表 perm=ro，本板没有非易失存储），
    #   但分级按**这个动词的性质**填，不按"今天能不能成功"填：
    #   存储一旦做上来，不会有人回来改这一行。
    #   param set 在这块板上没有桥可关，写入就是行驶请求本身 ⇒ motion。
    ( 19, "param set",  "name int:value",                 "board", "none", "motion"),
    ( 20, "param save", "",                               "board", "none", "gates"),
    ( 21, "fault",      "",                               "board", "none", "none"),
    ( 22, "fault codes","",                               "board", "none", "none"),
    # ⚠ param load 把 EEPROM 里存的值装回运行时 —— 它会改变限速/超时这些
    #   真正被执行的量 ⇒ 和 param set 同级，不是 gates。
    ( 23, "param load", "",                               "board", "none", "motion"),
    # ⚠ estop 分级 none —— 上位机自己的规矩：停止类动作是**把动力拿走**，
    #   在急停这件事上多问一句话是错的时机说错的话。
    ( 24, "estop",      "",                               "board", "none", "none"),
    # ⚠ clear 相反：它**解除**一个安全闩锁 ⇒ gates。
    ( 25, "clear",      "",                               "board", "none", "gates"),
    ( 26, "calib list",   "",                             "board", "none", "none"),
    ( 27, "calib status", "",                             "board", "none", "none"),
    # ⚠ 标定本身不驱动任何东西 —— 它只采样并在内存里记两个端点。
    #   但 pedal_full 那一步要人**踩下油门踏板**，车若恰好使能着挂着档就会走。
    #   ⇒ 逐步的 moves 在 calib list 每一行里报，这里按整个动词取最坏。
    ( 28, "calib start",  "enum:pedal",                   "board", "none", "motion"),
    ( 29, "calib confirm","",                             "board", "none", "motion"),
    # 放弃不写回任何东西 ⇒ 它把动力拿走，不是给出去。
    # ⚠ 这三条的 id 是 39/40/41，不是 30/31/32 —— 后者 2026-08-31 发现已经
    #   给了下面的 BRAKE 1/0/OFF（第一个 commit 就在了）。按本文件顶上那条
    #   “id 只增不重用”，先拿到的那三条留着，后来的这三条换号。
    #   ⇒ 号在这一段里看着是跳的，而那正是那条规矩的样子：
    #   排序好看不是它的目标，一个 id 永远指同一条命令才是。
    ( 39, "calib abort",  "",                             "board", "none", "gates"),
    # ⚠ none：本板的自检只读已有状态 + 对一块隔离的 EEPROM 区读写，
    #   不重配任何正在服役的外设，也不驱动任何东西。
    #   （会动的那两项 —— CAN loopback —— 在正式固件里是 skip。）
    ( 40, "selftest run", "enum:all?",                    "board", "none", "none"),
    # ⚠ 同一件事的两种写法都得列 —— 分派认哪个，表就得有哪个。
    #   这一条是那个双向对照抓出来的（我只写了上面一行）。
    ( 41, "selftest run all", "",                          "board", "none", "none"),
    # ⚠ id 只增不重用，而 1..41 都用过了 ⇒ 42。DBIS §5.1 的身份行，只读。
    ( 42, "id",         "",                               "board", "none", "none"),

    # --- 会动车，或者会让别的东西动车 -----------------------------------
    #
    # ⚠⚠ BRAKE 1 标 motion 而不是 gates，理由是**刹车电缸的极性还没验过**
    #    （缸 2026-08-28 第一次真接上）。如果 ph_apply 反了，"刹车"这条命令
    #    去做的是**松刹车** —— 一台停在坡上的车会走。
    #    ⇒ 在那一天验完之前，这两条都按最坏的那种读法分级。
    #    这正是上位机自己写下的那个方向：**弄错的代价，一边是多问一次，
    #    另一边是别人的手。**
    ( 30, "BRAKE 1",    "",                               "board", "none", "motion"),
    ( 31, "BRAKE 0",    "",                               "board", "none", "motion"),
    # 撤掉 CLI 闩锁并发 0 —— 就是松开。
    ( 32, "BRAKE OFF",  "",                               "board", "none", "motion"),
    # ⚠ FOC OFF 看着像"关掉"，它做的是**退位**：CLI 不再压着指令，
    #   车回到踏板/遥控的正常仲裁。⇒ 落在他们定义的后半句
    #   "or can let something else move it"。
    ( 33, "FOC OFF",    "",                               "board", "none", "motion"),
    # ⚠ FOC STOP 相反：闩锁**留着**，指令压成停。是把动力拿走，不是给出去。
    ( 34, "FOC STOP",   "",                               "board", "none", "gates"),
    ( 35, "FOC D",      "int:percent int:steer_percent?", "board", "none", "motion"),
    ( 36, "FOC R",      "int:percent int:steer_percent?", "board", "none", "motion"),
    ( 37, "FOC RAW",    "int:permille int:steer int:flags int:gear int:timeout10ms?",
                                                          "board", "none", "motion"),
    # ⚠ 灌帧：把八个字节喂进 0x211 的处理路径，就当它是从总线上来的。
    #    分级 motion 而不是 none —— 它自己什么都不驱动，但它能让本板
    #    **相信对端是活的、是信我们的**，而那些正是 arm 判据读的量。
    #    ⇒ 落在"or can let something else move it"那半句上。
    ( 38, "FOC RX211",  "hex:b0 hex:b1 hex:b2 hex:b3 hex:b4 hex:b5 hex:b6 hex:b7",
                                                          "board", "none", "motion"),
]


def rows():
    return [
        {"id": i, "name": n, "args": a, "scope": s, "needs": d, "danger": g}
        for (i, n, a, s, d, g) in COMMANDS
    ]


def canonical(rs):
    """校验和取在这串字节上。逐字节同驱动板 —— 不要"顺手改整齐"。"""
    out = []
    for r in sorted(rs, key=lambda r: r["id"]):
        out.append("%d\x1f%s\x1f%s\x1f%s\x1f%s\x1f%s"
                   % (r["id"], r["name"], r["args"], r["scope"],
                      r["needs"], r["danger"]))
    return ("\x1e".join(out)).encode("utf-8")


def cmdhash(rs):
    return zlib.crc32(canonical(rs)) & 0xFFFFFFFF


def dispatch_names():
    """foc_can.c 真正应答的每一条命令。

    ⚠ 这一半**不能猜** —— 它是上面那张声明表要被对照的地面真相。
    本板的分派写法和驱动板不同（那边是 str_eq(argv[0], "x")，这边整行匹配）：
        strcmp(line, "X") == 0          整行
        strncmp(line, "X ", n) == 0     前缀带参数 ⇒ 名字去掉尾空格
    ⚠ line[0]=='M'/'S' 那两支不是命令，是 Jetson 回传的 MAP,/SAVE, 状态行。
    """
    src = io.open(CLI, encoding="utf-8").read()
    names = set(re.findall(r'strcmp\(line,\s*"([^"]+)"\)\s*==\s*0', src))
    for lit in re.findall(r'strncmp\(line,\s*"([^"]+)"\s*,\s*\d+\)\s*==\s*0', src):
        names.add(lit.rstrip(" "))
    return names


def duplicate_ids(rs):
    """id -> [name, ...]，只留被用了不止一次的那些。"""
    by = {}
    for r in rs:
        by.setdefault(r["id"], []).append(r["name"])
    return dict((i, ns) for (i, ns) in sorted(by.items()) if len(ns) > 1)


def check(verbose=True):
    real = dispatch_names()
    declared = set(r["name"] for r in rows())
    ok = True

    # ⚠⚠ id 被用了两次 —— 文件顶上那条“id 只增不重用”的规矩自己没有人守着。
    dups = duplicate_ids(rows())
    if dups:
        ok = False
        if verbose:
            for i, ns in dups.items():
                print("cmd table: id %d 被用了 %d 次: %s" % (i, len(ns), ", ".join(ns)))
            print("  => 两条后果，第二条更要紧：")
            print("     ① 上位机把人话挂在 id 上（本文件顶上那条规矩）。")
            print("        一个 id 两条命令 ⇒ 写给其中一条的解释会静默地落到另一条上。")
            print("     ② cmdhash 取在 canonical() 上，而它 sorted(key=id)。")
            print("        相同 id 之间的先后全靠**源文件里的行序**定（稳定排序）。")
            print("        ⇒ 把两行调个位置、一条命令都没改，cmdhash 就变了；")
            print("        上位机会（正确地）报“这张表不是这块板的”，整张表不采用，")
            print("        ⇒ %d 条命令**全部**按 motion 处理。" % len(COMMANDS))

    # 同一个名字写两次：dangerOfCommand 按名字 find，只能命中头一条。
    names = {}
    for r in rows():
        names[r["name"]] = names.get(r["name"], 0) + 1
    dupn = sorted(n for (n, c) in names.items() if c > 1)
    if dupn:
        ok = False
        if verbose:
            print("cmd table: 名字重了: %s" % ", ".join(dupn))
            print("  => 上位机按名字查（commands.ts 的 find），只看得见头一条；")
            print("     另一条的分级写了等于没写。")

    missing = sorted(real - declared)
    if missing:
        ok = False
        if verbose:
            print("cmd table: foc_can.c 应答但表里没有: %s" % ", ".join(missing))
            print("  => 上位机会把它们当 motion（安全方向），但这张表在说谎：")
            print("     它看起来是完整的。")

    extra = sorted(declared - real)
    if extra:
        ok = False
        if verbose:
            print("cmd table: 表里有但 foc_can.c 不应答: %s" % ", ".join(extra))
            print("  => 界面上会画出一个按不动的按钮。")

    if ok and verbose:
        print("cmd table: %d 条命令，与 foc_can.c 两个方向都对得上，cmdhash 0x%08x"
              % (len(COMMANDS), cmdhash(rows())))
    return ok


HEADER = """/**
 * @file    cmd_hash.h
 * @brief   由 tools/gen_cmd_table.py 生成 —— 不要手改。
 *
 * 本固件控制台命令表的指纹，登在 `=tlm <ver> build cmdhash=` 里。
 *
 * 表本身**不在板上**，它随镜像一起发（COMMANDS.json，理由见生成器开头）。
 * 这一个数字的作用只有一个：让上位机分得清"那个文件描述的是这块板"
 * 和"那个文件描述的是同一个版本号的另一个构建" —— 而后者不是罕见情况，
 * 一个版本号覆盖它报过的每一棵脏工作树。
 *
 * ! 不是安全措施也不是防护措施。它只回答一个问题 —— 这张表属不属于这块板。
 *   对不上的时候上位机退回它自己的规则：不认识的命令算最危险的那种。
 */
#ifndef CMD_HASH_H
#define CMD_HASH_H

#define FW_CMD_TABLE_HASH   0x%08Xu

#endif /* CMD_HASH_H */
"""


def write_header(path):
    h = cmdhash(rows())
    text = HEADER % h
    old = None
    if os.path.exists(path):
        old = io.open(path, encoding="utf-8").read()
    # 内容没变就不写 —— 否则每次构建都碰一下 mtime，把整棵依赖树弄脏。
    if old != text:
        io.open(path, "w", encoding="utf-8", newline="\n").write(text)
    return h


def write_json(path, version=None, scm=None):
    rs = rows()
    doc = {
        "schema": SCHEMA,
        "firmware": {
            "name": "DlaBoard_B",
            "version": version or "",
            "scm": scm or "",
        },
        # ⚠ 就是固件报在 `=tlm build cmdhash=` 里的那个数。相等才用。
        "cmdhash": cmdhash(rs),
        "danger_levels": list(DANGER),
        "needs_levels": list(NEEDS),
        "commands": sorted(rs, key=lambda r: r["id"]),
    }
    d = os.path.dirname(path)
    if d and not os.path.isdir(d):
        os.makedirs(d)
    io.open(path, "w", encoding="utf-8", newline="\n").write(
        json.dumps(doc, indent=2, ensure_ascii=False, sort_keys=False) + "\n")
    return doc["cmdhash"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--header")
    ap.add_argument("--json")
    ap.add_argument("--version")
    ap.add_argument("--scm")
    a = ap.parse_args()

    ok = check(verbose=True)

    if a.header:
        h = write_header(a.header)
        print("cmd table: 写了 %s (cmdhash 0x%08x)" % (a.header, h))
    if a.json:
        h = write_json(a.json, a.version, a.scm)
        print("cmd table: 写了 %s (%d 条命令, cmdhash 0x%08x)"
              % (a.json, len(COMMANDS), h))
    if not (a.header or a.json or a.check):
        for r in sorted(rows(), key=lambda r: r["id"]):
            print("%3d  %-12s %-8s %s" % (r["id"], r["name"], r["danger"], r["args"]))

    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
