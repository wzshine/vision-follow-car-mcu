#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把已经构建好的产物，打成上位机认得的发布目录。

用法:  python tools/make_release.py [--out dist] [--config Debug|Release]

产出 `dist/DlaBoard_B-<版本>[-<commit>]/`：
    MANIFEST.txt      名字、镜像、地址、crc32
    DlaBoard_B.bin    上位机的 ISP 烧写器写的是这个
    DlaBoard_B.hex    Debug/ISP.py 用的是这个
    COMMANDS.json     命令表（见 gen_cmd_table.py）

形状不是我定的，是照上位机 `packages/board-tools/src/release.ts` 写的：

  * 目录名必须能被 `^(?:.*[-_])?v?(\\d+\\.\\d+\\.\\d+)(?:[-_]([0-9a-f]{7,40}))?$`
    匹配上，否则那个目录**根本不会被当成一个发布**。
  * `name:` 那一行是他们判断"这个发布是给哪块板的"的唯一来源 ——
    ⚠ 他们**不从文件夹名读**，理由是文件夹名谁都能改。
    而"说不出自己是给哪块板的发布，绝不会被拿来烧任何一块板"。
  * 镜像行只认 `loader:` / `slot A:` / `slot B:` 三种前缀。

⚠⚠ 两件必须说清楚的：

1. **本板没有 A/B 槽，也没有 bootloader。** 一块单镜像的板，
   写成 `slot A:` 是因为**他们的词表里只有这三个词**，不是因为这块板有槽。
   ⇒ `load` 是 0x08000000（复位向量本身），不是某个 loader 之后的偏移。

2. **crc32 写的是固件会算的那个数，不是文件的 CRC32。**
   他们文件开头那句警告："清单里的 crc32 不是文件的 crc32，是固件自己算的
   那个数（整个镜像，把它自己的校验和字段读成 0）"。
   ⚠⚠ 这一段以前写的是"本板没有头，两边算的是同一个数"，并留了一句
   "哪天本板做了盖章，这里要跟着改"。**盖章 2026-08-30 就做了（bc666c2），
   这里没跟着改** ⇒ 那之后打出来的每一个发布包，MANIFEST 里的 crc32
   都和上位机 imageCrc32() 算的对不上。
   ⇒ fw_flash_plan 在**动板子之前**就会拦下来（flash.ts:81 和 tools.ts:1825
     各查一遍），报「和发布清单对不上」。全流程的最后一步走不完。
   ⇒ 一条留在注释里的"以后要改"，和没写没有区别。2026-08-31 修，
     而且这一次它有守卫：打完包当场按上位机的规则复算一遍再断言。
"""
import argparse
import io
import os
import re
import shutil
import subprocess
import sys
import zlib

# ⚠ 中文 Windows 控制台是 GBK，编不出 ⚠/⇒ —— 不先做这一步，
#   下面每一条带符号的消息都会把脚本崩掉。理由见 tools/_console.py。
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _console  # noqa: E402,F401

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

sys.path.insert(0, HERE)
import gen_cmd_table  # noqa: E402


# 镜像自述头的位置和魔数 —— 和 tools/stamp_image.py、Core/Inc/fw_image.h、
# 以及上位机 packages/board-tools/src/image.ts 是同一份。
HEADER_OFFSET = 0x200
OFF_CRC = 12
MAGIC = bytes([0x4F, 0x43, 0x49, 0x4D, 0x21, 0x47, 0x30, 0x31])


def image_crc32(data):
    """固件会算的那个数：整段镜像，**它自己的校验和字段读作 0**。

    ⚠ 逐条照上位机 image.ts 的 imageCrc32() 写，包括那条退化规则：
      认不出魔数（= 这个镜像没有自述头）就退回整文件 CRC32。
      ⇒ 本板 2026-08-30 之后有头，走的是上面那条；退化那条留着，
        是因为**两边必须在"没有头"这件事上也一致**。
    """
    at = HEADER_OFFSET
    if len(data) < at + 16 or bytes(data[at:at + 8]) != MAGIC:
        return zlib.crc32(data) & 0xFFFFFFFF
    c = bytearray(data)
    c[at + OFF_CRC:at + OFF_CRC + 4] = bytes(4)
    return zlib.crc32(bytes(c)) & 0xFFFFFFFF


def fw_version():
    """版本号从 fw_version.h 读，不另存一份。"""
    src = io.open(os.path.join(ROOT, "Core", "Inc", "fw_version.h"),
                  encoding="utf-8").read()
    got = {}
    for part in ("MAJOR", "MINOR", "PATCH"):
        m = re.search(r"#define\s+FW_VER_%s\s+(\d+)U" % part, src)
        if not m:
            raise SystemExit("读不出 FW_VER_%s" % part)
        got[part] = m.group(1)
    return "%s.%s.%s" % (got["MAJOR"], got["MINOR"], got["PATCH"])


def scm():
    """git 短 hash + 脏标记，和 scripts/gen_build_info.sh 同一口径。"""
    try:
        h = subprocess.check_output(
            ["git", "rev-parse", "--short=7", "HEAD"],
            cwd=ROOT, stderr=subprocess.DEVNULL).decode().strip()
    except Exception:
        return "unknown", False
    try:
        dirty = subprocess.check_output(
            ["git", "status", "--porcelain"],
            cwd=ROOT, stderr=subprocess.DEVNULL).decode().strip() != ""
    except Exception:
        dirty = True
    return h, dirty


def verify(out):
    """把刚打出来的目录当成一个陌生的发布包，重新验一遍。

    ⚠⚠ 验的是**产物里的那一份**，不是上面几个变量。这两件事分开过一次就够记住：
    MANIFEST 里的 crc32 用错了算法（整文件 vs 镜像自述），而脚本一路打印
    "发布目录: ..." 全绿 —— 错的那个数从生成到落盘没有任何一步会不同意。

    两条，都照上位机的规则：
      · MANIFEST 的镜像行（release.ts 的正则）→ 文件名、bytes、crc32
        再和 image_crc32() 复算的比（flash.ts:81 / tools.ts:1825 就是这么比的）。
      · SHA256SUMS 里每一条和文件本身的摘要（fw_fetch 下完之后这么验）。
    """
    import hashlib
    man = io.open(os.path.join(out, "MANIFEST.txt"), encoding="utf-8").read()
    # ⚠ 逐字照 release.ts 的 LINE 正则 —— 差一点，那一行在他们那儿就不是镜像行。
    rows = re.findall(
        r"^\s*(?:loader|slot [AB]):\s+(\S+)\s+(\d+) bytes,\s*crc32\s+([0-9a-f]+),"
        r"\s*load\s+(0x[0-9a-fA-F]+)", man, re.M)
    if not rows:
        raise SystemExit("MANIFEST 里一行镜像都解析不出来 —— 上位机会当它没有镜像。")
    for (name, nbytes, chex, _load) in rows:
        data = io.open(os.path.join(out, name), "rb").read()
        if len(data) != int(nbytes):
            raise SystemExit("%s: 清单说 %s 字节，文件是 %d" % (name, nbytes, len(data)))
        got = image_crc32(data)
        if got != int(chex, 16):
            raise SystemExit(
                "%s: 清单 crc32 %s，按上位机的算法复算是 %08x —— 烧录会在动板子"
                "之前被拦下。" % (name, chex, got))

    sums = io.open(os.path.join(out, "SHA256SUMS"), encoding="utf-8").read()
    listed = dict((m.group(2), m.group(1)) for m in
                  re.finditer(r"^([0-9a-f]{64})\s+\*?(.+?)\s*$", sums, re.M))
    for name in sorted(os.listdir(out)):
        if name in ("SHA256SUMS", "MANIFEST.txt"):
            continue
        if name not in listed:
            raise SystemExit("SHA256SUMS 里没有 %s —— fw_fetch 会整包丢弃。" % name)
        digest = hashlib.sha256(io.open(os.path.join(out, name), "rb").read()).hexdigest()
        if digest != listed[name]:
            raise SystemExit("%s 的 sha256 和 SHA256SUMS 对不上。" % name)

    return len(rows), len(listed)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="dist")
    # ⚠ 取产物的目录就是构建配置名（build.sh 的 DLA_CONFIG）。
    #   两个目录里可以同时躺着两份 .bin，而它们的文件名一模一样。
    ap.add_argument("--config", default="Debug", choices=("Debug", "Release"))
    a = ap.parse_args()

    # ⚠ 表和固件对不上就不发。这一步在动任何文件之前 ——
    #   一个发布目录里躺着一张描述别的构建的命令表，正是上位机
    #   花了一整个提交去防的那件事。
    if not gen_cmd_table.check(verbose=True):
        raise SystemExit("命令表和 foc_can.c 对不上 —— 不发布。")

    version = fw_version()
    commit, dirty = scm()

    # ⚠⚠ 脏工作树**不把 commit 写进目录名**。
    #    他们的目录名正则只收纯十六进制，而 `4dacba0+` 里的那个 `+` 的意思正是
    #    "这个标识对应的不是一个确定的镜像"。把 `+` 抹掉塞进去，就是拿一个
    #    看起来精确的名字去描述一份说不清是什么的东西 ——
    #    **一个陈旧的 hash 比 unknown 糟得多，因为它看起来是个答案。**
    #    ⇒ 脏树发出来的目录只带版本号，认板靠 cmdhash。
    if dirty or commit == "unknown":
        dirname = "DlaBoard_B-%s" % version
        print("⚠ 工作树是脏的（或取不到 git）⇒ 目录名不带 commit：%s" % dirname)
        print("  认这份发布配不配得上手上那块板，靠的是 cmdhash，不是目录名。")
    else:
        dirname = "DlaBoard_B-%s-%s" % (version, commit)

    out = os.path.join(ROOT, a.out, dirname)
    if not os.path.isdir(out):
        os.makedirs(out)

    built = os.path.join(ROOT, a.config)
    binf = os.path.join(built, "DlaBoard_B.bin")
    hexf = os.path.join(built, "DlaBoard_B.hex")
    for f in (binf, hexf):
        if not os.path.exists(f):
            raise SystemExit("缺 %s —— 先跑 DLA_CONFIG=%s sh scripts/build.sh"
                             % (f, a.config))

    data = io.open(binf, "rb").read()
    crc = image_crc32(data)
    if crc == (zlib.crc32(data) & 0xFFFFFFFF):
        # 两个数相等只有一个原因：这个镜像没有自述头（没盖过章）。
        # 那本身不是错误，但发布包不该在没人察觉的情况下退回那条路。
        print("⚠ 这份镜像没有自述头（没盖过章）⇒ crc32 退回了整文件 CRC32。")
        print("  scripts/build.sh 会盖章；单独跑 make 不会。")

    shutil.copy2(binf, os.path.join(out, "DlaBoard_B.bin"))
    shutil.copy2(hexf, os.path.join(out, "DlaBoard_B.hex"))

    h = gen_cmd_table.write_json(
        os.path.join(out, "COMMANDS.json"),
        version=version,
        scm=commit + ("+" if dirty else ""))

    manifest = (
        "name: DlaBoard_B\n"
        "version: %s\n"
        "scm: %s%s\n"
        # ⭐ 上位机今天不读这一行（它只认 name/version/scm 和镜像行）。
        # 写在这里是因为发布目录应该能**自己说出**它是哪种构建：
        # 以前这件事只存在于服务器索引里一个手写的 "config" 字段，
        # 而那份索引和它描述的字节之间没有任何东西守着。
        "config: %s\n"
        "\n"
        "# ⚠ 本板没有 bootloader，也没有 A/B 槽 —— 只有一个镜像，从 0x08000000\n"
        "#   开始。写成 `slot A:` 是因为上位机 release.ts 的镜像行只认\n"
        "#   loader / slot A / slot B 这三个词。\n"
        "# ⚠ 也没有镜像盖章，所以这里的 crc32 是文件的普通 CRC32。\n"
        "#   他们的 imageCrc32() 在认不出头部 magic 时退化成同一个算法，\n"
        "#   两边对得上 —— 但理由是本板没有头，不是那条规则不适用。\n"
        "slot A: DlaBoard_B.bin %d bytes, crc32 %08x, load 0x08000000\n"
    ) % (version, commit, "+" if dirty else "", a.config, len(data), crc)

    io.open(os.path.join(out, "MANIFEST.txt"), "w",
            encoding="utf-8", newline="\n").write(manifest)

    # ---- SHA256SUMS ----
    #
    # 上位机的 `fw_fetch` 从服务器下完之后，**用发布包自己的 SHA256SUMS 逐个校验**，
    # 全过才放进本机固件目录；包里没有这个文件它会整包丢弃
    # （"没有办法证明这些字节是对的"）。
    # ⚠ 它跳过 SHA256SUMS 和 MANIFEST.txt 自己，其余每个文件都必须在里面 ——
    #   少一个就报「SHA256SUMS 里没有它」。
    # ⚠ 格式是 `<sha256> *<文件名>`（星号是二进制模式标记），照驱动板的发布包写。
    import hashlib
    sums = []
    for name in sorted(os.listdir(out)):
        if name in ("SHA256SUMS", "MANIFEST.txt"):
            continue
        # ⚠ 变量名不叫 h —— 上面 cmdhash 用的就是 h，重名会把它盖掉，
        #   而后面那行 print 才会报错（当场就抓到了，但换个位置就未必）。
        digest = hashlib.sha256(io.open(os.path.join(out, name), "rb").read()).hexdigest()
        sums.append("%s *%s" % (digest, name))
    io.open(os.path.join(out, "SHA256SUMS"), "w",
            encoding="utf-8", newline="\n").write("\n".join(sums) + "\n")

    # ⭐ 复验，在报告任何一个数之前。
    n_img, n_sum = verify(out)

    print("发布目录: %s（%s 构建）" % (out, a.config))
    print("  DlaBoard_B.bin  %d bytes  crc32 %08x" % (len(data), crc))
    print("  COMMANDS.json   %d 条命令  cmdhash %u (0x%08x)"
          % (len(gen_cmd_table.COMMANDS), h, h))
    print("  SHA256SUMS      %d 个文件" % len(sums))
    print("  ✔ 复验：%d 个镜像的 crc32 按上位机的算法复算一致，%d 条 sha256 逐个对上"
          % (n_img, n_sum))
    print()
    print("⚠ cmdhash 必须和板子 `tlm` 里 build 记录报的那个数相等，")
    print("  否则上位机整张表不用（没有部分采纳）。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
