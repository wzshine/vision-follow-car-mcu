#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""给构建出来的镜像盖章：把长度和 CRC32 写进它自己的头。

用法:  python tools/stamp_image.py Debug/DlaBoard_B.bin [Debug/DlaBoard_B.hex]

为什么要有这一步
----------------
镜像头里的版本、Debug 标志、commit 都是编译期就知道的，**长度和 CRC 不是** ——
长度要等链接完，CRC 要等长度。⇒ 只能在构建之后往回写，这一步就叫"盖章"。

⚠ 没盖章的镜像那两个字段是 `0xFFFFFFFF`，固件如实报 `crc unstamped`。
  **"没有这个信息"和"这个信息是好的"必须是两个不同的答案** ——
  所以未盖章不是 0（0 是一个合法的长度，也是一个合法的 CRC）。

布局是上位机已经在解析的那一份
------------------------------
判据：`DlaStudio/packages/board-tools/src/image.ts`
  HEADER_OFFSET=0x200  OFF_BYTES=8  OFF_CRC=12  OFF_VER=16  OFF_FLAGS=19
  OFF_SCM=20 (12 字节)  MAGIC=4F 43 49 4D 21 47 30 31
⚠ 差一个字节，他们的 `hasMagic()` 就认不出这是个头，`imageCrc32()` 退化成
  整文件 CRC32，和我们盖的章对不上 —— 而那看起来像"镜像坏了"。

⚠⚠ CRC 覆盖整段镜像，**但 crc 字段本身读作 0** —— 校验和不能覆盖它自己。
   固件侧 fw_image.c 的 crc32_run() 用同一条规则跳过那四个字节。
"""
import os
import sys
import zlib

# ⚠ 见 tools/_console.py：不先固定成 UTF-8，下面那句带 ⚠ 的话
#   会在中文 Windows 控制台上把这个脚本崩掉 —— 而它每次构建都打。
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _console  # noqa: E402,F401

HEADER_OFFSET = 0x200
OFF_BYTES = 8
OFF_CRC = 12
MAGIC = bytes([0x4F, 0x43, 0x49, 0x4D, 0x21, 0x47, 0x30, 0x31])
UNSTAMPED = 0xFFFFFFFF


def stamp(data: bytearray) -> tuple:
    at = HEADER_OFFSET
    if len(data) < at + 32:
        raise SystemExit("镜像太短，放不下头（需要至少 0x220 字节）")
    if bytes(data[at:at + 8]) != MAGIC:
        raise SystemExit(
            "0x200 处不是镜像头魔数 —— 链接脚本的 .fw_header 段没生效？\n"
            "  实际读到: " + bytes(data[at:at + 8]).hex(' '))

    n = len(data)
    data[at + OFF_BYTES:at + OFF_BYTES + 4] = n.to_bytes(4, 'little')
    # ⚠ 先把 crc 字段清零再算 —— 和固件侧、和上位机 imageCrc32() 同一条规则。
    data[at + OFF_CRC:at + OFF_CRC + 4] = (0).to_bytes(4, 'little')
    crc = zlib.crc32(bytes(data)) & 0xFFFFFFFF
    data[at + OFF_CRC:at + OFF_CRC + 4] = crc.to_bytes(4, 'little')
    return n, crc


def write_hex(path: str, data: bytes, base: int = 0x08000000) -> None:
    """重新生成 Intel HEX。⚠ 不能只改 .bin —— `Debug/ISP.py` 烧的是 .hex，
    只盖 .bin 的章会让**烧进板子的那一份没有章**，而 .bin 看起来是对的。"""
    lines = []
    hi = None
    for off in range(0, len(data), 32):
        addr = base + off
        if (addr >> 16) != hi:
            hi = addr >> 16
            rec = bytes([2, 0, 0, 4, (hi >> 8) & 0xFF, hi & 0xFF])
            lines.append(':%s%02X' % (rec.hex().upper(), (-sum(rec)) & 0xFF))
        chunk = data[off:off + 32]
        rec = bytes([len(chunk), (addr >> 8) & 0xFF, addr & 0xFF, 0]) + chunk
        lines.append(':%s%02X' % (rec.hex().upper(), (-sum(rec)) & 0xFF))
    lines.append(':00000001FF')
    with open(path, 'w', newline='\n') as f:
        f.write('\n'.join(lines) + '\n')


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    binp = sys.argv[1]
    data = bytearray(open(binp, 'rb').read())
    n, crc = stamp(data)
    open(binp, 'wb').write(bytes(data))
    print("盖章: %s  %d bytes  crc32 %08x" % (binp, n, crc))

    if len(sys.argv) > 2:
        write_hex(sys.argv[2], bytes(data))
        print("重写: %s（⚠ ISP.py 烧的是 .hex，只盖 .bin 的章等于没盖）" % sys.argv[2])
    return 0


if __name__ == '__main__':
    sys.exit(main())
