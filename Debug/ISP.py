import serial
import time
import sys
import os

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM11"
FILE = sys.argv[2] if len(sys.argv) > 2 else "DlaBoard_B.hex"

BAUD = 115200
FLASH_BASE = 0x08000000
CHUNK = 64

# ====== 按你的自动下载电路设置 ======
# 你的FlyMcu日志：
# DTR低 = 复位
# DTR高 = 释放复位
# RTS高 = 进入BootLoader
DTR_RESET_LEVEL = False   # 低电平 = 复位
DTR_RUN_LEVEL   = True    # 高电平 = 释放复位

RTS_BOOT_LEVEL  = True    # 高电平 = 进BootLoader
RTS_APP_LEVEL   = False   # 低电平 = 运行用户程序


def checksum(data):
    x = 0
    for b in data:
        x ^= b
    return x & 0xFF


def read_exact(ser, n, timeout=2):
    old = ser.timeout
    ser.timeout = timeout
    data = ser.read(n)
    ser.timeout = old
    if len(data) != n:
        raise TimeoutError(f"Timeout read {n}, got {data.hex(' ')}")
    return data


def wait_ack(ser, timeout=2):
    b = read_exact(ser, 1, timeout)[0]
    if b != 0x79:
        raise RuntimeError(f"NACK or bad response: 0x{b:02X}")
    return True


def send_cmd(ser, cmd):
    ser.write(bytes([cmd, cmd ^ 0xFF]))
    ser.flush()
    wait_ack(ser)


def enter_bootloader(ser):
    print("进入BootLoader...")

    ser.setRTS(RTS_BOOT_LEVEL)
    time.sleep(0.05)

    ser.setDTR(DTR_RESET_LEVEL)
    time.sleep(0.15)

    ser.setDTR(DTR_RUN_LEVEL)
    time.sleep(0.3)

    ser.reset_input_buffer()
    ser.reset_output_buffer()

    print("同步...")
    ser.write(b"\x7F")
    ser.flush()
    wait_ack(ser, 3)
    print("同步成功")


def get_id(ser):
    send_cmd(ser, 0x02)
    n = read_exact(ser, 1)[0]
    data = read_exact(ser, n + 1)
    wait_ack(ser)
    pid = int.from_bytes(data, "big")
    print(f"PID: 0x{pid:04X}")
    return pid


def mass_erase(ser):
    print("全片擦除...")
    send_cmd(ser, 0x44)  # Extended Erase

    # Mass erase: 0xFFFF + checksum 0x00
    ser.write(b"\xFF\xFF\x00")
    ser.flush()

    wait_ack(ser, 30)
    print("擦除成功")


def write_memory(ser, address, data):
    send_cmd(ser, 0x31)

    addr = address.to_bytes(4, "big")
    ser.write(addr + bytes([checksum(addr)]))
    ser.flush()
    wait_ack(ser)

    n = len(data) - 1
    packet = bytes([n]) + data
    ser.write(packet + bytes([checksum(packet)]))
    ser.flush()
    wait_ack(ser, 3)


def read_memory(ser, address, length):
    send_cmd(ser, 0x11)

    addr = address.to_bytes(4, "big")
    ser.write(addr + bytes([checksum(addr)]))
    ser.flush()
    wait_ack(ser)

    n = length - 1
    ser.write(bytes([n, n ^ 0xFF]))
    ser.flush()
    wait_ack(ser)

    return read_exact(ser, length, 3)


def go_app(ser, address=FLASH_BASE):
    print("跳转运行...")
    send_cmd(ser, 0x21)

    addr = address.to_bytes(4, "big")
    ser.write(addr + bytes([checksum(addr)]))
    ser.flush()
    wait_ack(ser)


def parse_hex(path):
    mem = {}

    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        upper = 0
        for line in f:
            line = line.strip()
            if not line or line[0] != ":":
                continue

            raw = bytes.fromhex(line[1:])
            count = raw[0]
            addr = (raw[1] << 8) | raw[2]
            rectype = raw[3]
            data = raw[4:4 + count]

            if rectype == 0x00:
                abs_addr = upper + addr
                for i, b in enumerate(data):
                    mem[abs_addr + i] = b

            elif rectype == 0x01:
                break

            elif rectype == 0x04:
                upper = int.from_bytes(data, "big") << 16

    if not mem:
        raise RuntimeError("HEX文件为空或解析失败")

    start = min(mem.keys())
    end = max(mem.keys()) + 1
    image = bytearray([0xFF] * (end - start))

    for addr, b in mem.items():
        image[addr - start] = b

    return start, bytes(image)


def load_firmware(path):
    ext = os.path.splitext(path)[1].lower()

    if ext == ".hex":
        addr, data = parse_hex(path)
        return addr, data

    elif ext == ".bin":
        with open(path, "rb") as f:
            return FLASH_BASE, f.read()

    else:
        raise RuntimeError("只支持 .hex 或 .bin")


def main():
    if not os.path.exists(FILE):
        print(f"文件不存在: {FILE}")
        return

    start_addr, fw = load_firmware(FILE)

    print(f"串口: {PORT}")
    print(f"文件: {FILE}")
    print(f"起始地址: 0x{start_addr:08X}")
    print(f"大小: {len(fw)} bytes")

    with serial.Serial(
        PORT,
        BAUD,
        bytesize=8,
        parity=serial.PARITY_EVEN,
        stopbits=1,
        timeout=1
    ) as ser:

        enter_bootloader(ser)
        get_id(ser)

        mass_erase(ser)

        print("开始写入...")
        total = len(fw)

        for offset in range(0, total, CHUNK):
            chunk = fw[offset:offset + CHUNK]
            addr = start_addr + offset
            write_memory(ser, addr, chunk)

            percent = int((offset + len(chunk)) * 100 / total)
            print(f"\r写入进度: {percent}%  0x{addr:08X}", end="")

        print("\n写入完成")

        print("开始校验...")
        for offset in range(0, total, CHUNK):
            chunk = fw[offset:offset + CHUNK]
            addr = start_addr + offset
            rb = read_memory(ser, addr, len(chunk))

            if rb != chunk:
                print(f"\n校验失败: 0x{addr:08X}")
                return

            percent = int((offset + len(chunk)) * 100 / total)
            print(f"\r校验进度: {percent}%", end="")

        print("\n校验成功")

        # 退出BootLoader，跳转APP
        go_app(ser, FLASH_BASE)

        ser.setRTS(RTS_APP_LEVEL)
        ser.setDTR(DTR_RUN_LEVEL)

        print("完成")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print("\n失败:", e)