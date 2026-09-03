"""DlaBoard_A 运行期 CLI 测试小工具（USART1 8N1 115200）。
用法: python cli_test.py COM10 "FOC?" "RC?" "IMU?" "ST?"
每条命令发送后等待并打印 MCU 回应(约 0.6s 窗口)。"""
import serial, sys, time

port = sys.argv[1] if len(sys.argv) > 1 else "COM10"
cmds = sys.argv[2:] if len(sys.argv) > 2 else ["FOC?"]

with serial.Serial(port, 115200, bytesize=8, parity=serial.PARITY_NONE,
                   stopbits=1, timeout=0.1) as ser:
    time.sleep(0.2)
    ser.reset_input_buffer()
    for c in cmds:
        ser.write((c + "\r\n").encode())
        ser.flush()
        time.sleep(0.6)
        resp = ser.read(4096).decode(errors="replace").strip()
        print(f">>> {c}")
        for ln in resp.splitlines():
            if ln.strip():
                print(f"    {ln.strip()}")
        if not resp:
            print("    (no response)")
