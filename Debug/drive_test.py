"""持续驱动测试：每 150ms 重发驱动命令(维持 FOC 的 300ms 死区)，期间采样 ST?，
结束发 STOP+OFF。用法: python drive_test.py COM10 "FOC D 15" 3
  参数3 = 持续秒数(默认 3)。轮子务必悬空！"""
import serial, sys, time

port = sys.argv[1] if len(sys.argv) > 1 else "COM10"
cmd  = sys.argv[2] if len(sys.argv) > 2 else "FOC D 15"
secs = float(sys.argv[3]) if len(sys.argv) > 3 else 3.0

def send(ser, c):
    ser.write((c + "\r\n").encode()); ser.flush()

def read(ser, t=0.2):
    time.sleep(t)
    return ser.read(4096).decode(errors="replace").strip()

with serial.Serial(port, 115200, timeout=0.1) as ser:
    time.sleep(0.2); ser.reset_input_buffer()
    print(f"DRIVE '{cmd}' for {secs}s (wheels must be raised)")
    t0 = time.time(); n = 0
    while time.time() - t0 < secs:
        send(ser, cmd); n += 1
        time.sleep(0.15)
        if n % 4 == 0:                 # 每 ~0.6s 采样一次状态
            send(ser, "ST?")
            r = read(ser, 0.15)
            for ln in r.splitlines():
                if "FOC st" in ln:
                    print(f"  {ln.strip()}")
    send(ser, "FOC STOP"); read(ser, 0.2)
    send(ser, "FOC OFF");  read(ser, 0.2)
    send(ser, "ST?"); r = read(ser, 0.3)
    for ln in r.splitlines():
        if "FOC st" in ln:
            print(f"  STOPPED: {ln.strip()}")
    print("done")
