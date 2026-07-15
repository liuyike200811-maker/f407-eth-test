#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
STM32 Modbus RTU 从站单测脚本 (电脑当主站, 冒充 HMI)
====================================================
用法:  python3 modbus_test.py [串口]
       默认串口 /dev/ttyUSB0 (USB转TTL), 115200-8-N-1, 站号1

依赖:  只用 pyserial ->  pip install pyserial   (或 sudo pacman -S python-pyserial)
       不需要 pymodbus, Modbus 帧本脚本手搓。

契约: HMI/STM32_Modbus寄存器契约.txt (v2)
  设定区 4x0000~0006 (下标0~6):  α β 频率 波幅 升高 循环 升速
  反馈区 4x0010~0018 (下标16~24): 状态 故障从站 在线数 WKC 轴1 轴2 轴3 心跳 当前模式
  控制区 线圈 0x0000~0008:        模式0~4 归零 急停 下电 故障复位
"""
import sys, time, struct

try:
    import serial
except ImportError:
    sys.exit("缺 pyserial:  pip install pyserial")

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
BAUD = 115200
SLAVE = 1

STATUS_TXT = {0: "启动中", 1: "待机", 2: "运行中", 3: "归零中", 4: "故障/未就绪", 5: "已下电"}

def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return crc

def txrx(ser, pdu: bytes, expect_len: int) -> bytes:
    """发一帧(自动加站号+CRC), 收响应并校验。返回响应去掉站号/功能/CRC后的数据段。"""
    frame = bytes([SLAVE]) + pdu
    frame += struct.pack("<H", crc16(frame))
    ser.reset_input_buffer()
    ser.write(frame)
    resp = ser.read(expect_len)
    if len(resp) < 5:
        raise IOError(f"无响应或太短 (收到 {len(resp)} 字节: {resp.hex()}) —— 板子没跑到位/接线反了/串口选错?")
    if resp[0] != SLAVE:
        raise IOError(f"站号不对: {resp[0]}")
    if resp[1] & 0x80:
        raise IOError(f"从站返回异常码 0x{resp[2]:02X} (功能0x{resp[1]&0x7f:02X})")
    if crc16(resp[:-2]) != struct.unpack("<H", resp[-2:])[0]:
        raise IOError(f"CRC 校验失败: {resp.hex()}")
    return resp

def read_holding(ser, start, count):
    pdu = bytes([0x03]) + struct.pack(">HH", start, count)
    resp = txrx(ser, pdu, 5 + count * 2)
    n = resp[2]
    return list(struct.unpack(">" + "H" * (n // 2), resp[3:3 + n]))

def write_register(ser, addr, value):
    pdu = bytes([0x06]) + struct.pack(">HH", addr, value & 0xFFFF)
    txrx(ser, pdu, 8)

def write_coil(ser, addr, on):
    pdu = bytes([0x05]) + struct.pack(">HH", addr, 0xFF00 if on else 0x0000)
    txrx(ser, pdu, 8)

def s16(v):   # 无符号16位 -> 有符号
    return v - 0x10000 if v >= 0x8000 else v

def main():
    print(f"打开 {PORT} @ {BAUD} 8N1, 站号 {SLAVE} ...")
    ser = serial.Serial(PORT, BAUD, bytesize=8, parity="N", stopbits=1, timeout=0.3)
    time.sleep(0.2)

    # --- 测试1: 连读心跳, 证明板子活着 + 协议通 ---
    print("\n[测试1] 连读心跳寄存器 4x0017 (下标23) 5 次, 应看到数值在变:")
    last = None
    for i in range(5):
        hb = read_holding(ser, 23, 1)[0]
        moving = "  <-- 在涨✓" if (last is not None and hb != last) else ""
        print(f"   心跳 = {hb}{moving}")
        last = hb
        time.sleep(0.3)

    # --- 测试2: 读整个反馈区, 解码 ---
    print("\n[测试2] 读反馈区 4x0010~4x0018 (下标16~24):")
    fb = read_holding(ser, 16, 9)
    print(f"   运行状态字 = {fb[0]} ({STATUS_TXT.get(fb[0], '?')})")
    print(f"   故障从站号 = {fb[1]}   在线从站数 = {fb[2]}   WKC = {fb[3]}")
    print(f"   轴1位置 = {s16(fb[4])/10:.1f}mm  轴2 = {s16(fb[5])/10:.1f}mm  轴3 = {s16(fb[6])/10:.1f}mm")
    print(f"   心跳 = {fb[7]}   当前模式 = {fb[8]} (99=待机)")

    # --- 测试3: 读设定区默认值 ---
    print("\n[测试3] 读设定区 4x0000~4x0006 (下标0~6), 应是固件默认值:")
    sp = read_holding(ser, 0, 7)
    names = ["α幅度(度)", "β幅度(度)", "频率(厘赫)", "波幅(mm)", "升高(mm)", "循环(次)", "升速(RPM)"]
    for nm, v in zip(names, sp):
        print(f"   {nm} = {v}")

    # --- 测试4: 写一个参数再读回, 验证写功能 ---
    print("\n[测试4] 写 4x0000(α幅度)=22, 读回验证:")
    write_register(ser, 0, 22)
    time.sleep(0.1)
    back = read_holding(ser, 0, 1)[0]
    print(f"   读回 α幅度 = {back}   {'写入成功✓' if back == 22 else '不一致✗'}")
    # 还原成默认 15
    write_register(ser, 0, 15)

    print("\n全部通过的话 => 板子 Modbus 从站没问题, 可以去接 HMI 了。")
    ser.close()

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"\n✗ 出错: {e}")
        sys.exit(1)
