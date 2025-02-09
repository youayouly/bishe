#!/usr/bin/env python3
import serial
import time
from detect import BallDetector

# 串口配置，根据实际情况修改设备名称及波特率
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

# 初始化检测器（请根据实际相机校准参数设置）
detector = BallDetector(
    show_display=False,
    focal_length=550,    # 根据实际校准
    real_diameter=0.04  # 球的真实直径（单位：米）
)

def send_data(ball_type, x, y, distance, angle):
    def to_16bit_signed(value):
        return value & 0xFFFF if value >= 0 else (1 << 16) + value

    # 确保数值范围合法
    assert 0 <= x <= 640, f"X坐标超限: {x}"
    assert 0 <= y <= 480, f"Y坐标超限: {y}"

    # 将 distance 以米为单位转换为毫米后存入 payload（整数）
    payload = [
        to_16bit_signed(ball_type),
        to_16bit_signed(x),
        to_16bit_signed(y),
        to_16bit_signed(int(distance * 1000)),  # 距离转换为毫米
        to_16bit_signed(int(angle * 100))         # angle扩大100倍传输
    ]

    # 计算校验和：对每个16位数据分别取高8位和低8位异或
    checksum = 0
    for val in payload:
        checksum ^= (val >> 8) & 0xFF
        checksum ^= val & 0xFF

    # 构造数据字符串，格式为 "$a,b,c,d,e,f\n"
    data = f"${payload[0]},{payload[1]},{payload[2]},{payload[3]},{payload[4]},{checksum}\n"
    ser.write(data.encode('ascii'))
    print(f"[SENT] {data.strip()}")

def receive_response():
    """接收并处理响应"""
    try:
        response = ser.readline().decode('utf-8', errors='replace').strip()
    except Exception as e:
        print("解码异常:", e)
        response = ""
    if response:
        print(f"[UART] {response}")

def main():
    frame_count = 0
    try:
        while True:
            ret, frame = detector.cap.read()
            if not ret:
                continue

            frame_count += 1
            # 每5帧发送一次数据，降低发送频率
            if frame_count % 15 == 0:
                result = detector.detect_balls(frame)
                # 检测函数返回6个元素：[ball_type, x, y, distance, angle, frame]
                if len(result) == 6:
                    ball_type, x, y, distance, angle, _ = result
                    send_data(ball_type, x, y, distance, angle)
                    receive_response()
            time.sleep(0.05)  # 每帧间隔50ms
    except KeyboardInterrupt:
        print("\n程序终止")
    finally:
        detector.release()
        ser.close()

if __name__ == "__main__":
    main()
