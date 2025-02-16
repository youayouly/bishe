#!/usr/bin/env python3
import serial
import time
import cv2
from detect import BallDetector

# 串口配置，根据实际情况修改设备名称及波特率
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

def send_data(ball_type, x, y, distance, angle):
    def to_16bit_signed(value):
        ivalue = int(value)
        return ivalue & 0xFFFF if ivalue >= 0 else (1 << 16) + ivalue

    # 确保坐标范围合法（转换为整数）
    ix = int(x)
    iy = int(y)
    assert 0 <= ix <= 640, f"X坐标超限: {ix}"
    assert 0 <= iy <= 480, f"Y坐标超限: {iy}"

    # 转换距离（单位：米转换为毫米）和角度（扩大100倍）
    payload = [
        to_16bit_signed(int(ball_type)),
        to_16bit_signed(ix),
        to_16bit_signed(iy),
        to_16bit_signed(int(distance * 1000)),  # 距离转换为毫米
        to_16bit_signed(int(angle * 100))         # 角度扩大100倍
    ]

    # 构造数据字符串，格式为 "$a,b,c,d,e\n"
    data = f"${payload[0]},{payload[1]},{payload[2]},{payload[3]},{payload[4]}\n"
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
    detector = BallDetector(show_display=False, record_video=False, focal_length=550, real_diameter=0.04)
    frame_count = 0
    try:
        while True:
            ret, frame = detector.cap.read()
            if not ret:
                continue
            frame_count += 1
            result = detector.detect_balls(frame)
            if len(result) == 6:
                ball_type, x, y, distance, angle, processed_frame = result
                # 每帧均发送数据
                send_data(ball_type, x, y, distance, angle)
                receive_response()
                if detector.record_video and detector.video_writer is not None:
                    detector.video_writer.write(processed_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n程序终止")
    finally:
        detector.cap.release()
        if detector.video_writer is not None:
            detector.video_writer.release()
        ser.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()