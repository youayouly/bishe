# uart.py
import serial
import time
from detect import BallDetector  # 导入重构后的检测类

# 树莓派串口设备（根据实际修改）
# ser = serial.Serial('COM13', 115200, timeout=1)
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)


# 初始化检测器（关闭显示提升性能）
detector = BallDetector(show_display=False)

# uart.py（树莓派端）
def send_data(ball_type, x, y):
    # 计算校验和（异或校验）
    checksum = ball_type ^ (x & 0xFF) ^ (x >> 8) ^ (y & 0xFF) ^ (y >> 8)
    # 格式化数据包
    data = f"${ball_type},{x},{y},{checksum}\n"
    ser.write(data.encode())

def receive_response():
    response = ser.readline().decode().strip()
    if response:
        print("Received:", response)

def main():
    try:
        while True:
            ret, frame = detector.cap.read()
            if not ret:
                print("摄像头读取失败")
                break
            
            # 检测球数据
            ball_type, x, y = detector.detect_balls(frame)
            
            if ball_type > 0:  # 检测到球时发送
                send_data(ball_type, x, y)
                receive_response()
            
            time.sleep(0.1)  # 控制发送频率
            
    except KeyboardInterrupt:
        print("程序终止")
    finally:
        detector.release()
        ser.close()

if __name__ == "__main__":
    main()