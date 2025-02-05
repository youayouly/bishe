import serial
import time
from detect import BallDetector

# 串口配置（根据实际修改）
# ser = serial.Serial('COM13', 115200, timeout=1)
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

# 初始化检测器（带相机参数）
detector = BallDetector(
    show_display=False,
    focal_length=650,    # 需要根据实际相机校准
    real_diameter=0.067  # 球的真实直径（单位：米）
)

def send_data(ball_type, x, y, distance, angle):
    # 确保使用16位有符号整数
    payload = [
        int(ball_type),
        int(x) & 0xFFFF,
        int(y) & 0xFFFF,
        int(distance * 1000) & 0xFFFF,
        int(angle * 100) & 0xFFFF
    ]
    
    # 逐字节异或校验
    checksum = 0
    for value in payload:
        checksum ^= (value >> 8) & 0xFF  # 先处理高字节
        checksum ^= value & 0xFF        # 再处理低字节
    
    data = f"${payload[0]},{payload[1]},{payload[2]},{payload[3]},{payload[4]},{checksum}\n"
    ser.write(data.encode())

def receive_response():
    """接收并处理响应"""
    response = ser.readline().decode().strip()
    if response:
        print(f"[UART响应] {response}")

def main():
    try:
        while True:
            ret, frame = detector.cap.read()
            if not ret:
                print("[错误] 摄像头读取失败")
                break
            
            # 检测球体数据
            ball_type, x, y, distance, angle, _ = detector.detect_balls(frame)
            # 用下划线忽略最后一个返回值
            
            if ball_type > 0:
                print(f"检测到球: 类型={ball_type}, X={x}, Y={y}, 距离={distance:.2f}m, 角度={angle:.2f}°")
                send_data(ball_type, x, y, distance, angle)
                receive_response()
            
            time.sleep(0.05)  # 约20Hz发送频率
            
    except KeyboardInterrupt:
        print("\n程序终止")
    finally:
        detector.release()
        ser.close()

if __name__ == "__main__":
    main()