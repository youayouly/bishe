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
    def to_16bit_signed(value):
        return value & 0xFFFF if value >=0 else (1 << 16) + value
    
    # 确保数值范围合法
    assert 0 <= x <= 640, f"X坐标超限: {x}"
    assert 0 <= y <= 480, f"Y坐标超限: {y}"
    
    payload = [
        to_16bit_signed(ball_type),
        to_16bit_signed(x),
        to_16bit_signed(y),
        to_16bit_signed(int(distance * 1000)),
        to_16bit_signed(int(angle * 100))
    ]
    
    # 校验和计算
    checksum = 0
    for val in payload:
        checksum ^= (val >> 8) & 0xFF
        checksum ^= val & 0xFF
    
    data = f"${payload[0]},{payload[1]},{payload[2]},{payload[3]},{payload[4]},{checksum}\n"
    ser.write(data.encode())
    print(f"[SENT] {data.strip()}")


def receive_response():
    """接收并处理响应"""
    response = ser.readline().decode().strip()
    if response:
        print(f"[UART响应] {response}")

def main():
    try:
        while True:
            ret, frame = detector.cap.read()
            if not ret or frame is None:  # 增加空帧检查
                print("摄像头读取失败: 无效帧")
                time.sleep(0.1)
                continue
            
            # 解包时增加类型检查
            result = detector.detect_balls(frame)
            if len(result) != 6:  # 确保返回结构有效
                print(f"无效检测结果: {result}")
                continue
                
            ball_type, x, y, distance, angle, _ = result
            
            if ball_type > 0:  # 有效检测
                print(f"检测到球: 类型={ball_type}, X={x}, Y={y}, 距离={distance:.2f}m, 角度={angle:.2f}°")
                send_data(ball_type, x, y, distance, angle)
                receive_response()
            
            time.sleep(0.05)  # 降低CPU占用

    except KeyboardInterrupt:
        print("\n程序终止")
    finally:
        detector.release()
        ser.close()

if __name__ == "__main__":
    main()