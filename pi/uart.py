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
        return value & 0xFFFF if value >= 0 else (1 << 16) + value

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

    # 构造数据字符串：5个字段加上校验和，共6个字段，格式为 "$a,b,c,d,e,f\n"
    data = f"${payload[0]},{payload[1]},{payload[2]},{payload[3]},{payload[4]},{checksum}\n"
    ser.write(data.encode('ascii'))
    print(f"[SENT] {data.strip()}")

def receive_response():
    """接收并处理响应"""
    try:
        # 修改为UTF-8解码，并在出错时替换非法字符
        response = ser.readline().decode('utf-8', errors='replace').strip()
    except Exception as e:
        print("解码异常:", e)
        response = ""
    if response:
        print(f"[UART响应] {response}")


def main():
    try:
        while True:
            ret, frame = detector.cap.read()
            if not ret:
                continue
            
            # 检测球数据（正确解包6个返回值）
            result = detector.detect_balls(frame)
            if len(result) == 6:  # 匹配实际返回值数量
                ball_type, x, y, distance, angle, _ = result
                
                # 调用 send_data 统一发送带校验和的数据
                send_data(ball_type, x, y, distance, angle)
                
                # 接收响应
                receive_response()
            
            time.sleep(0.05)  # 降低CPU占用

    except KeyboardInterrupt:
        print("\n程序终止")
    finally:
        detector.release()
        ser.close()

if __name__ == "__main__":
    main()
