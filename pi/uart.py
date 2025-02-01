import serial
import time

# 打开串口（树莓派上的 USB 串口通常是 /dev/ttyUSB0 或 /dev/ttyAMA0）
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  
time.sleep(2)  # 等待串口稳定

print("Waiting for data from STM32...")

try:
    while True:
        response = ser.readline().decode('utf-8').strip()  # 读取 STM32 发送的数据
        if response:
            print("Received from STM32:", response)  # 在终端打印接收到的数据
except KeyboardInterrupt:
    print("\nClosing Serial Port")
    ser.close()



# import serial
# import time

# # 连接 STM32 的串口（通常是 /dev/ttyUSB0）
# ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  # 确保端口正确
# time.sleep(2)  # 等待串口稳定

# print("Waiting for data from STM32...")

# try:
#     while True:
#         # 读取 STM32 发送的数据
#         data = ser.readline().decode('utf-8').strip()
#         if data:
#             print("Received from STM32:", data)  # 在终端显示 STM32 的数据
#         time.sleep(0.1)  # 避免过度占用 CPU
# except KeyboardInterrupt:
#     print("Closing Serial Port")
#     ser.close()
