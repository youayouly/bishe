import matplotlib.pyplot as plt
import numpy as np
from tkinter import Tk, Scale, HORIZONTAL, Label

# 定义距离范围
distances = np.linspace(1200, 200, 901)

# 定义目标位置
target_distance = 300
target_angle = 0  # 目标角度为正前方 0 度

# 初始 PID 参数
Kp_distance = 0.0001
Ki_distance = 0.00001
Kd_distance = 0.0001

Kp_angle = 0.0001
Ki_angle = 0.00001
Kd_angle = 0.0001

# 全局变量，初始化 PID 积分和前一误差值
integral_distance = 0.0
prev_error_distance = 0.0

integral_angle = 0.0
prev_error_angle = 0.0

def compute_pid(distances, angles, Kp_distance, Ki_distance, Kd_distance, Kp_angle, Ki_angle, Kd_angle):
    global integral_distance, prev_error_distance, integral_angle, prev_error_angle
    # 计算距离误差
    errors_distance = distances - target_distance
    
    # 更新积分项和微分项（针对距离）
    integral_distance += errors_distance
    derivative_distance = errors_distance - prev_error_distance
    
    # 计算电机输出
    motor_outputs = Kp_distance * errors_distance + Ki_distance * integral_distance + Kd_distance * derivative_distance
    motor_outputs = np.clip(motor_outputs, -0.4, 0.3)  # 限制输出范围在0.1到0.3之间
    
    prev_error_distance = errors_distance
    
    # 计算角度误差（假设目标角度为0度）
    errors_angle = angles - target_angle  # 模拟角度误差
    
    # 更新积分项和微分项（针对角度）
    integral_angle += errors_angle
    derivative_angle = errors_angle - prev_error_angle
    
    # 计算舵机输出
    servo_outputs = Kp_angle * errors_angle + Ki_angle * integral_angle + Kd_angle * derivative_angle
    servo_outputs = np.clip(servo_outputs, 0.1, 0.3)  # 限制输出范围在0.1到0.3之间
    
    prev_error_angle = errors_angle
    
    return motor_outputs, servo_outputs

def update_plot():
    global Kp_distance, Ki_distance, Kd_distance, Kp_angle, Ki_angle, Kd_angle
    angles = np.linspace(-2000, 2000, len(distances))  # 模拟角度变化范围
    motor_outputs, servo_outputs = compute_pid(distances, angles, Kp_distance, Ki_distance, Kd_distance, Kp_angle, Ki_angle, Kd_angle)
    
    # 获取输出的最大值和最小值
    max_motor_output = np.max(motor_outputs)
    min_motor_output = np.min(motor_outputs)
    max_servo_output = np.max(servo_outputs)
    min_servo_output = np.min(servo_outputs)
    
    # 清除之前的图形
    ax1.cla()
    ax2.cla()
    
    # 绘制电机输出曲线
    ax1.plot(distances, motor_outputs, 'r-', label='Motor Output')
    ax1.axhline(y=max_motor_output, color='r', linestyle='--', label=f'最大值: {max_motor_output:.2f}')
    ax1.axhline(y=min_motor_output, color='g', linestyle='--', label=f'最小值: {min_motor_output:.2f}')
    ax1.set_xlabel('距离')
    ax1.set_ylabel('电机输出')
    ax1.set_title('电机输出随距离变化曲线')
    ax1.legend()
    ax1.grid()
    
    # 绘制舵机输出曲线
    ax2.plot(angles, servo_outputs, 'b-', label='Servo Output')
    ax2.axhline(y=max_servo_output, color='r', linestyle='--', label=f'最大值: {max_servo_output:.2f}')
    ax2.axhline(y=min_servo_output, color='g', linestyle='--', label=f'最小值: {min_servo_output:.2f}')
    ax2.set_xlabel('角度')
    ax2.set_ylabel('舵机输出')
    ax2.set_title('舵机输出随角度变化曲线')
    ax2.legend()
    ax2.grid()
    
    fig.canvas.draw()

# 设置图形
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

# 创建 Tkinter 窗口
root = Tk()
root.title("PID Parameter Tuning")

# 创建滑块和标签
def create_slider(label_text, from_, to, resolution, initial_value, row, column, command):
    label = Label(root, text=label_text)
    label.grid(row=row, column=column)
    slider = Scale(root, from_=from_, to=to, resolution=resolution, orient=HORIZONTAL, command=command)
    slider.set(initial_value)
    slider.grid(row=row, column=column+1)
    return slider

def update_kp_distance(value):
    global Kp_distance
    Kp_distance = float(value)
    update_plot()

def update_ki_distance(value):
    global Ki_distance
    Ki_distance = float(value)
    update_plot()

def update_kd_distance(value):
    global Kd_distance
    Kd_distance = float(value)
    update_plot()

def update_kp_angle(value):
    global Kp_angle
    Kp_angle = float(value)
    update_plot()

def update_ki_angle(value):
    global Ki_angle
    Ki_angle = float(value)
    update_plot()

def update_kd_angle(value):
    global Kd_angle
    Kd_angle = float(value)
    update_plot()

create_slider("Kp_distance", 0.00001, 0.001, 0.00001, Kp_distance, 0, 0, update_kp_distance)
create_slider("Ki_distance", 0.00001, 0.001, 0.00001, Ki_distance, 1, 0, update_ki_distance)
create_slider("Kd_distance", 0.00001, 0.001, 0.00001, Kd_distance, 2, 0, update_kd_distance)
create_slider("Kp_angle", 0.00001, 0.001, 0.00001, Kp_angle, 3, 0, update_kp_angle)
create_slider("Ki_angle", 0.00001, 0.001, 0.00001, Ki_angle, 4, 0, update_ki_angle)
create_slider("Kd_angle", 0.00001, 0.001, 0.00001, Kd_angle, 5, 0, update_kd_angle)

plt.tight_layout()
plt.show()

root.mainloop()