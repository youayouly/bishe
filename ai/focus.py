import cv2
import numpy as np

def calculate_focal_length_from_video(known_radius, real_distance, smoothing_window=5):
    # 打开视频流（如果有摄像头可以直接打开）
    cap = cv2.VideoCapture(0)  # 0表示默认的摄像头

    if not cap.isOpened():
        print("无法打开视频流")
        return

    # 设置摄像头分辨率
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # 初始化一个列表，用来存储焦距值（用于平滑）
    focal_lengths = []

    while True:
        ret, frame = cap.read()
        if not ret:
            print("无法读取视频帧")
            break

        # 图像预处理：转换到 HSV 色彩空间
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 设置橙色的HSV范围（根据提供的参数调整）
        lower_orange = np.array([5, 114, 120])  # 橙色最低范围
        upper_orange = np.array([18, 159, 255])  # 橙色最高范围

        # 创建掩模，提取橙色部分
        mask = cv2.inRange(hsv, lower_orange, upper_orange)

        # 对掩模进行形态学操作，去噪
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

        # 查找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) == 0:
            print("没有检测到橙色小球")
            continue

        # 假设第一个轮廓是小球
        largest_contour = max(contours, key=cv2.contourArea)

        # 计算轮廓的最小外接圆
        (x, y), radius_in_pixels = cv2.minEnclosingCircle(largest_contour)

        # 如果圆的半径太小，可能不是目标小球，跳过此帧
        if radius_in_pixels < 10:
            continue

        # 焦距计算：使用相似三角形公式
        focal_length = (known_radius * real_distance) / radius_in_pixels

        # 将焦距添加到平滑列表中
        focal_lengths.append(focal_length)

        # 如果列表的长度大于平滑窗口，则移除最旧的焦距值
        if len(focal_lengths) > smoothing_window:
            focal_lengths.pop(0)

        # 计算平滑后的焦距（取窗口中的平均值）
        smoothed_focal_length = np.mean(focal_lengths)

        # 在图像中绘制检测到的小球
        cv2.circle(frame, (int(x), int(y)), int(radius_in_pixels), (0, 255, 0), 2)

        # 在图像上显示焦距
        cv2.putText(frame, f"Focal Length: {smoothed_focal_length:.2f} pixels", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # 输出平滑后的焦距到终端
        print(f"Focal Length: {smoothed_focal_length:.2f} pixels")

        # 显示处理后的图像
        cv2.imshow('Video Stream', frame)

        # 显示调试图像
        cv2.imshow('HSV Image', hsv)
        cv2.imshow('Mask', mask)

        # 按'q'键退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    known_radius = 2  # 小球的已知半径（单位：cm）
    real_distance = 40  # 小球到相机的已知距离（单位：cm）

    calculate_focal_length_from_video(known_radius, real_distance)
