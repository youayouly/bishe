import cv2
import numpy as np
import os

def capture_and_save_image():
    """捕获一张照片并保存到当前工作目录"""
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头")
        return None

    ret, frame = cap.read()
    if not ret:
        print("无法读取图像")
        cap.release()
        return None

    # 保存图像到当前工作目录
    image_path = 'captured_image.jpg'
    cv2.imwrite(image_path, frame)
    cap.release()
    return image_path

def update(val):
    """回调函数，用于更新参数并重新处理图像"""
    # 获取滑动条的当前值
    kernel_size = cv2.getTrackbarPos('Kernel Size', 'Control Panel')
    block_size = cv2.getTrackbarPos('Block Size', 'Control Panel')
    C = cv2.getTrackbarPos('C', 'Control Panel')

    # 调整核大小，确保为奇数
    kernel_size = kernel_size if kernel_size % 2 == 1 else kernel_size + 1

    # 创建结构元素
    kernel = np.ones((kernel_size, kernel_size), np.uint8)

    # 读取保存的图像
    frame = cv2.imread('captured_image.jpg')
    if frame is None:
        print("无法读取保存的图像")
        return

    # 转换为灰度图像
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 直方图均衡化，增强图像对比度
    gray = cv2.equalizeHist(gray)

    # 自适应阈值化：动态调整阈值，减小阴影影响
    white_mask = cv2.adaptiveThreshold(
        gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
        cv2.THRESH_BINARY, block_size, C
    )

    # 形态学操作：开运算去除噪点，再闭运算填补空洞
    white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel, iterations=2)
    white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel, iterations=3)

    # 显示结果
    cv2.imshow('Original Frame', frame)
    cv2.imshow('White Ball Mask', white_mask)

def main():
    # 捕获并保存一张照片
    image_path = capture_and_save_image()
    if image_path is None:
        return

    # 创建窗口
    cv2.namedWindow('Control Panel')

    # 创建滑动条
    cv2.createTrackbar('Kernel Size', 'Control Panel', 5, 21, update)
    cv2.createTrackbar('Block Size', 'Control Panel', 11, 21, update)
    cv2.createTrackbar('C', 'Control Panel', 2, 10, update)

    # 初始化参数
    update(0)

    # 按下'q'键退出循环
    while True:
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
