import cv2
import numpy as np
import time

# 初始化摄像头
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)

# 创建HSV调节窗口
cv2.namedWindow("HSV Adjust", cv2.WINDOW_NORMAL)

# 橙色乒乓球参数
cv2.createTrackbar("Orange H1", "HSV Adjust", 0, 179, lambda x: x)
cv2.createTrackbar("Orange H2", "HSV Adjust", 22, 179, lambda x: x)
cv2.createTrackbar("Orange S1", "HSV Adjust", 100, 255, lambda x: x)
cv2.createTrackbar("Orange S2", "HSV Adjust", 255, 255, lambda x: x)
cv2.createTrackbar("Orange V1", "HSV Adjust", 100, 255, lambda x: x)
cv2.createTrackbar("Orange V2", "HSV Adjust", 255, 255, lambda x: x)

# 白色乒乓球参数
cv2.createTrackbar("White H1", "HSV Adjust", 0, 179, lambda x: x)
cv2.createTrackbar("White H2", "HSV Adjust", 179, 179, lambda x: x)
cv2.createTrackbar("White S1", "HSV Adjust", 0, 255, lambda x: x)
cv2.createTrackbar("White S2", "HSV Adjust", 30, 255, lambda x: x)
cv2.createTrackbar("White V1", "HSV Adjust", 220, 255, lambda x: x)
cv2.createTrackbar("White V2", "HSV Adjust", 255, 255, lambda x: x)

# 反光抑制参数
cv2.createTrackbar("Glare Thresh", "HSV Adjust", 220, 255, lambda x: x)

# 形态学内核
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))

def detect_balls(mask, min_radius=10, max_radius=100):
    """通用乒乓球检测函数"""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    valid_balls = []
    
    for cnt in contours:
        area = cv2.contourArea(cnt)
        perimeter = cv2.arcLength(cnt, True)
        
        # 面积过滤
        if perimeter == 0 or not (np.pi*(min_radius**2) < area < np.pi*(max_radius**2)):
            continue
        
        # 圆度计算
        circularity = (4 * np.pi * area) / (perimeter**2)
        if circularity < 0.65:
            continue
        
        # 获取最小外接圆
        (x, y), radius = cv2.minEnclosingCircle(cnt)
        if min_radius <= radius <= max_radius:
            valid_balls.append( (int(x), int(y), int(radius), cnt) )
    
    return valid_balls

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # 反光抑制处理
    glare_thresh = cv2.getTrackbarPos("Glare Thresh", "HSV Adjust")
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, glare_mask = cv2.threshold(gray, glare_thresh, 255, cv2.THRESH_BINARY)
    frame = cv2.inpaint(frame, glare_mask, 3, cv2.INPAINT_TELEA)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # 获取颜色参数
    orange_lower = np.array([cv2.getTrackbarPos(n, "HSV Adjust") for n in 
                           ["Orange H1","Orange S1","Orange V1"]])
    orange_upper = np.array([cv2.getTrackbarPos(n, "HSV Adjust") for n in 
                           ["Orange H2","Orange S2","Orange V2"]])
    
    white_lower = np.array([cv2.getTrackbarPos(n, "HSV Adjust") for n in 
                          ["White H1","White S1","White V1"]])
    white_upper = np.array([cv2.getTrackbarPos(n, "HSV Adjust") for n in 
                          ["White H2","White S2","White V2"]])

    # 创建颜色掩膜
    mask_orange = cv2.inRange(hsv, orange_lower, orange_upper)
    mask_white = cv2.inRange(hsv, white_lower, white_upper)

    # 形态学优化
    mask_orange = cv2.morphologyEx(mask_orange, cv2.MORPH_OPEN, kernel, 2)
    mask_orange = cv2.morphologyEx(mask_orange, cv2.MORPH_CLOSE, kernel, 3)
    
    mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_OPEN, kernel, 3)
    mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_CLOSE, kernel, 5)

    # 检测橙色乒乓球
    orange_balls = detect_balls(mask_orange)
    for x, y, r, cnt in orange_balls:
        # 绘制轮廓（橙色）
        cv2.drawContours(frame, [cnt], -1, (0, 165, 255), 2)
        # 绘制中心点（红色）
        cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)
        # 显示坐标
        cv2.putText(frame, f"Orange ({x},{y})", (x-50, y-r-15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)

    # 检测白色乒乓球
    white_balls = detect_balls(mask_white)
    for x, y, r, cnt in white_balls:
        # 绘制轮廓（青色）
        cv2.drawContours(frame, [cnt], -1, (255, 255, 0), 2)
        # 绘制中心点（蓝色）
        cv2.circle(frame, (x, y), 5, (255, 0, 0), -1)
        # 显示坐标
        cv2.putText(frame, f"White ({x},{y})", (x-50, y-r-15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

    # 显示处理结果
    display = cv2.hconcat([
        frame,
        cv2.cvtColor(mask_orange, cv2.COLOR_GRAY2BGR),
        cv2.cvtColor(mask_white, cv2.COLOR_GRAY2BGR)
    ])
    cv2.imshow("Detection Result", display)

    key = cv2.waitKey(1)
    if key == ord('s'):
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        cv2.imwrite(f"detection_{timestamp}.jpg", display)
    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()