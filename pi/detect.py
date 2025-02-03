# detect.py
import cv2
import numpy as np
import time

class BallDetector:
    def __init__(self, show_display=False):
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        self.show_display = show_display  # 是否显示调试窗口
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        
        # 初始化默认HSV参数（可调整）
        self.orange_lower = np.array([0, 100, 100])
        self.orange_upper = np.array([22, 255, 255])
        self.white_lower = np.array([74, 0, 107])
        self.white_upper = np.array([132, 38, 255])
        self.glare_thresh = 234
        
        if self.show_display:
            self._init_trackbars()

    def _init_trackbars(self):
        """初始化调试窗口的轨迹栏"""
        cv2.namedWindow("HSV Adjust", cv2.WINDOW_NORMAL)
        cv2.createTrackbar("Orange H1", "HSV Adjust", 0, 179, lambda x: x)
        cv2.createTrackbar("Orange H2", "HSV Adjust", 22, 179, lambda x: x)
        cv2.createTrackbar("Orange S1", "HSV Adjust", 100, 255, lambda x: x)
        cv2.createTrackbar("Orange S2", "HSV Adjust", 255, 255, lambda x: x)
        cv2.createTrackbar("Orange V1", "HSV Adjust", 100, 255, lambda x: x)
        cv2.createTrackbar("Orange V2", "HSV Adjust", 255, 255, lambda x: x)
        cv2.createTrackbar("White H1", "HSV Adjust", 74, 179, lambda x: x)
        cv2.createTrackbar("White H2", "HSV Adjust", 132, 179, lambda x: x)
        cv2.createTrackbar("White S1", "HSV Adjust", 0, 255, lambda x: x)
        cv2.createTrackbar("White S2", "HSV Adjust", 38, 255, lambda x: x)
        cv2.createTrackbar("White V1", "HSV Adjust", 107, 255, lambda x: x)
        cv2.createTrackbar("White V2", "HSV Adjust", 255, 255, lambda x: x)
        cv2.createTrackbar("Glare Thresh", "HSV Adjust", 234, 255, lambda x: x)

    def detect_balls(self, frame):
        """处理单帧图像，返回检测到的球数据"""
        # 反光抑制处理
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, glare_mask = cv2.threshold(gray, self.glare_thresh, 255, cv2.THRESH_BINARY)
        frame = cv2.inpaint(frame, glare_mask, 3, cv2.INPAINT_TELEA)
        
        # 颜色检测
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_orange = cv2.inRange(hsv, self.orange_lower, self.orange_upper)
        mask_white = cv2.inRange(hsv, self.white_lower, self.white_upper)
        
        # 形态学优化
        mask_orange = cv2.morphologyEx(mask_orange, cv2.MORPH_OPEN, self.kernel, 2)
        mask_orange = cv2.morphologyEx(mask_orange, cv2.MORPH_CLOSE, self.kernel, 3)
        mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_OPEN, self.kernel, 3)
        mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_CLOSE, self.kernel, 5)
        
        # 检测球
        orange_balls = self._find_balls(mask_orange)
        white_balls = self._find_balls(mask_white)
        
        # 优先返回橙色球（假设1为橙色，2为白色）
        if orange_balls:
            x, y = orange_balls[0][0], orange_balls[0][1]
            return 1, x, y
        elif white_balls:
            x, y = white_balls[0][0], white_balls[0][1]
            return 2, x, y
        return 0, 0, 0  # 未检测到球

    def _find_balls(self, mask, min_radius=10, max_radius=100):
        """内部方法：检测特定掩膜中的球"""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid_balls = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0 or not (np.pi*(min_radius**2) < area < np.pi*(max_radius**2)):
                continue
            circularity = (4 * np.pi * area) / (perimeter**2)
            if circularity < 0.65:
                continue
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            if min_radius <= radius <= max_radius:
                valid_balls.append( (int(x), int(y), int(radius), cnt) )
        return valid_balls

    def update_settings_from_trackbars(self):
        """从轨迹栏更新参数（仅在显示模式启用时调用）"""
        if not self.show_display:
            return
        self.orange_lower = np.array([cv2.getTrackbarPos(n, "HSV Adjust") for n in 
                                    ["Orange H1","Orange S1","Orange V1"]])
        self.orange_upper = np.array([cv2.getTrackbarPos(n, "HSV Adjust") for n in 
                                    ["Orange H2","Orange S2","Orange V2"]])
        self.white_lower = np.array([cv2.getTrackbarPos(n, "HSV Adjust") for n in 
                                   ["White H1","White S1","White V1"]])
        self.white_upper = np.array([cv2.getTrackbarPos(n, "HSV Adjust") for n in 
                                   ["White H2","White S2","White V2"]])
        self.glare_thresh = cv2.getTrackbarPos("Glare Thresh", "HSV Adjust")

    def release(self):
        """释放资源"""
        self.cap.release()
        if self.show_display:
            cv2.destroyAllWindows()

# 独立运行时的测试代码
if __name__ == "__main__":
    detector = BallDetector(show_display=True)
    try:
        while True:
            ret, frame = detector.cap.read()
            if not ret:
                break
            detector.update_settings_from_trackbars()
            ball_type, x, y = detector.detect_balls(frame)
            print(f"Detected: Type={ball_type}, X={x}, Y={y}")
            time.sleep(0.1)
    finally:
        detector.release()