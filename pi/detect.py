import cv2
import numpy as np
import time
import math

class BallDetector:
    def __init__(self, show_display=False, focal_length=554.26, real_diameter=0.04, smoothing_window=5, calibration_file='/usr/src/ai/calibration/valid/calibration.xml'):
        # 相机初始化
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        # 物理参数
        self.focal_length = focal_length     # 相机焦距（像素）
        self.real_diameter = real_diameter     # 实际球体直径（米）
        
        # 图像处理参数
        self.show_display = show_display
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        
        # HSV阈值默认值
        self.orange_lower = np.array([0, 100, 100])
        self.orange_upper = np.array([22, 255, 255])
        self.white_lower = np.array([74, 0, 107])
        self.white_upper = np.array([132, 38, 255])
        self.glare_thresh = 234
        
        # 加载相机标定参数
        if calibration_file:
            fs = cv2.FileStorage(calibration_file, cv2.FILE_STORAGE_READ)
            if fs.isOpened():
                # 加载相机内参矩阵和畸变系数
                self.camera_matrix = fs.getNode('camera_matrix').mat()
                self.distortion_coefficients = fs.getNode('distortion_coefficients').mat()
                fs.release()
            else:
                print(f"无法打开标定文件: {calibration_file}")
        else:
            # 如果没有提供标定文件，使用默认值
            self.camera_matrix = np.array([[focal_length, 0, 320],
                                           [0, focal_length, 240],
                                           [0, 0, 1]], dtype=np.float32)
            self.distortion_coefficients = np.zeros(5, dtype=np.float32)

            
        # 滤波参数：滑动平均
        self.smoothing_window = smoothing_window
        self.x_history = []
        self.y_history = []
        
        # 初始化卡尔曼滤波器（可选）
        self.kalman = cv2.KalmanFilter(4, 2)
        self.kalman.measurementMatrix = np.array([[1, 0, 0, 0],
                                                  [0, 1, 0, 0]], np.float32)
        self.kalman.transitionMatrix = np.array([[1, 0, 1, 0],
                                                 [0, 1, 0, 1],
                                                 [0, 0, 1, 0],
                                                 [0, 0, 0, 1]], np.float32)
        self.kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03
        self.kalman.measurementNoiseCov = np.eye(2, dtype=np.float32) * 0.5
        self.kalman.errorCovPost = np.eye(4, dtype=np.float32)
        self.kalman.statePost = np.zeros((4, 1), dtype=np.float32)
        
        if self.show_display:
            self._init_trackbars()

    def _init_trackbars(self):
        """初始化调试轨迹栏"""
        cv2.namedWindow("HSV Adjust")
        cv2.createTrackbar("Orange H1", "HSV Adjust", 0, 179, lambda x: x)
        cv2.createTrackbar("Orange H2", "HSV Adjust", 22, 179, lambda x: x)
        cv2.createTrackbar("Orange S1", "HSV Adjust", 100, 255, lambda x: x)
        cv2.createTrackbar("Orange S2", "HSV Adjust", 255, 255, lambda x: x)
        cv2.createTrackbar("Orange V1", "HSV Adjust", 100, 255, lambda x: x)
        cv2.createTrackbar("Orange V2", "HSV Adjust", 255, 255, lambda x: x)
        cv2.createTrackbar("White H1", "HSV Adjust", 74, 179, lambda x: x)
        cv2.createTrackbar("White H2", "HSV Adjust", 132, 179, lambda x: x)
        cv2.createTrackbar("White S1", "HSV Adjust", 11, 255, lambda x: x)#白球的底线不能太低，要不然反光进来
        cv2.createTrackbar("White S2", "HSV Adjust", 33, 255, lambda x: x)
        cv2.createTrackbar("White V1", "HSV Adjust", 107, 255, lambda x: x)
        cv2.createTrackbar("White V2", "HSV Adjust", 255, 255, lambda x: x)
        cv2.createTrackbar("Glare Thresh", "HSV Adjust", 234, 255, lambda x: x)

    def update_settings_from_trackbars(self):
        """从轨迹栏更新HSV参数"""
        if not self.show_display:
            return
        self.orange_lower = np.array([cv2.getTrackbarPos("Orange H1", "HSV Adjust"),
                                      cv2.getTrackbarPos("Orange S1", "HSV Adjust"),
                                      cv2.getTrackbarPos("Orange V1", "HSV Adjust")])
        self.orange_upper = np.array([cv2.getTrackbarPos("Orange H2", "HSV Adjust"),
                                      cv2.getTrackbarPos("Orange S2", "HSV Adjust"),
                                      cv2.getTrackbarPos("Orange V2", "HSV Adjust")])
        self.white_lower = np.array([cv2.getTrackbarPos("White H1", "HSV Adjust"),
                                     cv2.getTrackbarPos("White S1", "HSV Adjust"),
                                     cv2.getTrackbarPos("White V1", "HSV Adjust")])
        self.white_upper = np.array([cv2.getTrackbarPos("White H2", "HSV Adjust"),
                                     cv2.getTrackbarPos("White S2", "HSV Adjust"),
                                     cv2.getTrackbarPos("White V2", "HSV Adjust")])
        self.glare_thresh = cv2.getTrackbarPos("Glare Thresh", "HSV Adjust")

    def _glare_suppression(self, frame):
        """反光抑制处理"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, glare_mask = cv2.threshold(gray, self.glare_thresh, 255, cv2.THRESH_BINARY)
        return cv2.inpaint(frame, glare_mask, 3, cv2.INPAINT_TELEA)

    def _color_detection(self, frame):
        """颜色分割及形态学优化"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        orange_mask = cv2.inRange(hsv, self.orange_lower, self.orange_upper)
        white_mask = cv2.inRange(hsv, self.white_lower, self.white_upper)
        # 先开运算去除噪点，再闭运算填补空洞
        orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_OPEN, self.kernel, iterations=2)
        orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_CLOSE, self.kernel, iterations=3)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, self.kernel, iterations=3)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, self.kernel, iterations=5)
        return orange_mask, white_mask

    def _find_balls(self, mask, min_radius=10, max_radius=100):
        """轮廓检测及球体筛选"""
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
                valid_balls.append((x, y, radius*2))  # 保存球的直径
        return valid_balls

    def _apply_moving_average(self, x, y):
        """滑动平均滤波"""
        self.x_history.append(x)
        self.y_history.append(y)
        if len(self.x_history) > self.smoothing_window:
            self.x_history.pop(0)
            self.y_history.pop(0)
        return np.mean(self.x_history), np.mean(self.y_history)

    def _apply_kalman_filter(self, x, y):
        """卡尔曼滤波处理"""
        measurement = np.array([[np.float32(x)], [np.float32(y)]])
        self.kalman.correct(measurement)
        prediction = self.kalman.predict()
        return prediction[0][0], prediction[1][0]

    def _calculate_metrics(self, ball_info, frame):
        """计算距离、角度及滤波处理"""
        try:
            x, y, diameter = ball_info
            x, y = int(x), int(y)
            # 使用滑动平均滤波（如需卡尔曼滤波，请切换下面两行）
            x, y = self._apply_moving_average(x, y)
            # x, y = self._apply_kalman_filter(x, y)
            if diameter <= 0 or not (0 <= x < 640 and 0 <= y < 480):
                return 0, 0, 0, 0, 0, frame
            distance = (self.real_diameter * self.focal_length) / max(diameter, 1e-6)
            dx = x - 320  # 图像中心为基准
            angle = math.degrees(math.atan(dx / self.focal_length)) if self.focal_length != 0 else 0
            if self.show_display:
                self._draw_debug_info(frame, int(x), int(y), distance, angle)
            return 1, int(x), int(y), distance, angle, frame
        except Exception as e:
            print("指标计算异常:", e)
            return 0, 0, 0, 0, 0, frame

    def _draw_debug_info(self, frame, x, y, distance, angle):
        """在图像上绘制调试信息"""
        cv2.line(frame, (320, 0), (320, 480), (100, 100, 100), 1)
        cv2.line(frame, (0, 240), (640, 240), (100, 100, 100), 1)
        cv2.circle(frame, (x, y), 10, (0, 255, 0), 2)
        cv2.line(frame, (320, 240), (x, y), (0, 0, 255), 2)
        cv2.putText(frame, f"Dist: {distance:.2f}m", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(frame, f"Angle: {angle:.1f} deg", (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

    def detect_and_display(self):
        """主检测与显示循环"""
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break
            if self.show_display:
                self.update_settings_from_trackbars()
            status, x, y, distance, angle, processed_frame = self.detect_balls(frame)
            cv2.imshow("Frame", processed_frame)
            # 可在此处添加数据传输到STM32的逻辑
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            time.sleep(0.1)
        self.cap.release()
        cv2.destroyAllWindows()

    def detect_balls(self, frame):
        """球体检测主流程"""
        try:
            if frame is None:
                return 0, 0, 0, 0, 0, None
            processed_frame = self._glare_suppression(frame)
            orange_mask, white_mask = self._color_detection(processed_frame)
            orange_balls = self._find_balls(orange_mask) or []
            white_balls = self._find_balls(white_mask) or []
            if len(orange_balls) > 0:
                return self._calculate_metrics(orange_balls[0], frame)
            elif len(white_balls) > 0:
                return self._calculate_metrics(white_balls[0], frame)
            else:
                return 0, 0, 0, 0, 0, frame
        except Exception as e:
            print("检测异常:", e)
            return 0, 0, 0, 0, 0, frame

if __name__ == "__main__":
    detector = BallDetector(show_display=True)
    detector.detect_and_display()
