import cv2
import numpy as np
import time
import math

class BallDetector:
    def __init__(self, show_display=False, record_video=False, focal_length=554.26, real_diameter=0.04, 
                 smoothing_window=5, calibration_file='/usr/src/ai/calibration/valid/calibration.xml'):
        # 相机初始化
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        # 物理参数
        self.focal_length = focal_length
        self.real_diameter = real_diameter
        
        # 图像处理参数
        self.show_display = show_display
        self.record_video = record_video
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        
        # 橙色HSV阈值
        self.orange_lower = np.array([0, 100, 100])
        self.orange_upper = np.array([22, 255, 255])
        
        # 白色灰度参数
        self.white_lower = 200   # 亮度下限
        self.white_upper = 255   # 亮度上限
        self.white_blur = 5      # 高斯模糊核大小
        
        # 形态学参数
        self.orange_iter_open = 2
        self.orange_iter_close = 3
        self.white_iter_open = 3
        self.white_iter_close = 5
        
        # 几何参数
        self.min_radius = 10
        self.max_radius = 100
        self.min_circularity = 0.65
        
        # 反光抑制阈值
        self.glare_thresh = 234
        
        # 标定参数加载
        self._load_calibration(calibration_file)
        
        # 滤波参数
        self.smoothing_window = smoothing_window
        self.x_history = []
        self.y_history = []
        
        if self.show_display:
            self._init_trackbars()
        
        if self.record_video:
            self.video_writer = cv2.VideoWriter('/usr/src/ai/record/detection.avi', 
                                                cv2.VideoWriter_fourcc(*'XVID'), 
                                                8, (640, 480))
        else:
            self.video_writer = None

    def _load_calibration(self, calibration_file):
        """加载相机标定参数"""
        if calibration_file:
            fs = cv2.FileStorage(calibration_file, cv2.FILE_STORAGE_READ)
            if fs.isOpened():
                self.camera_matrix = fs.getNode('camera_matrix').mat()
                self.distortion_coefficients = fs.getNode('distortion_coefficients').mat()
                fs.release()
            else:
                print(f"无法打开标定文件: {calibration_file}")
                self._set_default_calibration()
        else:
            self._set_default_calibration()

    def _set_default_calibration(self):
        """设置默认标定参数"""
        self.camera_matrix = np.array([[self.focal_length, 0, 320],
                                      [0, self.focal_length, 240],
                                      [0, 0, 1]], dtype=np.float32)
        self.distortion_coefficients = np.zeros(5, dtype=np.float32)

    def _init_trackbars(self):
        """初始化可视化调节面板"""
        cv2.namedWindow("Settings", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Settings", 800, 600)
        
        # 橙色小球参数
        cv2.createTrackbar("Orange H1", "Settings", 0, 179, lambda x: x)
        cv2.createTrackbar("Orange H2", "Settings", 22, 179, lambda x: x)
        cv2.createTrackbar("Orange S1", "Settings", 100, 255, lambda x: x)
        cv2.createTrackbar("Orange S2", "Settings", 255, 255, lambda x: x)
        cv2.createTrackbar("Orange V1", "Settings", 100, 255, lambda x: x)
        cv2.createTrackbar("Orange V2", "Settings", 255, 255, lambda x: x)
        cv2.createTrackbar("O_Open", "Settings", 2, 5, lambda x: x)
        cv2.createTrackbar("O_Close", "Settings", 3, 5, lambda x: x)
        
        # 白色小球参数
        cv2.createTrackbar("W_Lower", "Settings", 154, 255, lambda x: x)
        cv2.createTrackbar("W_Upper", "Settings", 186, 255, lambda x: x)
        cv2.createTrackbar("W_Blur", "Settings", 5, 15, lambda x: x if x%2==1 else x+1)
        cv2.createTrackbar("W_Open", "Settings", 2, 5, lambda x: x)
        cv2.createTrackbar("W_Close", "Settings", 5, 5, lambda x: x)
        
        # 通用参数
        cv2.createTrackbar("MinRadius", "Settings", 10, 50, lambda x: x)
        cv2.createTrackbar("MaxRadius", "Settings", 100, 200, lambda x: x)
        cv2.createTrackbar("Circularity", "Settings", 65, 100, lambda x: x)
        cv2.createTrackbar("GlareThresh", "Settings", 234, 255, lambda x: x)

    def update_settings(self):
        """从轨迹栏更新所有参数"""
        # 橙色参数
        self.orange_lower = np.array([
            cv2.getTrackbarPos("Orange H1", "Settings"),
            cv2.getTrackbarPos("Orange S1", "Settings"),
            cv2.getTrackbarPos("Orange V1", "Settings")
        ])
        self.orange_upper = np.array([
            cv2.getTrackbarPos("Orange H2", "Settings"),
            cv2.getTrackbarPos("Orange S2", "Settings"),
            cv2.getTrackbarPos("Orange V2", "Settings")
        ])
        self.orange_iter_open = cv2.getTrackbarPos("O_Open", "Settings")
        self.orange_iter_close = cv2.getTrackbarPos("O_Close", "Settings")
        
        # 白色参数
        self.white_lower = cv2.getTrackbarPos("W_Lower", "Settings")
        self.white_upper = cv2.getTrackbarPos("W_Upper", "Settings")
        self.white_blur = cv2.getTrackbarPos("W_Blur", "Settings")
        self.white_iter_open = cv2.getTrackbarPos("W_Open", "Settings")
        self.white_iter_close = cv2.getTrackbarPos("W_Close", "Settings")
        
        # 通用参数
        self.min_radius = cv2.getTrackbarPos("MinRadius", "Settings")
        self.max_radius = cv2.getTrackbarPos("MaxRadius", "Settings")
        self.min_circularity = cv2.getTrackbarPos("Circularity", "Settings") / 100.0
        self.glare_thresh = cv2.getTrackbarPos("GlareThresh", "Settings")

    def _process_white(self, frame):
        """灰度法处理白色小球"""
        # 转换为灰度并高斯模糊
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (self.white_blur, self.white_blur), 0)
        
        # 自适应阈值处理
        _, mask = cv2.threshold(blurred, self.white_lower, self.white_upper, cv2.THRESH_BINARY)
        
        # 形态学操作
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel, iterations=self.white_iter_open)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel, iterations=self.white_iter_close)
        
        return mask

    def _process_orange(self, frame):
        """HSV法处理橙色小球"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.orange_lower, self.orange_upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel, iterations=self.orange_iter_open)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel, iterations=self.orange_iter_close)
        return mask

    def _find_contours(self, mask):
        """轮廓检测与筛选"""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid = []
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 100:
                continue
                
            # 圆形度计算
            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0:
                continue
            circularity = (4 * np.pi * area) / (perimeter**2)
            
            # 边界圆验证
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            
            if (self.min_radius <= radius <= self.max_radius and 
                circularity >= self.min_circularity):
                valid.append((x, y, radius*2))
                
        return valid

    def detect_balls(self, frame):
        """主检测流程"""
        try:
            # 预处理
            frame = cv2.undistort(frame, self.camera_matrix, self.distortion_coefficients)
            glare_free = self._glare_suppression(frame)
            
            # 并行处理两种颜色
            orange_mask = self._process_orange(glare_free)
            white_mask = self._process_white(glare_free)
            
            # 结果融合
            orange_balls = self._find_contours(orange_mask)
            white_balls = self._find_contours(white_mask)
            
            # 优先返回橙色检测结果
            if orange_balls:
                return self._calculate_metrics(orange_balls[0], frame)
            elif white_balls:
                return self._calculate_metrics(white_balls[0], frame)
            return 0, 0, 0, 0, 0, frame
            
        except Exception as e:
            print(f"检测异常: {str(e)}")
            return 0, 0, 0, 0, 0, frame

    def _glare_suppression(self, frame):
        """反光抑制处理"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, glare_mask = cv2.threshold(gray, self.glare_thresh, 255, cv2.THRESH_BINARY)
        return cv2.inpaint(frame, glare_mask, 3, cv2.INPAINT_TELEA)

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
            if self.show_display or self.record_video:
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

    def detect_and_display(self):
        """主检测与显示循环"""
        try:
            while True:
                start_time = time.time()
                ret, frame = self.cap.read()
                if not ret:
                    break
                if self.show_display:
                    self.update_settings()
                    orange_mask = self._process_orange(frame)
                    white_mask = self._process_white(frame)
                    cv2.imshow("Orange Mask", orange_mask)
                    cv2.imshow("White Mask", white_mask)
                status, x, y, distance, angle, processed_frame = self.detect_balls(frame)

                # 在终端实时打印检测到的数据
                if status:
                    print(f"位置: ({x}, {y}), 距离: {distance:.2f}米, 角度: {angle:.1f}度")

                cv2.imshow("Frame", processed_frame)

                # 记录视频
                if self.record_video and self.video_writer is not None:
                    self.video_writer.write(processed_frame)

                # 如果需要，将数据发送到STM32的逻辑可以在此添加

                # 按下'q'键退出循环
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

                # 控制帧率
                elapsed_time = time.time() - start_time
                sleep_time = max(0, (1/30) - elapsed_time)
                time.sleep(sleep_time)
        finally:
            self.cap.release()
            if self.video_writer is not None:
                self.video_writer.release()
            cv2.destroyAllWindows()

if __name__ == "__main__":
    detector = BallDetector(show_display=False, record_video=True)
    detector.detect_and_display()