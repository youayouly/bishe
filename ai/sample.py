import cv2
import numpy as np
import time
import math
import configparser
from joblib import Parallel, delayed


class BallDetector:
    def __init__(self, show_display=False, record_video=False, focal_length=554.26, real_diameter=0.04, config_file='/usr/src/ai/config.txt'):
        # 读取配置文件
        config = configparser.ConfigParser()
        config.read(config_file)
        
        # 相机初始化
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # 设置分辨率
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)  # 设置帧率
        self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)     # 关闭自动对焦
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))  # 设置格式为 MJPG
        
        # 物理参数
        self.focal_length = focal_length
        self.real_diameter = real_diameter
        
        # 图像处理参数
        self.show_display = show_display
        self.record_video = record_video
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        
        # 橙色HSV阈值
        self.orange_lower = np.array([config.getint('DEFAULT', 'orange_lower_h'),
                                      config.getint('DEFAULT', 'orange_lower_s'),
                                      config.getint('DEFAULT', 'orange_lower_v')])
        self.orange_upper = np.array([config.getint('DEFAULT', 'orange_upper_h'),
                                      config.getint('DEFAULT', 'orange_upper_s'),
                                      config.getint('DEFAULT', 'orange_upper_v')])
        
        # 白色灰度参数
        self.white_lower = config.getint('DEFAULT', 'white_lower')
        self.white_upper = config.getint('DEFAULT', 'white_upper')
        self.white_blur = config.getint('DEFAULT', 'white_blur')
        
        # 形态学参数
        self.orange_iter_open = config.getint('DEFAULT', 'orange_iter_open')
        self.orange_iter_close = config.getint('DEFAULT', 'orange_iter_close')
        self.white_iter_open = config.getint('DEFAULT', 'white_iter_open')
        self.white_iter_close = config.getint('DEFAULT', 'white_iter_close')
        
        # 几何参数
        self.min_radius = config.getint('DEFAULT', 'min_radius')
        self.max_radius = config.getint('DEFAULT', 'max_radius')
        self.min_circularity = config.getfloat('DEFAULT', 'min_circularity')
        
        # 反光抑制阈值
        self.glare_thresh = config.getint('DEFAULT', 'glare_thresh')
        
        # 标定参数加载
        self._load_calibration(config.get('DEFAULT', 'calibration_file'))
        
        # 滤波参数
        self.smoothing_window = config.getint('DEFAULT', 'smoothing_window')
        self.x_history = []
        self.y_history = []
        self.distance_history = []
        self.angle_history = []
        
        if self.show_display:
            self._init_trackbars()
        
        if self.record_video:
            self.video_writer = cv2.VideoWriter('/usr/src/ai/record/detection.avi', 
                                                cv2.VideoWriter_fourcc(*'XVID'), 
                                                8, (640, 480))
        else:
            self.video_writer = None

        # 初始化FPS相关变量
        self.frame_count = 0
        self.start_time = time.time()
        
        # 预分配缓冲区
        self.gray_buf = np.zeros((480, 640), dtype=np.uint8)
        self.hsv_buf = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # --------------------- 增强版 Kalman 滤波器 初始化 ---------------------
        # 状态变量: [x, y, dx, dy, diameter, ddiameter]（图像坐标和直径相关）
        # 测量变量: [x, y, diameter]
        self.kalman = cv2.KalmanFilter(6, 3)
        self.kalman.transitionMatrix = np.array([
            [1, 0, 1, 0, 0, 0],
            [0, 1, 0, 1, 0, 0],
            [0, 0, 1, 0, 0, 0],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 1],
            [0, 0, 0, 0, 0, 1]
        ], np.float32)
        
        self.kalman.measurementMatrix = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0]
        ], np.float32)
        
        self.kalman.processNoiseCov = np.eye(6, dtype=np.float32) * 0.01
        self.kalman.measurementNoiseCov = np.eye(3, dtype=np.float32) * 0.1
        self.kalman.errorCovPost = np.eye(6, dtype=np.float32) * 100
        
        self.last_prediction = None
        self.miss_count = 0
        # --------------------- 增强版 Kalman 滤波器 完毕 ---------------------

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
        cv2.createTrackbar("Orange H1", "Settings", self.orange_lower[0], 179, lambda x: x)
        cv2.createTrackbar("Orange H2", "Settings", self.orange_upper[0], 179, lambda x: x)
        cv2.createTrackbar("Orange S1", "Settings", self.orange_lower[1], 255, lambda x: x)
        cv2.createTrackbar("Orange S2", "Settings", self.orange_upper[1], 255, lambda x: x)
        cv2.createTrackbar("Orange V1", "Settings", self.orange_lower[2], 255, lambda x: x)
        cv2.createTrackbar("Orange V2", "Settings", self.orange_upper[2], 255, lambda x: x)
        cv2.createTrackbar("O_Open", "Settings", self.orange_iter_open, 5, lambda x: x)
        cv2.createTrackbar("O_Close", "Settings", self.orange_iter_close, 5, lambda x: x)
        
        # 白色小球参数
        cv2.createTrackbar("W_Lower", "Settings", self.white_lower, 255, lambda x: x)
        cv2.createTrackbar("W_Upper", "Settings", self.white_upper, 255, lambda x: x)
        cv2.createTrackbar("W_Blur", "Settings", self.white_blur, 15, lambda x: x if x % 2 == 1 else x+1)
        cv2.createTrackbar("W_Open", "Settings", self.white_iter_open, 5, lambda x: x)
        cv2.createTrackbar("W_Close", "Settings", self.white_iter_close, 5, lambda x: x)
        
        # 通用参数
        cv2.createTrackbar("MinRadius", "Settings", self.min_radius, 50, lambda x: x)
        cv2.createTrackbar("MaxRadius", "Settings", self.max_radius, 200, lambda x: x)
        cv2.createTrackbar("Circularity", "Settings", int(self.min_circularity * 100), 100, lambda x: x)
        cv2.createTrackbar("GlareThresh", "Settings", self.glare_thresh, 255, lambda x: x)

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

    def _process_whitе(self, frame):
        """灰度法处理白色小球"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (self.white_blur, self.white_blur), 0)
        _, mask = cv2.threshold(blurred, self.white_lower, self.white_upper, cv2.THRESH_BINARY)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel, iterations=self.white_iter_open)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel, iterations=self.white_iter_close)
        return mask

    def _process_orange(self, frame):
        """HSV法处理橙色小球，合并开闭运算为一次形态学操作"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.orange_lower, self.orange_upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel, iterations=self.orange_iter_open)
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel, iterations=self.orange_iter_close)

    def _find_contours(self, mask):
        """轮廓检测与筛选"""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 100:
                continue
            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0:
                continue
            circularity = (4 * np.pi * area) / (perimeter ** 2)
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            if self.min_radius <= radius <= self.max_radius and circularity >= self.min_circularity:
                valid.append((x, y, radius * 2))
        return valid

    def detect_balls(self, frame):
        """主检测流程（使用整帧检测，不使用预计算映射表）"""
        try:
            # 去畸变处理（直接调用 undistort）
            undistorted = cv2.undistort(frame, self.camera_matrix, self.distortion_coefficients)
            # 使用预分配缓冲区
            cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY, dst=self.gray_buf)
            cv2.cvtColor(frame, cv2.COLOR_BGR2HSV, dst=self.hsv_buf)
            self.hsv_buf[:, :, 2] = self.gray_buf  # 使用灰度图替代V通道
            frame_modified = cv2.cvtColor(self.hsv_buf, cv2.COLOR_HSV2BGR)
            
            glare_free = self._glare_suppression(frame_modified)
            
            # 并行处理两种颜色（使用线程后端避免 pickle 问题）
            results = Parallel(n_jobs=2, backend='threading')([
                delayed(self._process_orange)(glare_free),
                delayed(self._process_whitе)(glare_free)
            ])
            orange_mask, white_mask = results
            
            # 轮廓检测
            orange_balls = self._find_contours(orange_mask)
            white_balls = self._find_contours(white_mask)
            
            # 优先返回橙色检测结果
            if orange_balls:
                result = self._calculate_metrics(orange_balls[0], frame)
            elif white_balls:
                result = self._calculate_metrics(white_balls[0], frame)
            else:
                result = (0, 0, 0, 0, 0, frame)
            
            return result
        except Exception as e:
            print(f"检测异常: {str(e)}")
            return 0, 0, 0, 0, 0, frame

    def _glare_suppression(self, frame):
        """反光抑制处理，根据图像平均亮度决定是否启用"""
        avg_brightness = np.mean(self.gray_buf)
        if avg_brightness < 100:  # 低光照时不处理反光
            return frame
        _, glare_mask = cv2.threshold(self.gray_buf, self.glare_thresh, 255, cv2.THRESH_BINARY)
        return cv2.inpaint(frame, glare_mask, 3, cv2.INPAINT_TELEA)

    def _calculate_metrics(self, ball_info, frame):
        """改进的指标计算、增强版卡尔曼滤波以及连续帧平均平滑"""
        try:
            x, y, diameter = ball_info
            measurement = np.array([[x], [y], [diameter]], dtype=np.float32)
            
            # 执行预测步骤
            _ = self.kalman.predict()
            
            # 判断测量是否有效
            if not (0 <= x < 640 and 0 <= y < 480 and diameter > 0):
                self.miss_count += 1
                if self.miss_count > 5:  # 连续5帧无效则重置卡尔曼滤波器
                    self.kalman.init(self.kalman.statePost.shape[0], 3)
                    self.last_prediction = None
                    return 0, 0, 0, 0, 0, frame
            else:
                self.miss_count = 0
                # 校正步骤，更新卡尔曼状态
                self.kalman.correct(measurement)
            
            # 获取校正后的状态
            state = self.kalman.statePost
            
            # 计算实际距离（根据直径）和角度（根据x与图像中心偏差）
            distance = (self.real_diameter * self.focal_length) / max(state[4, 0], 1e-6)
            dx = state[0, 0] - 320
            angle = math.degrees(math.atan(dx / self.focal_length))
            
            # 将当前结果加入历史记录
            self.x_history.append(state[0, 0])
            self.y_history.append(state[1, 0])
            self.distance_history.append(distance)
            self.angle_history.append(angle)
            
            # 保持历史列表长度不超过 smoothing_window 帧
            if len(self.x_history) > self.smoothing_window:
                self.x_history.pop(0)
                self.y_history.pop(0)
                self.distance_history.pop(0)
                self.angle_history.pop(0)
            
            # 计算连续帧平均
            smooth_x = sum(self.x_history) / len(self.x_history)
            smooth_y = sum(self.y_history) / len(self.y_history)
            smooth_distance = sum(self.distance_history) / len(self.distance_history)
            smooth_angle = sum(self.angle_history) / len(self.angle_history)
            
            # 保存最后有效预测（平滑后的结果）
            self.last_prediction = (smooth_x, smooth_y, smooth_distance, smooth_angle)
            
            if self.show_display or self.record_video:
                self._draw_debug_info(frame, int(smooth_x), int(smooth_y), smooth_distance, smooth_angle)
            
            return 1, smooth_x, smooth_y, smooth_distance, smooth_angle, frame

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
        cv2.putText(frame, f"FPS: {self.fps:.2f}", (10, 110),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

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
                    orange_mask_disp = self._process_orange(frame)
                    white_mask_disp = self._process_whitе(frame)
                    cv2.imshow("Orange Mask", orange_mask_disp)
                    cv2.imshow("White Mask", white_mask_disp)
                status, x, y, distance, angle, processed_frame = self.detect_balls(frame)
                
                # 更新FPS计数器
                self.frame_count += 1
                elapsed_time = time.time() - self.start_time
                self.fps = self.frame_count / elapsed_time
                
                if status:
                    print(f"位置: ({x}, {y}), 距离: {distance:.2f}米, 角度: {angle:.1f}度, FPS: {self.fps:.2f}")
                
                cv2.imshow("Frame", processed_frame)
                
                if self.record_video and self.video_writer is not None:
                    self.video_writer.write(processed_frame)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                
                sleep_time = max(0, (1/30) - (time.time() - start_time))
                time.sleep(sleep_time)
        finally:
            self.cap.release()
            if self.video_writer is not None:
                self.video_writer.release()
            cv2.destroyAllWindows()


if __name__ == "__main__":
    detector = BallDetector(show_display=False, record_video=False)
    detector.detect_and_display()