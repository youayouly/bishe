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
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))  # 设置格式为 MJPG
        
        # 物理参数
        self.focal_length = focal_length
        self.real_diameter = real_diameter
        
        # 图像处理参数
        self.show_display = show_display
        self.record_video = record_video
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        
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
            # 复用灰度图计算HSV中的V通道
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            hsv[:, :, 2] = gray  # 使用灰度图替代V通道
            frame_modified = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
            
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
        avg_brightness = np.mean(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        if avg_brightness < 100:  # 低光照时不处理反光
            return frame
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, glare_mask = cv2.threshold(gray, self.glare_thresh, 255, cv2.THRESH_BINARY)
        return cv2.inpaint(frame, glare_mask, 3, cv2.INPAINT_TELEA)

    def _calculate_metrics(self, ball_info, frame):
        """计算距离、角度及滤波处理"""
        try:
            x, y, diameter = ball_info
            x, y = int(x), int(y)
            if diameter <= 0 or not (0 <= x < 640 and 0 <= y < 480):
                return 0, 0, 0, 0, 0, frame
            distance = (self.real_diameter * self.focal_length) / max(diameter, 1e-6)
            dx = x - 320  # 图像中心为基准
            angle = math.degrees(math.atan(dx / self.focal_length)) if self.focal_length != 0 else 0
            if self.show_display or self.record_video:
                self._draw_debug_info(frame, x, y, distance, angle)
            return 1, x, y, distance, angle, frame
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
        # 如果需要显示FPS信息，则调用 self.fps（已在__init__中初始化）
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