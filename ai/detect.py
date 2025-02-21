import cv2
import numpy as np
import time
import math
import configparser

class BallDetector:
    def __init__(self, show_display=False, record_video=False, focal_length=275, real_diameter=0.04, config_file='/usr/src/ai/config.txt'):
        config = configparser.ConfigParser()
        config.read(config_file)
        
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"设置后的分辨率: {self.width}x{self.height}")
        
        self.focal_length = focal_length
        self.real_diameter = real_diameter
        
        self.show_display = show_display
        self.record_video = record_video
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        
        self.orange_lower = np.array([config.getint('DEFAULT', 'orange_lower_h'),
                                      config.getint('DEFAULT', 'orange_lower_s'),
                                      config.getint('DEFAULT', 'orange_lower_v')])
        self.orange_upper = np.array([config.getint('DEFAULT', 'orange_upper_h'),
                                      config.getint('DEFAULT', 'orange_upper_s'),
                                      config.getint('DEFAULT', 'orange_upper_v')])
        
        self.white_lower = config.getint('DEFAULT', 'white_lower')
        self.white_upper = config.getint('DEFAULT', 'white_upper')
        self.white_blur = config.getint('DEFAULT', 'white_blur')
        
        self.orange_iter_open = config.getint('DEFAULT', 'orange_iter_open')
        self.orange_iter_close = config.getint('DEFAULT', 'orange_iter_close')
        self.white_iter_open = config.getint('DEFAULT', 'white_iter_open')
        self.white_iter_close = config.getint('DEFAULT', 'white_iter_close')
        
        self.min_radius = config.getint('DEFAULT', 'min_radius')
        self.max_radius = config.getint('DEFAULT', 'max_radius')
        self.min_circularity = config.getfloat('DEFAULT', 'min_circularity')
        
        self.glare_thresh = config.getint('DEFAULT', 'glare_thresh')
        
        self._load_calibration(config.get('DEFAULT', 'calibration_file'))
        
        self.smoothing_window = 5
        self.x_history = []
        self.y_history = []
        self.distance_history = []
        self.angle_history = []
        
        if self.show_display:
            self._init_trackbars()
        
        if self.record_video:
            self.video_writer = cv2.VideoWriter('/usr/src/ai/record/detection.avi', 
                                                cv2.VideoWriter_fourcc(*'XVID'), 
                                                8, (self.width, self.height))
        else:
            self.video_writer = None

        self.frame_count = 0
        self.start_time = time.time()
        self.fps = 0.0
        
        self.roi_top = 0
        self.roi_bottom = 320
        self.roi_height = self.roi_bottom - self.roi_top
        
        self.gray_buf = np.zeros((self.roi_height, self.width), dtype=np.uint8)
        self.hsv_buf = np.zeros((self.roi_height, self.width, 3), dtype=np.uint8)
        
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
        self.last_valid_result = (0, 0, 0, 0, 0, None)
        self.locked_ball = None  # 锁定最近的球
        self.locked_time = None  # 锁定时间
        self.lock_timeout = 10.0  # 锁定超时时间（秒）

    def _load_calibration(self, calibration_file):
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
        self.camera_matrix = np.array([[self.focal_length, 0, self.width // 2],
                                       [0, self.focal_length, self.height // 2],
                                       [0, 0, 1]], dtype=np.float32)
        self.distortion_coefficients = np.zeros(5, dtype=np.float32)

    def _init_trackbars(self):
        cv2.namedWindow("Settings", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Settings", 800, 600)
        
        cv2.createTrackbar("Orange H1", "Settings", self.orange_lower[0], 179, lambda x: x)
        cv2.createTrackbar("Orange H2", "Settings", self.orange_upper[0], 179, lambda x: x)
        cv2.createTrackbar("Orange S1", "Settings", self.orange_lower[1], 255, lambda x: x)
        cv2.createTrackbar("Orange S2", "Settings", self.orange_upper[1], 255, lambda x: x)
        cv2.createTrackbar("Orange V1", "Settings", self.orange_lower[2], 255, lambda x: x)
        cv2.createTrackbar("Orange V2", "Settings", self.orange_upper[2], 255, lambda x: x)
        cv2.createTrackbar("O_Open", "Settings", self.orange_iter_open, 5, lambda x: x)
        cv2.createTrackbar("O_Close", "Settings", self.orange_iter_close, 5, lambda x: x)
        
        cv2.createTrackbar("W_Lower", "Settings", self.white_lower, 255, lambda x: x)
        cv2.createTrackbar("W_Upper", "Settings", self.white_upper, 255, lambda x: x)
        cv2.createTrackbar("W_Blur", "Settings", self.white_blur, 15, lambda x: x if x % 2 == 1 else x+1)
        cv2.createTrackbar("W_Open", "Settings", self.white_iter_open, 5, lambda x: x)
        cv2.createTrackbar("W_Close", "Settings", self.white_iter_close, 5, lambda x: x)
        
        cv2.createTrackbar("MinRadius", "Settings", self.min_radius, 50, lambda x: x)
        cv2.createTrackbar("MaxRadius", "Settings", self.max_radius, 200, lambda x: x)
        cv2.createTrackbar("Circularity", "Settings", int(self.min_circularity * 100), 100, lambda x: x)
        cv2.createTrackbar("GlareThresh", "Settings", self.glare_thresh, 255, lambda x: x)

    def update_settings(self):
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
        
        self.white_lower = cv2.getTrackbarPos("W_Lower", "Settings")
        self.white_upper = cv2.getTrackbarPos("W_Upper", "Settings")
        self.white_blur = cv2.getTrackbarPos("W_Blur", "Settings")
        self.white_iter_open = cv2.getTrackbarPos("W_Open", "Settings")
        self.white_iter_close = cv2.getTrackbarPos("W_Close", "Settings")
        
        self.min_radius = cv2.getTrackbarPos("MinRadius", "Settings")
        self.max_radius = cv2.getTrackbarPos("MaxRadius", "Settings")
        self.min_circularity = cv2.getTrackbarPos("Circularity", "Settings") / 100.0
        self.glare_thresh = cv2.getTrackbarPos("GlareThresh", "Settings")

    def _process_white(self, frame):
        self.gray_buf[:] = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(self.gray_buf, (self.white_blur, self.white_blur), 0)
        _, mask = cv2.threshold(blurred, self.white_lower, self.white_upper, cv2.THRESH_BINARY)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel, iterations=self.white_iter_open)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel, iterations=self.white_iter_close)
        return mask

    def _process_orange(self, frame):
        self.hsv_buf[:] = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        self.hsv_buf[:, :, 2] = self.gray_buf
        frame_modified = cv2.cvtColor(self.hsv_buf, cv2.COLOR_HSV2BGR)
        mask = cv2.inRange(self.hsv_buf, self.orange_lower, self.orange_upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel, iterations=self.orange_iter_open)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel, iterations=self.orange_iter_close)
        return mask

    def _find_contours(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid_balls = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 50:
                continue
            (x, y), radius = cv2.minEnclosingCircle(contour)
            if self.min_radius <= radius <= self.max_radius:
                valid_balls.append((x, y, radius * 2))
        return valid_balls

    def detect_balls(self, frame):
        try:
            roi_frame = frame[self.roi_top:self.roi_bottom, 0:self.width]
            undistorted = cv2.undistort(roi_frame, self.camera_matrix, self.distortion_coefficients)
            orange_mask = self._process_orange(undistorted)
            white_mask = self._process_white(undistorted)
            
            orange_balls = self._find_contours(orange_mask)
            white_balls = self._find_contours(white_mask)
            
            all_balls = []
            for x, y, diameter in orange_balls:
                y += self.roi_top
                all_balls.append((x, y, diameter, 1))
            for x, y, diameter in white_balls:
                y += self.roi_top
                all_balls.append((x, y, diameter, 0))
            
            if not all_balls:
                self.locked_ball = None  # 未检测到球时解锁
                self.locked_time = None
                return 0, 0, 0, 0, 0, frame
            
            # 计算每个球的距离
            balls_with_distance = []
            for x, y, diameter, ball_type in all_balls:
                distance = (self.real_diameter * self.focal_length) / max(diameter, 1e-6)
                balls_with_distance.append((x, y, diameter, ball_type, distance))
            
            # 如果已锁定球，尝试匹配最近的球
            if self.locked_ball is not None and time.time() - self.locked_time < self.lock_timeout:
                locked_x, locked_y, locked_diameter, locked_type, locked_distance = self.locked_ball
                # 查找与锁定球最接近的球（基于位置和直径）
                closest_ball = min(balls_with_distance, key=lambda b: (
                    (b[0] - locked_x) ** 2 + (b[1] - locked_y) ** 2 + ((b[2] - locked_diameter) / 10) ** 2
                ), default=None)
                if closest_ball and abs(closest_ball[4] - locked_distance) < 0.2:  # 距离变化小于 0.2m
                    x, y, diameter, ball_type, distance = closest_ball
                    result = self._calculate_metrics((x, y, diameter), frame, ball_type)
                    self.last_valid_result = result
                    self.locked_ball = (x, y, diameter, ball_type, distance)  # 更新锁定球
                    return result
            
            # 未锁定或锁定超时，选择距离最近的球
            closest_ball = min(balls_with_distance, key=lambda b: b[4])  # 按距离排序
            x, y, diameter, ball_type, distance = closest_ball
            result = self._calculate_metrics((x, y, diameter), frame, ball_type)
            self.last_valid_result = result
            self.locked_ball = (x, y, diameter, ball_type, distance)  # 锁定最近的球
            self.locked_time = time.time()  # 记录锁定时间
            return result
        except Exception as e:
            print(f"检测异常: {str(e)}")
            self.locked_ball = None
            self.locked_time = None
            return 0, 0, 0, 0, 0, frame

    def _glare_suppression(self, frame):
        avg_brightness = np.mean(self.gray_buf)
        if avg_brightness < 100:
            return frame
        _, glare_mask = cv2.threshold(self.gray_buf, self.glare_thresh, 255, cv2.THRESH_BINARY)
        return cv2.inpaint(frame, glare_mask, 3, cv2.INPAINT_TELEA)

    def _calculate_metrics(self, ball_info, frame, ball_type=1):
        try:
            x, y, diameter = ball_info
            measurement = np.array([[x], [y], [diameter]], dtype=np.float32)
            
            _ = self.kalman.predict()
            
            if not (0 <= x < self.width and 0 <= y < self.height and diameter > 0):
                self.miss_count += 1
                if self.miss_count > 5:
                    self.kalman.errorCovPost = np.eye(6, dtype=np.float32) * 100
                    self.last_prediction = None
                    return 0, 0, 0, 0, 0, frame
            else:
                self.miss_count = 0
                self.kalman.correct(measurement)
            
            state = self.kalman.statePost
            smooth_x = state[0, 0]
            smooth_y = state[1, 0]
            smooth_diameter = state[4, 0]
            smooth_distance = (self.real_diameter * self.focal_length) / max(smooth_diameter, 1e-6)
            
            if smooth_distance < 0.20:
                self.x_history.clear()
                self.y_history.clear()
                self.distance_history.clear()
                self.angle_history.clear()
                return 0, 0, 0, 0, 0, frame
            
            dx = smooth_x - self.width // 2
            smooth_angle = math.degrees(math.atan(dx / self.focal_length))
            
            self.x_history.append(smooth_x)
            self.y_history.append(smooth_y)
            self.distance_history.append(smooth_distance)
            self.angle_history.append(smooth_angle)
            
            if len(self.x_history) > self.smoothing_window:
                self.x_history.pop(0)
                self.y_history.pop(0)
                self.distance_history.pop(0)
                self.angle_history.pop(0)
            
            smooth_x = sum(self.x_history) / len(self.x_history)
            smooth_y = sum(self.y_history) / len(self.y_history)
            smooth_distance = sum(self.distance_history) / len(self.distance_history)
            smooth_angle = sum(self.angle_history) / len(self.angle_history)
            
            self.last_prediction = (smooth_x, smooth_y, smooth_distance, smooth_angle)
            
            if self.show_display or self.record_video:
                self._draw_debug_info(frame, int(smooth_x), int(smooth_y), smooth_distance, smooth_angle)
            
            return ball_type, smooth_x, smooth_y, smooth_distance, smooth_angle, frame
        except Exception as e:
            print("指标计算异常:", e)
            return 0, 0, 0, 0, 0, frame

    def _draw_debug_info(self, frame, x, y, distance, angle):
        center_x = self.width // 2
        center_y = self.height // 2
        cv2.line(frame, (center_x, 0), (center_x, self.height), (100, 100, 100), 1)
        cv2.line(frame, (0, center_y), (self.width, center_y), (100, 100, 100), 1)
        cv2.circle(frame, (x, y), 10, (0, 255, 0), 2)
        cv2.line(frame, (center_x, center_y), (x, y), (0, 0, 255), 2)
        cv2.putText(frame, f"Dist: {distance:.2f}m", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(frame, f"Angle: {angle:.1f} deg", (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(frame, f"FPS: {self.fps:.2f}", (10, 110),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.line(frame, (0, self.roi_bottom), (self.width, self.roi_bottom), (255, 0, 0), 2)

    def detect_and_display(self):
        self.last_frame = None
        self.last_update_time = time.time()
        try:
            while True:
                start_time = time.time()
                ret, frame = self.cap.read()
                if not ret:
                    print("摄像头读取失败")
                    break
                
                if self.last_frame is not None and np.array_equal(frame, self.last_frame):
                    print("[WARN] Frame frozen, skipping detection")
                    time.sleep(0.01)
                    continue
                self.last_frame = frame.copy()
                
                if self.show_display:
                    self.update_settings()
                    orange_mask_disp = self._process_orange(frame[self.roi_top:self.roi_bottom, 0:self.width])
                    white_mask_disp = self._process_white(frame[self.roi_top:self.roi_bottom, 0:self.width])
                    cv2.imshow("Orange Mask", orange_mask_disp)
                    cv2.imshow("White Mask", white_mask_disp)
                
                status, x, y, distance, angle, processed_frame = self.detect_balls(frame)
                
                self.frame_count += 1
                elapsed_time = time.time() - self.start_time
                self.fps = self.frame_count / elapsed_time if elapsed_time > 0 else 0.0
                
                if status:
                    print(f"位置: ({x:.2f}, {y:.2f}), 距离: {distance:.2f}米, 角度: {angle:.1f}度, FPS: {self.fps:.2f}")
                    self.last_update_time = time.time()
                else:
                    if time.time() - self.last_update_time > 1.0:
                        print("[INFO] No ball detected for 1s, resetting")
                        self.last_valid_result = (0, 0, 0, 0, 0, frame)
                
                if self.show_display and processed_frame is not None and processed_frame.size > 0:
                    cv2.imshow("Frame", processed_frame)
                    cv2.resizeWindow("Frame", self.width, self.height)
                
                if self.record_video and self.video_writer is not None and processed_frame is not None and processed_frame.size > 0:
                    self.video_writer.write(processed_frame)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        finally:
            self.cap.release()
            if self.video_writer is not None:
                self.video_writer.release()
            cv2.destroyAllWindows()

if __name__ == "__main__":
    detector = BallDetector(show_display=False, record_video=False)
    detector.detect_and_display()