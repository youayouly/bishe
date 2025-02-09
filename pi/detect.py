import cv2
import numpy as np
import time
import math

class BallDetector:
    def __init__(self, show_display=False, focal_length=554.26, real_diameter=0.04):
        # 相机初始化
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        # 物理参数
        self.focal_length = focal_length     # 相机焦距（像素）
        self.real_diameter = real_diameter   # 实际球体直径（米）
        
        # 图像处理参数
        self.show_display = show_display
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        
        # HSV阈值默认值
        self.orange_lower = np.array([0, 100, 100])
        self.orange_upper = np.array([22, 255, 255])
        self.white_lower = np.array([74, 0, 107])
        self.white_upper = np.array([132, 38, 255])
        self.glare_thresh = 234
        
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
        cv2.createTrackbar("White S1", "HSV Adjust", 0, 255, lambda x: x)
        cv2.createTrackbar("White S2", "HSV Adjust", 38, 255, lambda x: x)
        cv2.createTrackbar("White V1", "HSV Adjust", 107, 255, lambda x: x)
        cv2.createTrackbar("White V2", "HSV Adjust", 255, 255, lambda x: x)
        cv2.createTrackbar("Glare Thresh", "HSV Adjust", 234, 255, lambda x: x)

    def detect_balls(self, frame):
        """主检测函数（增加异常处理）"""
        try:
            if frame is None:
                return 0, 0, 0, 0, 0, None  # 明确返回有效元组
    
            # 反光抑制处理
            processed_frame = self._glare_suppression(frame)
            
            # 颜色检测
            orange_mask, white_mask = self._color_detection(processed_frame)
            
            # 检测球体（增加空列表判断）
            orange_balls = self._find_balls(orange_mask) or []
            white_balls = self._find_balls(white_mask) or []
            
            # 优先返回橙色球（增加有效性检查）
            if len(orange_balls) > 0:
                return self._calculate_metrics(orange_balls[0], frame)
            elif len(white_balls) > 0:
                return self._calculate_metrics(white_balls[0], frame)
            else:
                return 0, 0, 0, 0, 0, frame  # 确保始终返回6元素元组
    
        except Exception as e:
            print(f"检测异常: {str(e)}")
            return 0, 0, 0, 0, 0, frame  # 异常时仍返回有效结构

    def _glare_suppression(self, frame):
        """反光抑制处理"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, glare_mask = cv2.threshold(gray, self.glare_thresh, 255, cv2.THRESH_BINARY)
        return cv2.inpaint(frame, glare_mask, 3, cv2.INPAINT_TELEA)

    def _color_detection(self, frame):
        """颜色分割"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        orange_mask = cv2.inRange(hsv, self.orange_lower, self.orange_upper)
        white_mask = cv2.inRange(hsv, self.white_lower, self.white_upper)
        
        # 形态学优化
        orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_OPEN, self.kernel, 2)
        orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_CLOSE, self.kernel, 3)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, self.kernel, 3)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, self.kernel, 5)
        return orange_mask, white_mask

    def _find_balls(self, mask, min_radius=10, max_radius=100):
        """轮廓检测"""
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
                valid_balls.append( (x, y, radius*2) )  # 存储直径
        return valid_balls

    def _calculate_metrics(self, ball_info, frame):
        """计算距离和角度（增加零除保护）"""
        try:
            x, y, diameter = ball_info
            x, y = int(x), int(y)
            
            # 有效性校验（直径必须>0）
            if diameter <= 0 or not (0 <= x < 640 and 0 <= y < 480):
                return 0, 0, 0, 0, 0, frame
            
            # 距离计算（防止零除错误）
            distance = (self.real_diameter * self.focal_length) / max(diameter, 1e-6)
            
            # 角度计算（增加异常捕获）
            dx = x - 320  # 以图像中心为基准
            angle = math.degrees(math.atan(dx / self.focal_length)) if self.focal_length != 0 else 0
            
            # 绘制调试信息
            if self.show_display and frame is not None:
                self._draw_debug_info(frame, x, y, distance, angle)
            
            return 1, x, y, distance, angle, frame  # 确保返回6元素
    
        except Exception as e:
            print(f"指标计算异常: {str(e)}")
            return 0, 0, 0, 0, 0, frame
        
        
    def _draw_debug_info(self, frame, x, y, distance, angle):
        """绘制调试信息"""
        # 绘制中心线
        cv2.line(frame, (320, 0), (320, 480), (100, 100, 100), 1)
        cv2.line(frame, (0, 240), (640, 240), (100, 100, 100), 1)
        
        # 绘制检测结果
        cv2.circle(frame, (x, y), 10, (0, 255, 0), 2)
        cv2.line(frame, (320, 240), (x, y), (0, 0, 255), 2)
        
        # 显示数值
        cv2.putText(frame, f"Dist: {distance:.2f}m", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(frame, f"Angle: {angle:.1f} deg", (10, 70), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)


    def update_settings_from_trackbars(self):
        """从轨迹栏更新参数"""
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
        
      # 在detect.py的检测逻辑中
#    def _calculate_metrics(self, ball_info, frame):
#        x, y, diameter = ball_info
#        x, y = int(x), int(y)
#        
#        # 校验坐标合法性
#        if not (0 <= x < 640 and 0 <= y < 480):
#            return 0, 0, 0, 0, 0, frame  # 返回无效数据
#        
#        # 其余计算逻辑...

    def release(self):
        """释放资源"""
        self.cap.release()
        if self.show_display:
            cv2.destroyAllWindows()

if __name__ == "__main__":
    detector = BallDetector(show_display=True)
    try:
        while True:
            ret, frame = detector.cap.read()
            if not ret:
                break
            detector.update_settings_from_trackbars()
            ball_type, x, y, distance, angle, frame = detector.detect_balls(frame)
            cv2.imshow("Frame", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            time.sleep(0.1)
    finally:
        detector.release()
        cv2.destroyAllWindows()