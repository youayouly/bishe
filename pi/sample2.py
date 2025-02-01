import cv2
import numpy as np
import time
import json

class TableTennisDetector:
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        self.set_camera_properties()
        self.setup_trackbars()
        self.setup_detection_params()
        self.load_config()  # 加载上次保存的参数

    def set_camera_properties(self):
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        time.sleep(1)  # 等待摄像头初始化

    def setup_trackbars(self):
        cv2.namedWindow("Control Panel", cv2.WINDOW_NORMAL)
        # 橙色球参数
        cv2.createTrackbar("Orange H1", "Control Panel", 0, 179, self.nothing)
        cv2.createTrackbar("Orange H2", "Control Panel", 22, 179, self.nothing)
        cv2.createTrackbar("Orange S", "Control Panel", 100, 255, self.nothing)
        cv2.createTrackbar("Orange V", "Control Panel", 100, 255, self.nothing)
        
        # 白色球参数
        cv2.createTrackbar("White S", "Control Panel", 30, 50, self.nothing)
        cv2.createTrackbar("White V", "Control Panel", 220, 255, self.nothing)
        
        # 通用参数
        cv2.createTrackbar("Glare", "Control Panel", 240, 255, self.nothing)
        cv2.createTrackbar("Min Radius", "Control Panel", 10, 100, self.nothing)
        cv2.createTrackbar("Max Radius", "Control Panel", 50, 200, self.nothing)

    def setup_detection_params(self):
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        self.fgbg = cv2.createBackgroundSubtractorMOG2(history=500, varThreshold=16)

    def nothing(self, x): pass

    def load_config(self, filename="config.json"):
        try:
            with open(filename) as f:
                params = json.load(f)
            for key, value in params.items():
                cv2.setTrackbarPos(key, "Control Panel", value)
            print(f"Loaded parameters from {filename}")
        except FileNotFoundError:
            print("No config file found, using default parameters")

    def save_config(self, filename="config.json"):
        params = {
            "Orange H1": cv2.getTrackbarPos("Orange H1", "Control Panel"),
            "Orange H2": cv2.getTrackbarPos("Orange H2", "Control Panel"),
            "Orange S": cv2.getTrackbarPos("Orange S", "Control Panel"),
            "Orange V": cv2.getTrackbarPos("Orange V", "Control Panel"),
            "White S": cv2.getTrackbarPos("White S", "Control Panel"),
            "White V": cv2.getTrackbarPos("White V", "Control Panel"),
            "Glare": cv2.getTrackbarPos("Glare", "Control Panel"),
            "Min Radius": cv2.getTrackbarPos("Min Radius", "Control Panel"),
            "Max Radius": cv2.getTrackbarPos("Max Radius", "Control Panel")
        }
        with open(filename, 'w') as f:
            json.dump(params, f)
        print(f"Saved parameters to {filename}")

    def process_frame(self, frame):
        # 反光抑制处理
        glare_thresh = cv2.getTrackbarPos("Glare", "Control Panel")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, glare_mask = cv2.threshold(gray, glare_thresh, 255, cv2.THRESH_BINARY)
        frame = cv2.inpaint(frame, glare_mask, 3, cv2.INPAINT_TELEA)

        # 背景减除（适用于白色地板）
        fg_mask = self.fgbg.apply(frame)
        frame = cv2.bitwise_and(frame, frame, mask=fg_mask)

        # 光照均衡
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
        l = clahe.apply(l)
        lab = cv2.merge((l,a,b))
        return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

    def detect_balls(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # 获取当前参数
        orange_h1 = cv2.getTrackbarPos("Orange H1", "Control Panel")
        orange_h2 = cv2.getTrackbarPos("Orange H2", "Control Panel")
        orange_s = cv2.getTrackbarPos("Orange S", "Control Panel")
        orange_v = cv2.getTrackbarPos("Orange V", "Control Panel")
        
        white_s = cv2.getTrackbarPos("White S", "Control Panel")
        white_v = cv2.getTrackbarPos("White V", "Control Panel")
        
        min_radius = cv2.getTrackbarPos("Min Radius", "Control Panel")
        max_radius = cv2.getTrackbarPos("Max Radius", "Control Panel")

        # 橙色球检测
        lower_orange = np.array([orange_h1, orange_s, orange_v])
        upper_orange = np.array([orange_h2, 255, 255])
        mask_orange = cv2.inRange(hsv, lower_orange, upper_orange)
        mask_orange = cv2.morphologyEx(mask_orange, cv2.MORPH_CLOSE, self.kernel, iterations=3)

        # 白色球检测
        lower_white = np.array([0, 0, white_v])
        upper_white = np.array([179, white_s, 255])
        mask_white = cv2.inRange(hsv, lower_white, upper_white)
        mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_OPEN, self.kernel, iterations=5)

        # 组合检测结果
        circles_orange = self.find_circles(mask_orange, min_radius, max_radius)
        circles_white = self.find_circles(mask_white, min_radius, max_radius)

        return circles_orange, circles_white, mask_orange, mask_white

    def find_circles(self, mask, min_radius, max_radius):
        circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, dp=1, minDist=50,
                                  param1=50, param2=30,
                                  minRadius=min_radius, maxRadius=max_radius)
        if circles is not None:
            return np.uint16(np.around(circles))
        return None

    def draw_results(self, frame, circles, color, label):
        if circles is not None:
            for ball in circles[0, :]:
                center = (ball[0], ball[1])
                radius = ball[2]
                cv2.circle(frame, center, radius, color, 3)
                cv2.putText(frame, f"{label} {radius}px", 
                           (ball[0]-60, ball[1]-radius-15),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    def run(self):
        while True:
            ret, frame = self.cap.read()
            if not ret: break

            processed_frame = self.process_frame(frame)
            orange, white, mask_orange, mask_white = self.detect_balls(processed_frame)
            
            # 绘制检测结果
            self.draw_results(frame, orange, (0, 165, 255), "Orange")
            self.draw_results(frame, white, (200, 200, 200), "White")
            
            # 显示处理过程
            display = cv2.hconcat([
                frame,
                cv2.cvtColor(mask_orange, cv2.COLOR_GRAY2BGR),
                cv2.cvtColor(mask_white, cv2.COLOR_GRAY2BGR)
            ])
            cv2.imshow("Detection Result", display)

            key = cv2.waitKey(1)
            if key == ord('s'):
                self.save_snapshot(frame, mask_orange, mask_white)
                self.save_config()
            elif key == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

    def save_snapshot(self, frame, mask1, mask2):
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        cv2.imwrite(f"frame_{timestamp}.jpg", frame)
        cv2.imwrite(f"mask_orange_{timestamp}.jpg", mask1)
        cv2.imwrite(f"mask_white_{timestamp}.jpg", mask2)
        print(f"Saved snapshots at {timestamp}")

if __name__ == "__main__":
    detector = TableTennisDetector()
    detector.run()