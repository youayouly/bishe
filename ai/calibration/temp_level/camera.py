import os
import sys
import locale

# 必须在导入其他库前设置环境变量
os.environ["LC_ALL"] = "C.UTF-8"
os.environ["LANG"] = "C.UTF-8"
os.environ["LANGUAGE"] = "C.UTF-8"

# 强制设置 Locale
#try:
#    locale.setlocale(locale.LC_ALL, "C.UTF-8")
#except locale.Error as e:
#    print(f"Locale 设置失败: {e}")
#    sys.exit(1)

# 验证 Locale
#print("当前 Locale 配置:")
#print(f"LC_ALL: {os.environ.get('LC_ALL', '未设置')}")
#print(f"LANG: {os.environ.get('LANG', '未设置')}")
#print(f"LANGUAGE: {os.environ.get('LANGUAGE', '未设置')}")
#print(f"Python locale: {locale.setlocale(locale.LC_ALL)}")

import cv2
import time

# 配置参数
CHECKERBOARD_SIZE = (7, 7)
SAVE_DIR = "/usr/src/ai/calibration/temp_level"
MAX_IMAGES = 20
INTERVAL_SEC = 3

def auto_capture_checkerboard():
    os.makedirs(SAVE_DIR, exist_ok=True)
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    count = 0
    last_capture = 0
    
    try:
        while count < MAX_IMAGES:
            ret, frame = cap.read()
            if not ret:
                print("摄像头读取失败")
                break
            
            # 复制原始帧用于保存
            original_frame = frame.copy()
            
            # 检测棋盘格
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            ret_cb, corners = cv2.findChessboardCorners(
                gray, CHECKERBOARD_SIZE,
                cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK
            )
            
            if ret_cb and (time.time() - last_capture) > INTERVAL_SEC:
                # 保存原始帧（不含绘制内容）
                filename = os.path.join(SAVE_DIR, f"checker_{count:03d}.jpg")
                cv2.imwrite(filename, original_frame)
                print(f"已保存: {filename}")
                count += 1
                last_capture = time.time()
                
                # 提高角点精度（仅用于显示，不影响保存）
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
                cv2.drawChessboardCorners(frame, CHECKERBOARD_SIZE, corners, ret_cb)
            
            # 在显示帧上添加文字（不影响保存的原始帧）
            #cv2.putText(frame, f"Captured: {count}/{MAX_IMAGES}", (10, 30),
                        #cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            #cv2.putText(frame, "Press 'q' to quit", (10, 60),
                        #cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            cv2.imshow("Auto Capture", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    except KeyboardInterrupt:
        print("用户中断，程序退出")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print(f"采集完成，图片保存在 {SAVE_DIR}")

if __name__ == "__main__":
    auto_capture_checkerboard()