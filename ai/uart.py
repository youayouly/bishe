#!/usr/bin/env python3
import serial
import time
import cv2
from detect import BallDetector
from collections import deque

ser = serial.Serial('/dev/ttyUSB0', 230400, timeout=1)  # 更新为 230400

def send_data(ball_type, x, y, distance, angle):
    data = f"${int(ball_type)},{int(x)},{int(y)},{int(distance * 1000)},{int(angle * 100)}\n"
    ser.write(data.encode('ascii'))
    print(f"[SENT] {data.strip()}")

def receive_response():
    # 非阻塞读取，避免等待
    if ser.in_waiting > 0:
        try:
            response = ser.readline().decode('utf-8', errors='replace').strip()
            if response:
                print(f"[UART] {response}")
        except Exception as e:
            print("解码异常:", e)

def main():
    # 启用视频录制
    detector = BallDetector(show_display=False, record_video=True, focal_length=275, real_diameter=0.04)
    
    # 可调节的发送间隔（每多少帧发送一次）
    SEND_INTERVAL = 5  # 默认每 5 帧发送一次，可改为任意值
    
    # 用于存储检测结果的队列
    result_buffer = deque(maxlen=SEND_INTERVAL)  # 缓冲区大小与发送间隔一致
    
    try:
        while True:
            start_time = time.time()
            ret, frame = detector.cap.read()
            if not ret or frame is None:
                print("摄像头读取失败")
                continue
            
            # 检测
            detect_start = time.time()
            result = detector.detect_balls(frame)
            detect_time = time.time() - detect_start
            
            if len(result) == 6:
                ball_type, x, y, distance, angle, processed_frame = result
                print(f"实时检测 - 位置: ({x:.2f}, {y:.2f}), 距离: {distance:.2f}米, 角度: {angle:.1f}度, DetectTime: {detect_time:.3f}s")
                
                # 将检测结果存入缓冲区
                result_buffer.append((ball_type, x, y, distance, angle))
                
                # 每 SEND_INTERVAL 帧发送一次平均值
                if detector.frame_count % SEND_INTERVAL == 0 and len(result_buffer) > 0:
                    # 计算平均值
                    avg_ball_type = max(set([r[0] for r in result_buffer]), key=[r[0] for r in result_buffer].count)  # 众数
                    avg_x = sum(r[1] for r in result_buffer) / len(result_buffer)
                    avg_y = sum(r[2] for r in result_buffer) / len(result_buffer)
                    avg_distance = sum(r[3] for r in result_buffer) / len(result_buffer)
                    avg_angle = sum(r[4] for r in result_buffer) / len(result_buffer)
                    
                    send_start = time.time()
                    send_data(avg_ball_type, avg_x, avg_y, avg_distance, avg_angle)
                    send_time = time.time() - send_start
                    print(f"SendTime: {send_time:.3f}s (Avg over {SEND_INTERVAL} frames)")
                    receive_response()
                
                # 写入视频
                if detector.record_video and detector.video_writer is not None and processed_frame is not None and processed_frame.size > 0:
                    detector.video_writer.write(processed_frame)
                    # 可选：打印写入确认
                    # print("已写入视频帧")
            
            # 计算 FPS
            detector.frame_count += 1
            elapsed_time = time.time() - detector.start_time
            detector.fps = detector.frame_count / elapsed_time if elapsed_time > 0 else 0
            # 可选：打印 FPS
            # print(f"FPS: {detector.fps:.2f}")
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
            # 控制帧率（可选，避免过快）
            sleep_time = max(0, (1/30) - (time.time() - start_time))
            time.sleep(sleep_time)
    except KeyboardInterrupt:
        print("\n程序终止")
    finally:
        detector.cap.release()
        if detector.video_writer is not None:
            detector.video_writer.release()
            print("视频写入器已释放，检查 /usr/src/ai/record/detection.avi")
        ser.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()