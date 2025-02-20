import os
import numpy as np
import cv2
import glob

def calibrate_camera(image_folder, checkerboard_size, square_size, calibration_file):
    # 准备对象点，如(0,0,0), (1,0,0), (2,0,0), ..., (w-1,h-1,0)
    objp = np.zeros((np.prod(checkerboard_size), 3), np.float32)
    objp[:, :2] = np.indices(checkerboard_size).T.reshape(-1, 2)
    objp *= square_size  # 单位：毫米

    # 存储对象点和图像点
    obj_points = []
    img_points = []

    # 获取图像文件路径
    images = glob.glob(os.path.join(image_folder, '*.png'))

    for image_path in images:
        img = cv2.imread(image_path)
        if img is None:
            print(f"无法读取图像: {image_path}")
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, checkerboard_size, None)

        if ret:
            obj_points.append(objp)
            img_points.append(corners)
            cv2.drawChessboardCorners(img, checkerboard_size, corners, ret)
            cv2.imshow('Chessboard', img)
            cv2.waitKey(500)

    cv2.destroyAllWindows()

    # 相机标定
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, gray.shape[::-1], None, None)
    if not ret:
        print("相机标定失败")
        return None, None

    # 打印标定结果
    print(f"相机矩阵:\n{mtx}")
    print(f"畸变系数:\n{dist}")

    # 保存到XML文件
    fs = cv2.FileStorage(calibration_file, cv2.FILE_STORAGE_WRITE)
    fs.write("camera_matrix", mtx)
    fs.write("distortion_coefficients", dist)
    fs.release()
    print(f"相机标定结果已保存到 {calibration_file}")
    
    return mtx, dist

def undistort_images(image_folder, output_folder, mtx, dist):
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    images = glob.glob(os.path.join(image_folder, '*.png'))

    for image_path in images:
        img = cv2.imread(image_path)
        if img is None:
            print(f"无法读取图像: {image_path}")
            continue

        h, w = img.shape[:2]
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
        dst = cv2.undistort(img, mtx, dist, None, new_camera_matrix)

        # 裁剪图像
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]

        output_path = os.path.join(output_folder, os.path.basename(image_path))
        cv2.imwrite(output_path, dst)
        print(f"已保存矫正后的图像: {output_path}")

if __name__ == '__main__':
    image_folder = r'./temp_level'  # 标定图像文件夹路径
    output_folder = r'./correct2'  # 矫正后图像保存文件夹路径
    checkerboard_size = (8, 5)  # 棋盘格内角点数（宽, 高）
    square_size = 25.0  # 棋盘格方格尺寸，单位：毫米
    calibration_file = '/usr/src/ai/calibration/valid/calibration2.xml'  # 保存标定结果的路径

    mtx, dist = calibrate_camera(image_folder, checkerboard_size, square_size, calibration_file)
    if mtx is not None and dist is not None:
        undistort_images(image_folder, output_folder, mtx, dist)
