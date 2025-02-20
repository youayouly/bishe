import cv2

calibration_file = '/usr/src/ai/calibration/valid/calibration.xml'
fs = cv2.FileStorage(calibration_file, cv2.FILE_STORAGE_READ)

if not fs.isOpened():
    print(f"无法打开文件: {calibration_file}")
else:
    print(f"成功打开文件: {calibration_file}")

camera_matrix = fs.getNode('cameraMatrix').mat()
dist_coeffs = fs.getNode('distCoeffs').mat()
fs.release()

if camera_matrix is None:
    print("未能读取相机矩阵")
else:
    print("成功读取相机矩阵")
