#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <dirent.h>
#include <sys/types.h>
#include <cstring>

int main() {
    // 定义棋盘格尺寸和每个方格的实际尺寸（单位：毫米）
    cv::Size checkerboard_size(8, 5);
    float square_size = 26.0f;

    // 获取图像文件路径
    std::string directory = "/usr/src/ai/calibration/temp_brown/";
    std::vector<std::string> image_paths;

    DIR* dir = opendir(directory.c_str());
    if (dir == nullptr) {
        std::cerr << "无法打开目录: " << directory << std::endl;
        return 1;
    }

    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        std::string filename = entry->d_name;
        if (filename.size() > 4 && filename.substr(filename.size() - 4) == ".jpg") {
            image_paths.push_back(directory + filename);
        }
    }

    closedir(dir);

    // 输出找到的图像文件路径
    std::cout << "找到的图像文件: ";
    for (const auto& path : image_paths) {
        std::cout << path << " ";
    }
    std::cout << std::endl;

    // 准备标定板的世界坐标点
    std::vector<std::vector<cv::Point3f>> obj_points_list;
    std::vector<std::vector<cv::Point2f>> img_points_list;
    std::vector<cv::Point3f> objp;
    for (int i = 0; i < checkerboard_size.height; ++i) {
        for (int j = 0; j < checkerboard_size.width; ++j) {
            objp.push_back(cv::Point3f(j * square_size, i * square_size, 0.0f));
        }
    }

    // 遍历图像文件，检测角点
    for (const auto& image_path : image_paths) {
        cv::Mat img = cv::imread(image_path);
        if (img.empty()) {
            std::cerr << "无法读取图像: " << image_path << std::endl;
            continue;
        }

        // 转换为灰度图像
        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

        // 检测棋盘格角点
        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(gray, checkerboard_size, corners,
                                               cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE |
                                               cv::CALIB_CB_FILTER_QUADS);
        if (found) {
            // 亚像素级精确化角点位置
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.1));

            obj_points_list.push_back(objp);
            img_points_list.push_back(corners);

            // 绘制角点
            cv::drawChessboardCorners(img, checkerboard_size, corners, found);
        } else {
            std::cerr << "未找到角点: " << image_path << std::endl;
        }

        // 显示图像
        cv::imshow("Chessboard", img);
        cv::waitKey(500); // 等待500毫秒
    }

    cv::destroyAllWindows();

    // 相机标定
    cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat dist_coeffs = cv::Mat::zeros(8, 1, CV_64F);
    std::vector<cv::Mat> rvecs, tvecs;
    double rms = cv::calibrateCamera(obj_points_list, img_points_list, cv::Size(640, 480), camera_matrix,
                                    dist_coeffs, rvecs, tvecs);
    std::cout << "标定均方根误差: " << rms << std::endl;

    // 输出相机内参和畸变系数
    std::cout << "相机内参矩阵: " << std::endl << camera_matrix << std::endl;
    std::cout << "畸变系数: " << std::endl << dist_coeffs << std::endl;

    return 0;
}
