#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include <tuple>
#include <algorithm>

class BallDetector {
public:
    BallDetector(bool showDisplay = false, bool recordVideo = false, 
                 double focalLength = 554.26, double realDiameter = 0.04)
      : showDisplay_(showDisplay), recordVideo_(recordVideo),
        focalLength_(focalLength), realDiameter_(realDiameter)
    {
        // 打开摄像头
        cap_.open(0);
        if (!cap_.isOpened()) {
            std::cerr << "无法打开摄像头!\n";
            exit(-1);
        }
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        cap_.set(cv::CAP_PROP_FPS, 30);

        // 默认标定参数（可通过 FileStorage 加载 calib 文件）
        cameraMatrix_ = (cv::Mat_<double>(3,3) << focalLength_, 0, 320,
                                                  0, focalLength_, 240,
                                                  0, 0, 1);
        distortionCoeffs_ = cv::Mat::zeros(1, 5, CV_64F);

        // 参数设定（这里采用硬编码阈值，可根据实际情况调整）
        orangeLower_ = cv::Scalar(5, 100, 100);
        orangeUpper_ = cv::Scalar(15, 255, 255);

        whiteLower_ = 200;
        whiteUpper_ = 255;
        whiteBlur_ = 7;

        orangeIterOpen_ = 1;
        orangeIterClose_ = 1;
        whiteIterOpen_ = 1;
        whiteIterClose_ = 1;
        minRadius_ = 5;
        maxRadius_ = 50;
        minCircularity_ = 0.7;
        glareThresh_ = 200;

        kernel_ = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5));

        frameCount_ = 0;
        startTime_ = cv::getTickCount();

        if(recordVideo_) {
            videoWriter_.open("C:/usr/src/ai/record/detection.avi", 
                cv::VideoWriter::fourcc('X','V','I','D'), 8, cv::Size(640,480));
            if (!videoWriter_.isOpened()) {
                std::cerr << "无法打开视频写入器!\n";
            }
        }
    }
    
    void run() {
        cv::Mat frame, processedFrame;
        while (true) {
            if (!cap_.read(frame)) break;

            int status, x, y;
            double distance, angle;
            std::tie(status, x, y, distance, angle, processedFrame) = detectBalls(frame);
            
            frameCount_++;
            double elapsedTime = (cv::getTickCount() - startTime_) / cv::getTickFrequency();
            double fps = frameCount_ / elapsedTime;
            
            if (status == 1) {
                std::cout << "位置: (" << x << ", " << y << "), 距离: " 
                          << distance << "米, 角度: " << angle << "度, FPS: " 
                          << fps << "\n";
            }
            
            if (showDisplay_) {
                cv::imshow("Frame", processedFrame);
                if (cv::waitKey(1) == 'q') break;
            }
            
            if(recordVideo_ && videoWriter_.isOpened()) {
                videoWriter_.write(processedFrame);
            }
        }
        
        cap_.release();
        if(videoWriter_.isOpened())
            videoWriter_.release();
        cv::destroyAllWindows();
    }
    
private:
    // 返回值依次为 status, x, y, distance, angle, 以及带调试信息的帧
    std::tuple<int, int, int, double, double, cv::Mat> detectBalls(const cv::Mat &frame) {
        cv::Mat undistorted;
        cv::undistort(frame, undistorted, cameraMatrix_, distortionCoeffs_);
        
        // 复用灰度图计算 HSV 中的 V 通道
        cv::Mat gray;
        cv::cvtColor(undistorted, gray, cv::COLOR_BGR2GRAY);
        
        cv::Mat hsv;
        cv::cvtColor(undistorted, hsv, cv::COLOR_BGR2HSV);
        std::vector<cv::Mat> hsvChannels;
        cv::split(hsv, hsvChannels);
        hsvChannels[2] = gray;
        cv::merge(hsvChannels, hsv);
        cv::Mat frameModified;
        cv::cvtColor(hsv, frameModified, cv::COLOR_HSV2BGR);
        
        cv::Mat glareFree = glareSuppression(frameModified);
        
        // 分别处理橙色和白色小球（若需要可并行化）
        cv::Mat orangeMask = processOrange(glareFree);
        cv::Mat whiteMask = processWhite(glareFree);
        
        std::vector<cv::Vec3f> orangeBalls = findTarget(orangeMask);
        std::vector<cv::Vec3f> whiteBalls = findTarget(whiteMask);
        
        if (!orangeBalls.empty()) {
            return calculateMetrics(orangeBalls[0], frame);
        } else if (!whiteBalls.empty()) {
            return calculateMetrics(whiteBalls[0], frame);
        } else {
            return std::make_tuple(0, 0, 0, 0.0, 0.0, frame);
        }
    }
    
    cv::Mat processOrange(const cv::Mat &frame) {
        cv::Mat hsv, mask;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, orangeLower_, orangeUpper_, mask);
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel_, cv::Point(-1,-1), orangeIterOpen_);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel_, cv::Point(-1,-1), orangeIterClose_);
        return mask;
    }
    
    cv::Mat processWhite(const cv::Mat &frame) {
        cv::Mat gray, blurred, mask;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray, blurred, cv::Size(whiteBlur_, whiteBlur_), 0);
        cv::threshold(blurred, mask, whiteLower_, whiteUpper_, cv::THRESH_BINARY);
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel_, cv::Point(-1,-1), whiteIterOpen_);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel_, cv::Point(-1,-1), whiteIterClose_);
        return mask;
    }
    
    // 返回 Vec3f：x, y, 直径（2 * radius）
    std::vector<cv::Vec3f> findTarget(const cv::Mat &mask) {
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        std::vector<cv::Vec3f> valid;
        for (const auto &cnt : contours) {
            double area = cv::contourArea(cnt);
            if (area < 100)
                continue;
            double perimeter = cv::arcLength(cnt, true);
            if (perimeter == 0)
                continue;
            double circularity = (4 * CV_PI * area) / (perimeter * perimeter);
            cv::Point2f center;
            float radius;
            cv::minEnclosingCircle(cnt, center, radius);
            if (radius >= minRadius_ && radius <= maxRadius_ && circularity >= minCircularity_) {
                valid.push_back(cv::Vec3f(center.x, center.y, radius * 2));
            }
        }
        return valid;
    }
    
    std::tuple<int, int, int, double, double, cv::Mat> calculateMetrics(const cv::Vec3f &ballInfo, const cv::Mat &frame) {
        int x = static_cast<int>(ballInfo[0]);
        int y = static_cast<int>(ballInfo[1]);
        float diameter = ballInfo[2];
        if(diameter <= 0 || x < 0 || x >= 640 || y < 0 || y >= 480)
            return std::make_tuple(0, 0, 0, 0.0, 0.0, frame);
        double distance = (realDiameter_ * focalLength_) / std::max(diameter, 1e-6f);
        int dx = x - 320;
        double angle = focalLength_ != 0 ? std::atan(dx / focalLength_) * 180.0 / CV_PI : 0;
        cv::Mat debugFrame = frame.clone();
        if(showDisplay_ || recordVideo_)
            drawDebugInfo(debugFrame, x, y, distance, angle);
        return std::make_tuple(1, x, y, distance, angle, debugFrame);
    }
    
    cv::Mat glareSuppression(const cv::Mat &frame) {
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        double avgBrightness = cv::mean(gray)[0];
        if (avgBrightness < 100)
            return frame;
        cv::Mat glareMask, result;
        cv::threshold(gray, glareMask, glareThresh_, 255, cv::THRESH_BINARY);
        cv::inpaint(frame, glareMask, result, 3, cv::INPAINT_TELEA);
        return result;
    }
    
    void drawDebugInfo(cv::Mat &frame, int x, int y, double distance, double angle) {
        cv::line(frame, cv::Point(320, 0), cv::Point(320, 480), cv::Scalar(100,100,100), 1);
        cv::line(frame, cv::Point(0, 240), cv::Point(640, 240), cv::Scalar(100,100,100), 1);
        cv::circle(frame, cv::Point(x,y), 10, cv::Scalar(0,255,0), 2);
        cv::line(frame, cv::Point(320,240), cv::Point(x,y), cv::Scalar(0,0,255), 2);
        cv::putText(frame, cv::format("Dist: %.2f m", distance), cv::Point(10,30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,255,255), 2);
        cv::putText(frame, cv::format("Angle: %.1f deg", angle), cv::Point(10,70),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,255,255), 2);
    }
    
    cv::VideoCapture cap_;
    cv::VideoWriter videoWriter_;
    bool showDisplay_;
    bool recordVideo_;
    double focalLength_, realDiameter_;
    cv::Mat cameraMatrix_, distortionCoeffs_;
    cv::Scalar orangeLower_, orangeUpper_;
    int whiteLower_, whiteUpper_;
    int whiteBlur_;
    int orangeIterOpen_, orangeIterClose_;
    int whiteIterOpen_, whiteIterClose_;
    int minRadius_, maxRadius_;
    double minCircularity_;
    int glareThresh_;
    cv::Mat kernel_;
    
    int frameCount_;
    int64 startTime_;
};

int main() {
    // 设为 true 可显示检测窗口及打印调试信息
    BallDetector detector(true, true);
    detector.run();
    return 0;
}