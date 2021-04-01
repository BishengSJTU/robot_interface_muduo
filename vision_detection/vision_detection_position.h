#ifndef ARUCO_DETECTION_H
#define ARUCO_DETECTION_H

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/QR>
#include <eigen3/Eigen/Geometry>
#include <chrono>
#include <string>
#include <zbar.h>
#include "config.h"
#include "Logging.h"
#include "LogFile.h"

#define VISIONEXITNO 2

extern "C" {
#include "apriltag_pose.h"
#include "apriltag.h"
#include "tag36h11.h"
#include "tag25h9.h"
#include "tag16h5.h"
#include "tagCircle21h7.h"
#include "tagCircle49h12.h"
#include "tagCustom48h12.h"
#include "tagStandard41h12.h"
#include "tagStandard52h13.h"
#include "common/getopt.h"
#include "common/homography.h"
}

class VisionDetection
{
private:
    // aruco二维码信息
    int dictionary_type_;
    int robot_dictionary_type_;
    double robot_marker_length_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;

    // 相机参数
    cv::Mat intrinsics_;
    cv::Mat distortion_;
    int image_width_;
    int image_height_;
    int image_fps_;
    double image_brightness_;
    double image_contrast_;
    double image_saturation_;
    double image_hue_;

    double coding_time_wait_;
    double qr_time_wait_;
    int cap_source_;
    std::string coding_type_;

public:
    VisionDetection(const Config &config);
    ~VisionDetection();
    // 获取目标Aruco码的姿态，并对停层位置进行判断
    bool GetPose(const int &index, cv::Mat &pose, int times);
    // Apriltag检测
    void Apriltag_Detection(cv::Mat &input_image, std::vector<std::vector<cv::Point2f>> &corners, std::vector<int> &ids);
    // 判断目标QR码是否在视野中
    std::vector<int> QRCodeInPicture();
};

#endif