#include "vision_detection_position.h"

VisionDetection::VisionDetection(const Config &config)
{
    // 设置aruco相关检测参数和检测字典
    dictionary_type_ = cv::aruco::DICT_6X6_50;
    robot_dictionary_type_ = cv::aruco::DICT_6X6_50;
    detector_params_ = cv::aruco::DetectorParameters::create();
    detector_params_->markerBorderBits = 1;
    detector_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    dictionary_ = cv::aruco::getPredefinedDictionary(dictionary_type_);
    robot_marker_length_ = config.get<double>("robot_marker_length");

    // 配置相机采集相关参数
    intrinsics_ = cv::Mat::zeros(cv::Size(3,3), CV_32F);
    distortion_ = cv::Mat::zeros(cv::Size(5,1), CV_32F);
    std::vector<float> intrinsics_vector;
    std::vector<float> distortion_vector;
    intrinsics_vector = config.get<std::vector<float>>("intrinsics");
    distortion_vector = config.get<std::vector<float>>("distortion");
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            intrinsics_.at<float>(i, j) = intrinsics_vector[i * 3 + j];
        }
    }
    for(int i = 0; i < 5; i++)
        distortion_.at<float>(0, i) = distortion_vector[i];
    image_width_ = config.get<int>("image_width");
    image_height_ = config.get<int>("image_height");
    image_brightness_ = config.get<double>("image_brightness");
    image_contrast_ = config.get<double>("image_contrast");
    image_saturation_ = config.get<double>("image_saturation");
    image_hue_ = config.get<double>("image_hue");
    image_fps_ = config.get<int>("image_fps");
    coding_time_wait_ = config.get<double>("aruco_time_wait");
    qr_time_wait_ = config.get<double>("qr_time_wait");
    cap_source_ = config.get<int>("cap_source");
    coding_type_ = "apriltag";
}

VisionDetection::~VisionDetection()
{

}

bool VisionDetection::GetPose(const int &index, cv::Mat &pose, int times) {
    static int i = 0;
    i++;
    bool result = true;
    cv::Mat src_image;
    cv::Mat dst_image1;

    {
        cv::VideoCapture cap_;
        cap_ = cap_source_;
        if (!cap_.isOpened()) {
            std::cerr << "Error : Cannot open the camera" << std::endl;
            LOG_ERROR << "Error : Cannot open the camera";
            return false;
        }
        cap_.set(CV_CAP_PROP_FRAME_WIDTH, image_width_);
        cap_.set(CV_CAP_PROP_FRAME_HEIGHT, image_height_);
        cap_.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
        cap_.set(CV_CAP_PROP_BRIGHTNESS, image_brightness_);
        cap_.set(CV_CAP_PROP_CONTRAST, image_contrast_);
        if (times == 1) {
            cap_.set(CV_CAP_PROP_AUTOFOCUS, 0);
            time_t timestamp_;
            time_t time_start = time(&timestamp_);
            while (1) {
                cap_.read(src_image);
                time_t time_end = time(&timestamp_);
                if (time_end - time_start > coding_time_wait_ / 2.0)
                    break;
            }
            src_image.copyTo(dst_image1);
        } else {
            cap_.set(CV_CAP_PROP_AUTOFOCUS, 0);
            time_t timestamp_;
            time_t time_start = time(&timestamp_);
            while (1) {
                cap_.read(src_image);
                time_t time_end = time(&timestamp_);
                if (time_end - time_start > (coding_time_wait_ / 2.0))
                    break;
            }
            src_image.copyTo(dst_image1);
        }
    }
    // 所有检测结果
    std::vector<int> all_ids;
    std::vector<std::vector<cv::Point2f>> all_corners;
    std::vector<cv::Vec3d> all_rvecs;
    std::vector<cv::Vec3d> all_tvecs;

    // 目标检测结果
    std::vector<int> target_id;
    std::vector<std::vector<cv::Point2f>> target_corner;
    std::vector<cv::Vec3d> target_rvec;
    std::vector<cv::Vec3d> target_tvec;

    cv::Ptr<cv::aruco::Dictionary> dict = cv::aruco::getPredefinedDictionary(robot_dictionary_type_);

    if(coding_type_ == "apriltag") {
        Apriltag_Detection(src_image, all_corners, all_ids);
    }
    else if(coding_type_ == "aruco") {
        cv::aruco::detectMarkers(src_image, dict, all_corners, all_ids, this->detector_params_);
    }
    else {
        std::cerr << "码类别配置错误" << std::endl;
        exit(VISIONEXITNO);
    }


    int search_times;
    for (search_times = 0; search_times < all_ids.size(); search_times++) {
        if (index == all_ids[search_times])
            break;
    }
    // 如果目标id的marker不在视野中，再次捕捉画面进行检测
    if (search_times >= all_ids.size()) {
        result = false;
    } else {
        target_id.push_back(all_ids[search_times]);
        target_corner.push_back(all_corners[search_times]);
    }

    // 画出目标Aruco码的边缘
    cv::aruco::drawDetectedMarkers(dst_image1, target_corner, target_id, cv::Scalar(0, 255, 0));
    // 估计目标Aruco码的位姿
    cv::aruco::estimatePoseSingleMarkers(target_corner, robot_marker_length_, intrinsics_, distortion_,
                                         target_rvec, target_tvec);
    // 此次估计不成功，返回失败
    if (target_rvec.size() == 0 || target_tvec.size() == 0) {
        result = false;
    } else {
        cv::Mat cv_rotation_matrix(cv::Mat::zeros(cv::Size(3, 3), CV_64F));
        cv::Rodrigues(target_rvec[0], cv_rotation_matrix);

        cv_rotation_matrix.at<double>(0, 0) = -cv_rotation_matrix.at<double>(0, 0);
        cv_rotation_matrix.at<double>(1, 0) = -cv_rotation_matrix.at<double>(1, 0);
        cv_rotation_matrix.at<double>(2, 0) = -cv_rotation_matrix.at<double>(2, 0);

        cv_rotation_matrix.at<double>(0, 2) = -cv_rotation_matrix.at<double>(0, 2);
        cv_rotation_matrix.at<double>(1, 2) = -cv_rotation_matrix.at<double>(1, 2);
        cv_rotation_matrix.at<double>(2, 2) = -cv_rotation_matrix.at<double>(2, 2);

        cv::Mat pose_matrix(cv::Mat::zeros(cv::Size(4, 4), CV_64F));
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                pose_matrix.at<double>(i, j) = cv_rotation_matrix.at<double>(i, j);
            }
        }
        pose_matrix.at<double>(3, 0) = 0;
        pose_matrix.at<double>(3, 1) = 0;
        pose_matrix.at<double>(3, 2) = 0;
        pose_matrix.at<double>(3, 3) = 1;
        pose_matrix.at<double>(0, 3) = target_tvec[0][0];
        pose_matrix.at<double>(1, 3) = target_tvec[0][1];
        pose_matrix.at<double>(2, 3) = target_tvec[0][2];
        pose = pose_matrix.clone();

        Eigen::Matrix3d rotation_matrix;
        Eigen::AngleAxisd rotation_vector;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                rotation_matrix(i, j) = cv_rotation_matrix.at<double>(i, j);
            }
        }
        rotation_vector.fromRotationMatrix(rotation_matrix);
        target_rvec[0][0] = (rotation_vector.axis() * rotation_vector.angle())[0];
        target_rvec[0][1] = (rotation_vector.axis() * rotation_vector.angle())[1];
        target_rvec[0][2] = (rotation_vector.axis() * rotation_vector.angle())[2];
        cv::aruco::drawAxis(dst_image1, intrinsics_, distortion_, target_rvec[0], target_tvec[0], 20.0);

//        cv::imwrite("/home/xunjie/robot_interface1.1/log/aruco_" + std::to_string(i) + ".jpg", dst_image1);
        result = true;
    }

    return result;
}

// Apriltag检测
void VisionDetection::Apriltag_Detection(cv::Mat &input_image, std::vector<std::vector<cv::Point2f>> &corners, std::vector<int> &ids) {
    //apriltag 检测
    apriltag_family_t *tf = tag36h11_create();
    apriltag_detector_t *td = apriltag_detector_create();              //set some parameters
    apriltag_detector_add_family(td,
                                 tf);                              //set parttern recongnize of the specific tag family
    td->quad_decimate = 2.0;
    td->quad_sigma = 0.0;
    td->nthreads = 8;
    td->debug = 0;
    td->refine_edges = 1;
    cv::Mat gray, frame;
    frame = input_image.clone();
    apriltag_detection_info_t info;     // parameters of the camera calibrations 在这里把标定得到的四元参数输入到程序里
    info.tagsize = 0.022; //标识的实际尺寸
    info.fx = 2621.237970071161;
    info.fy = 2622.524469754527;
    info.cx = 1621.963519064815;
    info.cy = 1189.737798965657;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    image_u8_t im = {.width = gray.cols,
            .height = gray.rows,
            .stride = gray.cols,
            .buf = gray.data
    };
    zarray_t *detections = apriltag_detector_detect(td, &im);
    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        ids.push_back(det->id);
        std::vector <cv::Point2f> corner;
        cv::Point2f p0(det->p[3][0], det->p[3][1]);
        cv::Point2f p1(det->p[2][0], det->p[2][1]);
        cv::Point2f p2(det->p[1][0], det->p[1][1]);
        cv::Point2f p3(det->p[0][0], det->p[0][1]);
        corner.push_back(p0);
        corner.push_back(p1);
        corner.push_back(p2);
        corner.push_back(p3);

        corners.push_back(corner);
    }
    apriltag_detector_destroy(td);
}

std::vector<int> VisionDetection::QRCodeInPicture() {
    std::vector<int> results;
    cv::Mat src_image;
    cv::Mat grey_image;
    zbar::ImageScanner scanner;
    scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
    {
        cv::VideoCapture cap_;
        cap_ = 0;
        if (!cap_.isOpened()) {
            std::cerr << "Error : Cannot open the camera" << std::endl;
            return results;
        }
        cap_.set(CV_CAP_PROP_FRAME_WIDTH, image_width_);
        cap_.set(CV_CAP_PROP_FRAME_HEIGHT, image_height_);
        cap_.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
        cap_.set(CV_CAP_PROP_BRIGHTNESS, image_brightness_);
        cap_.set(CV_CAP_PROP_CONTRAST, image_contrast_);
        cap_.set(CV_CAP_PROP_AUTOFOCUS, 0);

        time_t timestamp_;
        time_t time_start = time(&timestamp_);
        while (1) {
            cap_.read(src_image);
            time_t time_end = time(&timestamp_);
            if (time_end - time_start > qr_time_wait_)
                break;
        }
    }
    int image_width_ = src_image.cols;
    int image_height_ = src_image.rows;
    cvtColor(src_image, grey_image, CV_BGR2GRAY);
    uchar *raw = (uchar *) grey_image.data;
    zbar::Image image(image_width_, image_height_, "Y800", raw, image_width_ * image_height_);
    scanner.scan(image);
    std::cout << "QR";

    for (zbar::Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {
        std::string message(symbol->get_data());
        int result = std::atoi(message.c_str());
        std::cout << "result：" << result;
        results.push_back(result);
    }
    std::cout << std::endl;
    return results;
}