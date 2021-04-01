#include "kinematics.h"

void getEulerRPY(const Eigen::Matrix3d &rotation_matrix,double& roll, double& pitch, double& yaw) {

    if (fabs(rotation_matrix(2, 0)) >= 1) {
        yaw = 0;

        if (rotation_matrix(2, 0) < 0) {
            double delta = atan2(rotation_matrix(0, 1), rotation_matrix(0, 2));
            pitch = M_PI / (2.0);
            roll = delta;
        } else
        {
            double delta = atan2(-rotation_matrix(0, 1), -rotation_matrix(0, 2));
            pitch = -M_PI / (2.0);
            roll = delta;
        }
    } else {
        pitch = -asin(rotation_matrix(2, 0));
        roll = atan2(rotation_matrix(2, 1) / cos(pitch),
                     rotation_matrix(2, 2) / cos(pitch));
        yaw = atan2(rotation_matrix(1, 0) / cos(pitch),
                    rotation_matrix(0, 0) / cos(pitch));
    }
}


void GetTargetCart(RobotClient& robot_client,
                   const Eigen::MatrixXd& eye_hand,
                   const cv::Mat & pose,
                   const PoseTransform& target_pose_transform,
                   std::vector<float>& cart) {

    Eigen::MatrixXd T_Gripper_Camera = eye_hand;

    if(MATRIX_DEBUG) {
        std::cout << "T_Gripper_Camera:" << std::endl;
        std::cout << T_Gripper_Camera << std::endl;
    }

    Eigen::MatrixXd T_Camera_Gripper;
    T_Camera_Gripper = T_Gripper_Camera.inverse();

    if(MATRIX_DEBUG) {
        std::cout << "T_Camera_Gripper:" << std::endl;
        std::cout << T_Camera_Gripper << std::endl;
    }

    Eigen::MatrixXd T_Base_Gripper;
    std::vector<float> curr_cart;
    std::vector<float> curr_joint;
    robot_client.GetRobotPose(curr_joint, curr_cart);
    Coordinate2Matrix(curr_cart, T_Base_Gripper);

    if(MATRIX_DEBUG) {
        std::cout << "T_Base_Gripper:" << std::endl;
        std::cout << T_Base_Gripper << std::endl;
    }

    Eigen::MatrixXd T_Box_Target;
    std::vector<float> target_pose;
    target_pose.push_back(target_pose_transform.offset_x);
    target_pose.push_back(target_pose_transform.offset_y);
    target_pose.push_back(target_pose_transform.offset_z);
    target_pose.push_back(target_pose_transform.roll);
    target_pose.push_back(target_pose_transform.pitch);
    target_pose.push_back(target_pose_transform.yaw);
    Coordinate2Matrix(target_pose, T_Box_Target);

    if(MATRIX_DEBUG) {
        std::cout << "T_Box_Target:" << std::endl;
        std::cout << T_Box_Target << std::endl;
    }

    Eigen::MatrixXd T_Camera_Box;
    T_Camera_Box.resize(4, 4);
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            T_Camera_Box(i, j) = pose.at<double>(i, j);
        }
    }
    std::cout << "T_Camera_Box:" << std::endl;
    std::cout << T_Camera_Box << std::endl;

    Eigen::MatrixXd T_Base_GripperTarget;
    T_Base_GripperTarget = T_Base_Gripper * T_Gripper_Camera * T_Camera_Box * T_Box_Target * T_Camera_Gripper;

    if(MATRIX_DEBUG) {
        std::cout << "T_Base_GripperTarget:" << std::endl;
        std::cout << T_Base_GripperTarget << std::endl;
    }

    Eigen::Matrix3d rotation_matrix;
    double roll, pitch, yaw;
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            rotation_matrix(i, j) = T_Base_GripperTarget(i, j);
        }
    }
    getEulerRPY(rotation_matrix,roll, pitch, yaw);

    roll = roll * 180 / M_PI;
    pitch = pitch * 180 / M_PI;
    yaw = yaw * 180 / M_PI;

    cart.clear();
    cart.push_back(T_Base_GripperTarget(0, 3));
    cart.push_back(T_Base_GripperTarget(1, 3));
    cart.push_back(T_Base_GripperTarget(2, 3));
    cart.push_back(roll);
    cart.push_back(pitch);
    cart.push_back(yaw);
}

//姿态[xyzrpy]转换为矩阵形式
void Coordinate2Matrix(const std::vector<float> &coordinate,
                       Eigen::MatrixXd& T) {
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix = Eigen::AngleAxisd(coordinate[5] * M_PI / 180, Eigen::Vector3d::UnitZ()) *
                      Eigen::AngleAxisd(coordinate[4] * M_PI / 180, Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(coordinate[3] * M_PI / 180, Eigen::Vector3d::UnitX());
    T.resize(4, 4);
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            T(i, j) = rotation_matrix(i, j);
        }
    }
    T(3, 0) = 0;
    T(3, 1) = 0;
    T(3, 2) = 0;
    T(3, 3) = 1;
    T(0, 3) = coordinate[0];
    T(1, 3) = coordinate[1];
    T(2, 3) = coordinate[2];
}