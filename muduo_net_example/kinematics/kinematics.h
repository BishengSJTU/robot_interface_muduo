#ifndef KINEMATICS_H
#define KINEMATICS_H
#include "robot_client_tcp.h"
#include "pose_transform.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/QR>
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

//　各个矩阵输出调试
#define MATRIX_DEBUG false

//姿态调节函数
void getEulerRPY(const Eigen::Matrix3d &rotation_matrix,double& roll, double& pitch, double& yaw);

void GetTargetCart(RobotClient& robot_client,
                   const Eigen::MatrixXd& eye_hand,
                   const cv::Mat & pose,
                   const PoseTransform& target_pose_transform,
                   std::vector<float>& cart);

void Coordinate2Matrix(const std::vector<float> &coordinate,
                       Eigen::MatrixXd& T);



#endif