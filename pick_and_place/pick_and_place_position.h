#ifndef PICK_AND_PLACE_H
#define PICK_AND_PLACE_H

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <unistd.h>

#include "config.h"
#include "robot_client_tcp.h"
#include "pose_transform.h"
#include "kinematics.h"
#include "motion_list.h"
#include "my_plc.h"
#include "vision_detection_position.h"
#include "Logging.h"
#include "LogFile.h"


#include <thread>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/QR>
#include <sstream>

class PickAndPlace
{
private:
    // 机器人调整时速度
    float adjust_move_speed;

    // 机器人移动到指定路点速度
    float fixed_joint_speed;

    // 最大调整次数
    int max_count_adjust;

    // 取放档案时视觉偏置数据
    float pick_place_offset_x;
    float pick_place_offset_y;
    float pick_place_offset_z;
    float pick_place_roll;
    float pick_place_pitch;
    float pick_place_yaw;

    //　手眼矩阵
    std::vector<double> eye_hand;
    Eigen::MatrixXd eye_hand_matrix;

    // 配置文件
    const Config config_;

    // JAKA机器人
    RobotClient robot_client_;

    // 动作列表
    MotionList motion_list_;

    // PLC动作
    MyPLC plc_;

    // 二维码
    VisionDetection vision_detection_;


public:
    PickAndPlace(const std::string &config_file_name, const std::string &motion_list_name);
    ~PickAndPlace();
    void InitializePickAndPlace();
    //　从档案盒中取
    bool PickCab(int cab_id, int position, bool& mechanical_error);
    //　放到档案盒中
    bool PlaceCab(int cab_id, int position, bool& mechanical_error);
    //　从窗口中取
    bool PickWindow(int position, bool& mechanical_error);
    //　向窗口中放
    bool PlaceWindow(int position,  bool& mechanical_error);
    //　从暂存架中取
    bool PickStorage(int position, bool& mechanical_error);
    //　向暂存架中放
    bool PlaceStorage(int position,  bool& mechanical_error);
    // 机器人收起
    void Contraction();
    // 机器人伸展开
    void Stretch();
    // PLC动作
    bool PLCAction(int command);
    // 查询PLC状态
    void PLCState();
};

#endif