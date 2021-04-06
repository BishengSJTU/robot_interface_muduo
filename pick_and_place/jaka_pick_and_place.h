#ifndef JAKA_PICK_AND_PLACE_H
#define JAKA_PICK_AND_PLACE_H

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <unistd.h>

#include "config.h"
#include "robot_client_tcp.h"
#include "pose_transform.h"
#include "kinematics.h"
#include "my_plc.h"
#include "vision_detection_position.h"
#include "Logging.h"
#include "LogFile.h"
#include "mapping_table.h"


#include <thread>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/QR>
#include <sstream>

#define CODEING_ID 0 // 0号位姿估计码
#define CAB_SPACE 44.2 // 档案柜中档案间隙
#define WINDOW_SPACE 42.8 // 窗口中档案间隙

class JAKAPickAndPlace
{
private:
    // 机器人调整时速度
    float adjust_joint_speed;

    // 机器人移动到指定路点速度
    float fixed_joint_speed;

    // 机器人移动到指定路点速度
    float fixed_linear_speed;

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
    const Config fixed_config_;
    const Config flexible_config_;

    // JAKA机器人
    RobotClient robot_client_;

    // PLC动作
    MyPLC plc_;

    // 二维码
    VisionDetection vision_detection_;


public:
    JAKAPickAndPlace(const std::string &config_file_path);
    ~JAKAPickAndPlace();
    //　从档案盒中取
    bool JAKAPickCab(int cab_id, int position, bool& mechanical_error);
    //　放到档案盒中
    bool JAKAPlaceCab(int cab_id, int position, bool& mechanical_error);
    //　从窗口中取
    bool JAKAPickWindow(int position, bool& mechanical_error);
    //　向窗口中放
    bool JAKAPlaceWindow(int position,  bool& mechanical_error);
    //　从暂存架中取
    bool JAKAPickStorage(int position, bool& mechanical_error);
    //　向暂存架中放
    bool JAKAPlaceStorage(int position,  bool& mechanical_error);
    // 机器人收起
    void JAKAContraction();
    // 机器人伸展开
    void JAKAStretch();
    // PLC动作
    bool JAKAPLCAction(int command);
    // 查询PLC状态
    void JAKAPLCState();
};

#endif