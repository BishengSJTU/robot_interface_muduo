#include "jaka_pick_and_place.h"


JAKAPickAndPlace::JAKAPickAndPlace(const std::string &config_file_path, bool isInline):
fixed_config_(config_file_path + "/FixedConfig.YML"),
flexible_config_(config_file_path + "/FlexibleConfig.YML"),
vision_detection_(fixed_config_),
robot_client_(fixed_config_.get<std::string>("JAKA_ROBOT_IP"), fixed_config_.get<int>("JAKA_ROBOT_PORT"), isInline),
plc_(fixed_config_.get<std::string>("PLC_IP"), fixed_config_.get<int>("PLC_PORT"), isInline)
{
    if(!isInline)
        return;
    adjust_joint_speed = fixed_config_.get<float>("adjust_joint_speed");
    fixed_joint_speed = fixed_config_.get<float>("fixed_joint_speed");
    fixed_linear_speed = fixed_config_.get<float>("fixed_linear_speed");
    pick_place_offset_x = fixed_config_.get<float>("pick_place_offset_x");
    pick_place_offset_y = fixed_config_.get<float>("pick_place_offset_y");
    pick_place_offset_z = fixed_config_.get<float>("pick_place_offset_z");
    pick_place_roll = fixed_config_.get<float>("pick_place_roll");
    pick_place_pitch = fixed_config_.get<float>("pick_place_pitch");
    pick_place_yaw = fixed_config_.get<float>("pick_place_yaw");
    max_count_adjust = fixed_config_.get<int>("max_count_adjust");
    eye_hand = fixed_config_.get<std::vector<double> >("eye_hand");

    eye_hand_matrix.resize(4, 4);
    for(int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            eye_hand_matrix(i, j) = eye_hand[i * 4 + j];
        }
    }
    JAKAPLCAction(MoveB);
    JAKAPLCAction(AirpumpC);
}

JAKAPickAndPlace::~JAKAPickAndPlace()
{

}

bool JAKAPickAndPlace::JAKAPickCab(int cab_id, int position, bool& mechanical_error) {

    mechanical_error = false;
    bool pick_result = true;
    // 移动至中间过渡点
    int all_position_num = flexible_config_.get<int>("cab" + std::to_string(cab_id) + "_all_position_num");

    std::vector<float> transition_point = flexible_config_.get<std::vector<float> >("transition_point");
    robot_client_.MoveJ(transition_point, fixed_joint_speed);
    std::cout << "移动至transition point" << std::endl;


    auto plc_fun = [&]() -> void{
        if(!this->plc_.ControlPLC(MoveF)) {
            if(!this->plc_.ControlPLC(MoveF)) {
                if(!this->plc_.ControlPLC(MoveF)) {
                    LOG_ERROR << "PickCab不可修复的机械故障，原地保护性停止";
                    mechanical_error = true;
                }
            }
        }
    };
    std::thread t1(plc_fun);

    // 根据位置号判断机械臂的观察点Aruco码号和基准位置
    int root;
    int benchmark_position;
    MappingTable(all_position_num, position, root, benchmark_position);

    std::vector<float> watch_points;
    watch_points = flexible_config_.get<std::vector<float> >("watch_cab" + std::to_string(cab_id) + "_" + std::to_string(root));
    robot_client_.MoveJ(watch_points, fixed_joint_speed);
    std::cout << "移动至watch point" << std::endl;


    PoseTransform pose_transform;
    PoseTransform target_pose_transform(
            pick_place_offset_x,
            pick_place_offset_y,
            pick_place_offset_z,
            pick_place_roll,
            pick_place_pitch,
            pick_place_yaw);

    int times = 1;
    for(int adjust_time = 0; adjust_time < max_count_adjust; adjust_time++) {
        cv::Mat pose;
        std::vector<float> cart;

        std::cout << "---------------视觉调整： " << adjust_time + 1 << " -------------" << std::endl;

        if(!vision_detection_.GetPose(CODEING_ID, pose, times)) {
            robot_client_.Jog(JAKAX, fixed_linear_speed, 30);
            if(!vision_detection_.GetPose(CODEING_ID, pose, times)) {
                robot_client_.Jog(JAKAX, fixed_linear_speed, -60);
                if(!vision_detection_.GetPose(CODEING_ID, pose, times)) {
                    LOG_ERROR << "无法估计aruco码的位姿";
                    pick_result = false;
                    break;
                }
            }
        }

        GetTargetCart(robot_client_, eye_hand_matrix, pose, target_pose_transform, cart);
        robot_client_.MoveE(cart, adjust_joint_speed);

        std::cout << "---------------视觉调整完成一次------------------------" << std::endl;

        times++;
    }
    t1.join();
    if(mechanical_error)
        return false;

//////////////////////////////////////////根据不同现场需要更改/////////////////////////////////////////////
    if(pick_result) {
        //针对手眼之间偏置调整
        robot_client_.Jog(JAKARZ, fixed_joint_speed, 0.8);
        float offset = CAB_SPACE * (position - benchmark_position) - 9.5;
        robot_client_.Jog(JAKAX, fixed_linear_speed, offset);
        robot_client_.Jog(JAKAZ, fixed_linear_speed, 36);
        plc_.ControlPLC(AirpumpO);
        robot_client_.Jog(JAKAY, fixed_linear_speed, -93);
        robot_client_.Jog(JAKAZ, fixed_linear_speed, -16);
    }
/////////////////////////////////////////////////////////////////////////////////////////////////////

    if(!plc_.ControlPLC(MoveB)) {
        if(!plc_.ControlPLC(MoveB)) {
            if(!plc_.ControlPLC(MoveB)) {
                mechanical_error = true;
                LOG_ERROR << "Pickcab发生不可修复的机械故障，原地保护性停止！";
                return false;
            }
        }
    }
    // 档案不在手中
    if(!plc_.ArchiveInHand()) {
        pick_result = false;
        LOG_ERROR << "Pickcab档案盒未取出";
    }

    plc_.ControlPLC(AirpumpC);

    robot_client_.Jog(JAKAY, fixed_linear_speed, 60);
    robot_client_.MoveJ(watch_points, fixed_joint_speed);
    std::cout << "移动至watch point" << std::endl;
    robot_client_.MoveJ(transition_point, fixed_joint_speed);
    std::cout << "移动至transition point" << std::endl;

    LOG_INFO << "从档案柜中取档结束!";
    return pick_result;
}

bool JAKAPickAndPlace::JAKAPlaceCab(int cab_id, int position,  bool& mechanical_error) {

    mechanical_error = false;
    bool place_result = true;
    // 移动至中间过渡点
    int all_position_num = flexible_config_.get<int>("cab" + std::to_string(cab_id) + "_all_position_num");

    std::vector<float> transition_point = flexible_config_.get<std::vector<float> >(
            "transition_point");
    robot_client_.MoveJ(transition_point, fixed_joint_speed);
    std::cout << "移动至transition point" << std::endl;

    // 根据位置号判断机械臂的观察点位置和基准位置
    int root;
    int benchmark_position;
    MappingTable(all_position_num, position, root, benchmark_position);

    std::vector<float> watch_points;
    watch_points = flexible_config_.get<std::vector<float> >("watch_cab" + std::to_string(cab_id) + "_" + std::to_string(root));
    robot_client_.MoveJ(watch_points, fixed_joint_speed);
    std::cout << "移动至watch point" << std::endl;

    PoseTransform pose_transform;
    PoseTransform target_pose_transform(
            pick_place_offset_x,
            pick_place_offset_y,
            pick_place_offset_z,
            pick_place_roll,
            pick_place_pitch,
            pick_place_yaw);

    int times = 1;
    for (int adjust_time = 0; adjust_time < max_count_adjust; adjust_time++) {
        cv::Mat pose;
        std::vector<float> cart;

        std::cout << "---------------视觉调整次数:" << adjust_time + 1 << " --------------" << std::endl;

        if(!vision_detection_.GetPose(CODEING_ID, pose, times)) {
            robot_client_.Jog(JAKAX, fixed_linear_speed, 30);
            if(!vision_detection_.GetPose(CODEING_ID, pose, times)) {
                robot_client_.Jog(JAKAX, fixed_linear_speed, -60);
                if(!vision_detection_.GetPose(CODEING_ID, pose, times)) {
                    LOG_ERROR << "无法估计aruco码的位姿";
                    place_result = false;
                    break;
                }
            }
        }

        GetTargetCart(robot_client_, eye_hand_matrix, pose, target_pose_transform, cart);
        robot_client_.MoveE(cart, adjust_joint_speed);

        std::cout << "---------------视觉调整完成一次-----------------------" << std::endl;
        times++;
    }

//////////////////////////////////////////根据不同现场需要更改/////////////////////////////////////////////
    if (place_result) {
        //针对手眼之间偏置调整
        robot_client_.Jog(JAKARZ, fixed_joint_speed, 0.8);
        float offset = CAB_SPACE * (position - benchmark_position) - 7;
        robot_client_.Jog(JAKAX, fixed_linear_speed, offset);
        robot_client_.Jog(JAKAZ, fixed_linear_speed, 20);
        robot_client_.Jog(JAKAY, fixed_linear_speed, -92);
//////////////////////////////////////////////////////////////////////////////////////////////////////

        if(plc_.ControlPLC(MoveMF) == false) {
            mechanical_error = true;
            LOG_ERROR << "PlaceCab不可修复的机械故障，原地保护性停止！";
            return false;
        } else {
            robot_client_.Jog(JAKAY, fixed_linear_speed, 5);
            if(plc_.ControlPLC(MoveF) == false) {
                robot_client_.Jog(JAKAY, fixed_linear_speed, 10);
                // 不可修复的故障
                if (plc_.ControlPLC(MoveF) == false) {
                    mechanical_error = true;
                    LOG_ERROR << "PlaceCab不可修复的机械故障，原地保护性停止！";
                    return false;
                } else {
                    robot_client_.Jog(JAKAY, fixed_linear_speed, -10);
                }
            }
            robot_client_.Jog(JAKAY, fixed_linear_speed, -5);
        }

    }
    robot_client_.Jog(JAKAZ, fixed_linear_speed, 5);
    robot_client_.Jog(JAKAY, fixed_linear_speed, 60);


    robot_client_.MoveJ(watch_points, fixed_joint_speed);
    std::cout << "移动至watch point" << std::endl;
    robot_client_.MoveJ(transition_point, fixed_joint_speed);
    std::cout << "移动至transition point" << std::endl;
    if(mechanical_error)
        return false;

    LOG_INFO << "将档案盒放入档案柜中结束!";
    return place_result;
}

bool JAKAPickAndPlace::JAKAPickWindow(int position, bool& mechanical_error) {

    mechanical_error = false;
    bool pick_result = true;
    auto plc_fun = [&]() -> void{
        if(!this->plc_.ControlPLC(MoveF)) {
            if(!this->plc_.ControlPLC(MoveF)) {
                if(!this->plc_.ControlPLC(MoveF)) {
                    LOG_ERROR << "PickWindow发生不可修复的机械故障，原地保护性停止！";
                    mechanical_error = true;
                }
            }
        }
    };
    std::thread t1(plc_fun);

    // 基准位置
    int benchmark_position = 3;

    std::vector<float> transition_point = flexible_config_.get<std::vector<float> >(
            "transition_point");
    robot_client_.MoveJ(transition_point, fixed_joint_speed);
    std::cout << "移动至transition point" << std::endl;

    std::vector<float> watch_points;
    watch_points = flexible_config_.get<std::vector<float> >("watch_cab0_1");
    robot_client_.MoveJ(watch_points, fixed_joint_speed);
    std::cout << "移动至watch point" << std::endl;

    PoseTransform pose_transform;
    PoseTransform target_pose_transform(
            pick_place_offset_x,
            pick_place_offset_y,
            pick_place_offset_z,
            pick_place_roll,
            pick_place_pitch,
            pick_place_yaw);

    int times = 1;
    for(int adjust_time = 0; adjust_time < max_count_adjust; adjust_time++) {
        cv::Mat pose;
        std::vector<float> cart;

        std::cout << "---------------视觉调整次数: " << adjust_time + 1 << " -------------" << std::endl;

        if(!vision_detection_.GetPose(CODEING_ID, pose, times)) {
            robot_client_.Jog(JAKAX, fixed_linear_speed, 40);
            if(!vision_detection_.GetPose(CODEING_ID, pose, times)) {
                robot_client_.Jog(JAKAX, fixed_linear_speed, -80);
                if(!vision_detection_.GetPose(CODEING_ID, pose, times)) {
                    LOG_ERROR << "无法估计aruco码的位姿";
                    pick_result = false;
                    break;
                }
            }
        }

        GetTargetCart(robot_client_, eye_hand_matrix, pose, target_pose_transform, cart);
        robot_client_.MoveE(cart, adjust_joint_speed);

        std::cout << "---------------视觉调整结束一次------------------------" << std::endl;

        times++;
    }

    t1.join();
    if(mechanical_error)
        return false;

//////////////////////////////////////////根据不同现场需要更改/////////////////////////////////////////////
    if(pick_result) {
        //针对手眼之间偏置调整
        robot_client_.Jog(JAKARZ, fixed_joint_speed, 0.8);

        //44.2mm为固定值:档案盒间距
        float offset = WINDOW_SPACE * (position - benchmark_position) - 7;
        robot_client_.Jog(JAKAX, fixed_linear_speed, offset);
    }

    if(pick_result) {
        robot_client_.Jog(JAKAZ, fixed_linear_speed, 50);
        plc_.ControlPLC(AirpumpO);
        robot_client_.Jog(JAKAY, fixed_linear_speed, -88);
        robot_client_.Jog(JAKAZ, fixed_linear_speed, -20);
    }
//////////////////////////////////////////////////////////////////////////////////////////////////////

    if(!plc_.ControlPLC(MoveB)) {
        if(!plc_.ControlPLC(MoveB)) {
            if(!plc_.ControlPLC(MoveB)) {
                mechanical_error = true;
                LOG_ERROR << "PickWindow发生不可修复的机械故障，原地保护性停止！";
                return false;
            }
        }
    }

    // 档案不在手中
    if(!plc_.ArchiveInHand()) {
        LOG_ERROR << "PickWindow档案盒未取出";
        pick_result = false;
    }

    plc_.ControlPLC(AirpumpC);

    robot_client_.Jog(JAKAY, fixed_linear_speed, 40);
    robot_client_.MoveJ(watch_points, fixed_joint_speed);
    std::cout << "移动至watch point" << std::endl;
    robot_client_.MoveJ(transition_point, fixed_joint_speed);
    std::cout << "移动至transition point" << std::endl;


    LOG_INFO << "从窗口中取档结束!";
    return pick_result;
}

bool JAKAPickAndPlace::JAKAPlaceWindow(int position, bool &mechanical_error) {

    mechanical_error = false;
    bool place_result = true;
    // 基准位置
    int benchmark_position = 3;

    std::vector<float> transition_point = flexible_config_.get<std::vector<float> >(
            "transition_point");
    robot_client_.MoveJ(transition_point, fixed_joint_speed);
    std::cout << "移动至transition point" << std::endl;

    std::vector<float> watch_points;
    watch_points = flexible_config_.get<std::vector<float> >("watch_cab0_1");
    robot_client_.MoveJ(watch_points, fixed_joint_speed);
    std::cout << "移动至watch point" << std::endl;


    PoseTransform pose_transform;
    PoseTransform target_pose_transform(
            pick_place_offset_x,
            pick_place_offset_y,
            pick_place_offset_z,
            pick_place_roll,
            pick_place_pitch,
            pick_place_yaw);

    int times = 1;
    for(int adjust_time = 0; adjust_time < max_count_adjust; adjust_time++) {
        cv::Mat pose;
        std::vector<float> cart;

        std::cout << "---------------视觉调整次数: " << adjust_time + 1 << " -------------" << std::endl;

        if(!vision_detection_.GetPose(CODEING_ID, pose, times)) {
            robot_client_.Jog(JAKAX, fixed_linear_speed, 40);
            if(!vision_detection_.GetPose(CODEING_ID, pose, times)) {
                robot_client_.Jog(JAKAX, fixed_linear_speed, -80);
                if(!vision_detection_.GetPose(CODEING_ID, pose, times)) {
                    LOG_ERROR << "无法估计aruco码的位姿";
                    place_result = false;
                    break;
                }
            }
        }

        GetTargetCart(robot_client_, eye_hand_matrix, pose, target_pose_transform, cart);
        robot_client_.MoveE(cart, adjust_joint_speed);

        std::cout << "---------------视觉调整结束一次------------------------" << std::endl;

        times++;
    }

//////////////////////////////////////////根据不同现场需要更改/////////////////////////////////////////////
    if(place_result) {
        //针对手眼之间偏置调整
        robot_client_.Jog(JAKARZ, fixed_joint_speed, 0.8);
        //档案盒间距
        float offset = WINDOW_SPACE * (position - benchmark_position) - 7;
        robot_client_.Jog(JAKAX, fixed_linear_speed, offset);
        robot_client_.Jog(JAKARX, fixed_joint_speed, 1.8);
        robot_client_.Jog(JAKAZ, fixed_linear_speed, 30);
        robot_client_.Jog(JAKAY, fixed_linear_speed, -86);
/////////////////////////////////////////////////////////////////////////////////////////////////////

        if(plc_.ControlPLC(MoveMF) == false) {
            mechanical_error = true;
            LOG_ERROR << "PlaceCab不可修复的机械故障，原地保护性停止！";
            return false;
        } else {
            robot_client_.Jog(JAKAY, fixed_linear_speed, 5);
            if(plc_.ControlPLC(MoveF) == false) {
                robot_client_.Jog(JAKAY, fixed_linear_speed, 10);
                // 不可修复的故障
                if (plc_.ControlPLC(MoveF) == false) {
                    mechanical_error = true;
                    LOG_ERROR << "PlaceCab不可修复的机械故障，原地保护性停止！";
                    return false;
                } else {
                    robot_client_.Jog(JAKAY, fixed_linear_speed, -10);
                }
            }
            robot_client_.Jog(JAKAY, fixed_linear_speed, -5);
        }
    }
    robot_client_.Jog(JAKAZ, fixed_linear_speed, 5);
    robot_client_.Jog(JAKAY, fixed_linear_speed, 40);

    robot_client_.MoveJ(watch_points, fixed_joint_speed);
    std::cout << "移动至watch point" << std::endl;
    robot_client_.MoveJ(transition_point, fixed_joint_speed);
    std::cout << "移动至transition point" << std::endl;

    if(mechanical_error)
        return false;

    LOG_INFO << "向窗口中放入档案结束!";
    return true;
}

bool JAKAPickAndPlace::JAKAPickStorage(int position, bool &mechanical_error) {

    mechanical_error = false;
    bool pick_result = true;
    auto plc_fun = [&]() -> void{
        if(!this->plc_.ControlPLC(MoveF)) {
            if(!this->plc_.ControlPLC(MoveF)) {
                if(!this->plc_.ControlPLC(MoveF)) {
                    LOG_ERROR << "PickStorage发生不可修复的机械故障，原地保护性停止！";
                    mechanical_error = true;
                }
            }
        }
    };
    std::thread t1(plc_fun);

    std::vector<float> storage_transition_point = flexible_config_.get<std::vector<float> >(
            "storage_transition_point");
    robot_client_.MoveJ(storage_transition_point, fixed_joint_speed);
    std::cout << "移动至storage_transition_point" << std::endl;

    std::vector<float> storage_point = flexible_config_.get<std::vector<float> >(
            "storage_point_" + std::to_string(position));
    robot_client_.MoveJ(storage_point, fixed_joint_speed);
    std::cout << "移动至storage_point" << std::endl;

    t1.join();
    if(mechanical_error)
        return false;

//////////////////////////////////////////根据不同现场需要更改/////////////////////////////////////////////
    robot_client_.Jog(JAKAZ, fixed_linear_speed, 6);
    robot_client_.Jog(JAKAY, fixed_linear_speed, -21);
    robot_client_.Jog(JAKAX, fixed_linear_speed, -10);
    robot_client_.Jog(JAKARX, fixed_joint_speed, -1);
    plc_.ControlPLC(AirpumpO);
    robot_client_.Jog(JAKARZ, fixed_linear_speed, -30);
/////////////////////////////////////////////////////////////////////////////////////////////////////

    if(!plc_.ControlPLC(MoveB)) {
        if(!plc_.ControlPLC(MoveB)) {
            if(!plc_.ControlPLC(MoveB)) {
                mechanical_error = true;
                LOG_ERROR << "PickStorage发生不可修复的机械故障，原地保护性停止！";
                return false;
            }
        }
    }
    // 档案不在手中
    if(!plc_.ArchiveInHand()) {
        LOG_ERROR << "PickStorage档案盒未取出";
        pick_result = false;
    }

    plc_.ControlPLC(AirpumpC);

    robot_client_.MoveJ(storage_transition_point, fixed_joint_speed);
    std::cout << "移动至storage_transition_point" << std::endl;

    return pick_result;
}

bool JAKAPickAndPlace::JAKAPlaceStorage(int position, bool &mechanical_error) {

    mechanical_error = false;

    std::vector<float> storage_transition_point = flexible_config_.get<std::vector<float> >(
            "storage_transition_point");
    robot_client_.MoveJ(storage_transition_point, fixed_joint_speed);
    std::cout << "移动至storage_transition_point" << std::endl;

    std::vector<float> storage_point = flexible_config_.get<std::vector<float> >(
            "storage_point_" + std::to_string(position));
    robot_client_.MoveJ(storage_point, fixed_joint_speed);
    std::cout << "移动至storage_point" << std::endl;

//////////////////////////////////////////根据不同现场需要更改/////////////////////////////////////////////
    robot_client_.Jog(JAKAX, fixed_linear_speed, -10);
    robot_client_.Jog(JAKAY, fixed_linear_speed, -23);
    robot_client_.Jog(JAKAZ, fixed_linear_speed, -12);
/////////////////////////////////////////////////////////////////////////////////////////////////////

    if(plc_.ControlPLC(MoveMF) == false) {
        mechanical_error = true;
        LOG_ERROR << "PlaceCab不可修复的机械故障，原地保护性停止！";
        return false;
    } else {
        robot_client_.Jog(JAKAY, fixed_linear_speed, 5);
        if(plc_.ControlPLC(MoveF) == false) {
            robot_client_.Jog(JAKAY, fixed_linear_speed, 10);
            // 不可修复的故障
            if (plc_.ControlPLC(MoveF) == false) {
                mechanical_error = true;
                LOG_ERROR << "PlaceCab不可修复的机械故障，原地保护性停止！";
                return false;
            } else {
                robot_client_.Jog(JAKAY, fixed_linear_speed, -10);
            }
        }
        robot_client_.Jog(JAKAY, fixed_linear_speed, -5);
    }

    robot_client_.Jog(JAKAZ, fixed_linear_speed, 20);
    robot_client_.Jog(JAKAY, fixed_linear_speed, 50);

    robot_client_.MoveJ(storage_transition_point, fixed_joint_speed);
    std::cout << "移动至storage_transition_point" << std::endl;

    if(mechanical_error)
        return false;

    return true;
}

void JAKAPickAndPlace::JAKAContraction() {
    std::vector<float> contraction_point_1 = flexible_config_.get<std::vector<float> >(
            "contraction_point_1");
    robot_client_.MoveJ(contraction_point_1, fixed_joint_speed);
    std::cout << "移动至contraction_point_1" << std::endl;

    std::vector<float> contraction_point_2 = flexible_config_.get<std::vector<float> >(
            "contraction_point_2");
    robot_client_.MoveJ(contraction_point_2, fixed_joint_speed);
    std::cout << "移动至contraction_point_2" << std::endl;
}

void JAKAPickAndPlace::JAKAStretch() {
    std::vector<float> contraction_point_2 = flexible_config_.get<std::vector<float> >(
            "contraction_point_2");
    robot_client_.MoveJ(contraction_point_2, fixed_joint_speed);
    std::cout << "移动至contraction_point_2" << std::endl;

    std::vector<float> contraction_point_1 = flexible_config_.get<std::vector<float> >(
            "contraction_point_1");
    robot_client_.MoveJ(contraction_point_1, fixed_joint_speed);
    std::cout << "移动至contraction_point_1" << std::endl;
}


bool JAKAPickAndPlace::JAKAPLCAction(int command)
{
    plc_.ControlPLC(command);
    LOG_INFO << "PLC动作结束!";
    return true;
}

void JAKAPickAndPlace::JAKAPLCState()
{
    plc_.InquireState();
}
