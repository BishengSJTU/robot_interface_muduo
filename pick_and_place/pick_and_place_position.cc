#include "pick_and_place_position.h"


PickAndPlace::PickAndPlace(const std::string &config_file_name, const std::string &motion_list_name):
config_(config_file_name),
vision_detection_(config_)
{
    adjust_move_speed = config_.get<float>("adjust_move_speed");
    fixed_joint_speed = config_.get<float>("fixed_joint_speed");
    pick_place_offset_x = config_.get<float>("pick_place_offset_x");
    pick_place_offset_y = config_.get<float>("pick_place_offset_y");
    pick_place_offset_z = config_.get<float>("pick_place_offset_z");
    pick_place_roll = config_.get<float>("pick_place_roll");
    pick_place_pitch = config_.get<float>("pick_place_pitch");
    pick_place_yaw = config_.get<float>("pick_place_yaw");
    max_count_adjust = config_.get<int>("max_count_adjust");
    eye_hand = config_.get<std::vector<double> >("eye_hand");

    eye_hand_matrix.resize(4, 4);
    for(int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            eye_hand_matrix(i, j) = eye_hand[i * 4 + j];
        }
    }
	motion_list_.UpdateMotionLists(motion_list_name);
}

void PickAndPlace::InitializePickAndPlace(){
    robot_client_.InitializeRobot(config_.get<std::string>("JAKA_ROBOT_IP"), config_.get<int>("JAKA_ROBOT_PORT"));
    plc_.InitializePLC(config_.get<std::string>("PLC_IP"), config_.get<int>("PLC_PORT"));
    PLCAction(MoveB);
    PLCAction(AirpumpC);
}

PickAndPlace::~PickAndPlace()
{

}

bool PickAndPlace::PickCab(int cab_id, int position, bool& mechanical_error) {

    mechanical_error = false;
    bool pick_result = true;
    // 移动至中间过渡点
    int all_position_num = config_.get<int>("cab" + std::to_string(cab_id) + "_all_position_num");

    std::vector<float> transition_point = config_.get<std::vector<float> >("transition_point");
    robot_client_.MoveJ(transition_point, fixed_joint_speed);
    std::cout << "移动至transition point" << std::endl;


    auto plc_fun = [&]() -> void{
        if(!this->plc_.ControlPLC(MoveF)) {
            if(!this->plc_.ControlPLC(MoveF)) {
                if(!this->plc_.ControlPLC(MoveF)) {
                    mechanical_error = true;
                }
            }
        }
    };
    std::thread t1(plc_fun);

    // 根据位置号判断机械臂的观察点Aruco码号和基准位置
    int root;
    int benchmark_position;
    if(all_position_num == 26) {
        if (position >= 1 && position <= 7) {
            root = 1;
            benchmark_position = 4;
        } else if (position >= 8 && position <= 14) {
            root = 2;
            benchmark_position = 11;
        } else if (position >= 15 && position <= 21) {
            root = 3;
            benchmark_position = 18;
        } else if (position >= 22 && position <= 26) {
            root = 4;
            benchmark_position = 25;
        }
    }
    else if(all_position_num == 32) {
        if (position >= 1 && position <= 5) {
            root = 1;
            benchmark_position = 3;
        } else if (position >= 6 && position <= 10) {
            root = 2;
            benchmark_position = 8;
        } else if (position >= 11 && position <= 16) {
            root = 3;
            benchmark_position = 13;
        } else if (position >= 17 && position <= 22) {
            root = 4;
            benchmark_position = 19;
        } else if (position >= 23 && position <= 27) {
            root = 5;
            benchmark_position = 25;
        } else if (position >= 28 && position <= 32) {
            root = 6;
            benchmark_position = 30;
        }
    }
    else if(all_position_num == 35) {
        if (position >= 1 && position <= 6) {
            root = 1;
            benchmark_position = 3;
        } else if (position >= 7 && position <= 12) {
            root = 2;
            benchmark_position = 9;
        } else if (position >= 13 && position <= 18) {
            root = 3;
            benchmark_position = 15;
        } else if (position >= 19 && position <= 24) {
            root = 4;
            benchmark_position = 21;
        } else if(position >= 25 && position <= 30) {
            root = 5;
            benchmark_position = 27;
        } else if(position >= 31 && position <= 35) {
            root = 6;
            benchmark_position = 33;
        }
    }
    else if(all_position_num == 33) {
        if (position >= 1 && position <= 5) {
            root = 1;
            benchmark_position = 3;
        } else if (position >= 6 && position <= 11) {
            root = 2;
            benchmark_position = 8;
        } else if (position >= 12 && position <= 17) {
            root = 3;
            benchmark_position = 14;
        } else if (position >= 18 && position <= 23) {
            root = 4;
            benchmark_position = 20;
        } else if(position >= 24 && position <= 28) {
            root = 5;
            benchmark_position = 26;
        } else if(position >= 29 && position <= 33) {
            root = 6;
            benchmark_position = 31;
        }
    }
    std::vector<float> watch_points;
    watch_points = config_.get<std::vector<float> >("watch_cab" + std::to_string(cab_id) + "_" + std::to_string(root));
    std::cout << "moveJ begin!!!!" << std::endl;
    robot_client_.MoveJ(watch_points, fixed_joint_speed);
    std::cout << "moveJ end!!!!" << std::endl;

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

        if(!vision_detection_.GetPose(0, pose, times)) {
            robot_client_.Jog(0, 80, 40);
            if(!vision_detection_.GetPose(0, pose, times)) {
                robot_client_.Jog(0, 80, -80);
                if(!vision_detection_.GetPose(0, pose, times)) {
                    LOG_ERROR << "无法估计aruco码的位姿";
                    pick_result = false;
                    break;
                }
            }
        }

        GetTargetCart(robot_client_, eye_hand_matrix, pose, target_pose_transform, cart);
        robot_client_.MoveE(cart, adjust_move_speed);

        std::cout << "---------------视觉调整完成一次------------------------" << std::endl;

        times++;
    }
    if(pick_result) {
        //针对手眼之间偏置调整
        robot_client_.Jog(5, fixed_joint_speed, 0.8);

        //沿末端坐标系x轴方向偏移量44.2mm为固定值:档案盒间距
        float offset = 44.2 * (position - benchmark_position) - 9.5;
        robot_client_.Jog(0, 80, offset);
    }

    t1.join();

    if(mechanical_error)
        return false;

    if(pick_result) {
        ExecuteMotionList(robot_client_, plc_, motion_list_.pick_cab_motion_list_1);
    }
    if(!plc_.ControlPLC(2)) {
        if(!plc_.ControlPLC(2)) {
            if(!plc_.ControlPLC(2)) {
                mechanical_error = true;
                LOG_ERROR << "Pickcab发生不可修复的机械故障，原地保护性停止！";
                return false;
            }
        }
    }
    // 档案不在手中
    if(!plc_.ArchiveInHand())
        pick_result = false;

    plc_.ControlPLC(8);
    // 沿末端坐标系Y轴方向运动-60mm
    robot_client_.Jog(1, 80, 60);

    robot_client_.MoveJ(watch_points, fixed_joint_speed);
    std::cout << "移动至watch point" << std::endl;

    robot_client_.MoveJ(transition_point, fixed_joint_speed);
    std::cout << "移动至transition point" << std::endl;


    std::cout << "########################################################" << std::endl;
    std::cout << "                 从档案柜中取档结束!                   " << std::endl;
    std::cout << "########################################################" << std::endl;
    return pick_result;
}

bool PickAndPlace::PlaceCab(int cab_id, int position,  bool& mechanical_error) {

    mechanical_error = false;
    bool place_result = true;
    // 移动至中间过渡点
    int all_position_num = config_.get<int>("cab" + std::to_string(cab_id) + "_all_position_num");

    std::vector<float> transition_point = config_.get<std::vector<float> >(
            "transition_point");
    robot_client_.MoveJ(transition_point, fixed_joint_speed);
    std::cout << "移动至transition point" << std::endl;

    // 根据位置号判断机械臂的观察点Aruco码号和基准位置
    int root;
    int benchmark_position;
    if(all_position_num == 26) {
        if (position >= 1 && position <= 7) {
            root = 1;
            benchmark_position = 4;
        } else if (position >= 8 && position <= 14) {
            root = 2;
            benchmark_position = 11;
        } else if (position >= 15 && position <= 21) {
            root = 3;
            benchmark_position = 18;
        } else if (position >= 22 && position <= 26) {
            root = 4;
            benchmark_position = 25;
        }
    }
    else if(all_position_num == 32) {
        if (position >= 1 && position <= 5) {
            root = 1;
            benchmark_position = 3;
        } else if (position >= 6 && position <= 10) {
            root = 2;
            benchmark_position = 8;
        } else if (position >= 11 && position <= 16) {
            root = 3;
            benchmark_position = 13;
        } else if (position >= 17 && position <= 22) {
            root = 4;
            benchmark_position = 19;
        } else if (position >= 23 && position <= 27) {
            root = 5;
            benchmark_position = 25;
        } else if (position >= 28 && position <= 32) {
            root = 6;
            benchmark_position = 30;
        }
    }
    else if(all_position_num == 35) {
        if (position >= 1 && position <= 6) {
            root = 1;
            benchmark_position = 3;
        } else if (position >= 7 && position <= 12) {
            root = 2;
            benchmark_position = 9;
        } else if (position >= 13 && position <= 18) {
            root = 3;
            benchmark_position = 15;
        } else if (position >= 19 && position <= 24) {
            root = 4;
            benchmark_position = 21;
        } else if(position >= 25 && position <= 30) {
            root = 5;
            benchmark_position = 27;
        } else if(position >= 31 && position <= 35) {
            root = 6;
            benchmark_position = 33;
        }
    }
    else if(all_position_num == 33) {
        if (position >= 1 && position <= 5) {
            root = 1;
            benchmark_position = 3;
        } else if (position >= 6 && position <= 11) {
            root = 2;
            benchmark_position = 8;
        } else if (position >= 12 && position <= 17) {
            root = 3;
            benchmark_position = 14;
        } else if (position >= 18 && position <= 23) {
            root = 4;
            benchmark_position = 20;
        } else if(position >= 24 && position <= 28) {
            root = 5;
            benchmark_position = 26;
        } else if(position >= 29 && position <= 33) {
            root = 6;
            benchmark_position = 31;
        }
    }

    std::vector<float> watch_points;
    watch_points = config_.get<std::vector<float> >("watch_cab" + std::to_string(cab_id) + "_" + std::to_string(root));
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

        if(!vision_detection_.GetPose(0, pose, times)) {
            robot_client_.Jog(0, 80, 40);
            if(!vision_detection_.GetPose(0, pose, times)) {
                robot_client_.Jog(0, 80, -80);
                if(!vision_detection_.GetPose(0, pose, times)) {
                    LOG_ERROR << "无法估计aruco码的位姿";
                    place_result = false;
                    break;
                }
            }
        }

        GetTargetCart(robot_client_, eye_hand_matrix, pose, target_pose_transform, cart);
        robot_client_.MoveE(cart, adjust_move_speed);

        std::cout << "---------------视觉调整完成一次-----------------------" << std::endl;
        times++;
    }

    if (place_result) {
        //针对手眼之间偏置调整
        robot_client_.Jog(5, fixed_joint_speed, 0.8);

        //沿末端坐标系x轴方向偏移量44.2mm为固定值:档案盒间距
        float offset = 44.2 * (position - benchmark_position) - 7;
        robot_client_.Jog(0, 80, offset);

//        robot_client_.Jog(3, fixed_joint_speed, 1.8);

        ExecuteMotionList(robot_client_, plc_, motion_list_.place_cab_motion_list_);

        if(plc_.ControlPLC(1) == false) {
            //沿末端坐标系Z轴方向运动-5mm
            robot_client_.Jog(1, 80, 15);
            // 不可修复的故障
            if(plc_.ControlPLC(1) == false) {
                mechanical_error = true;
                LOG_ERROR << "PlaceCab不可修复的机械故障，原地保护性停止！";
                return false;
            } else {
                robot_client_.Jog(1, 80, -15);
            }
        }

    }

    robot_client_.Jog(2, 80, 5);
//    robot_client_.Jog(3, 80, -2);
    robot_client_.Jog(1, 80, 60);


//    auto plc_fun = [&]() -> void {
//        if(!this->plc_.ControlPLC(2)) {
//            if(!this->plc_.ControlPLC(2)) {
//                if(!this->plc_.ControlPLC(2)) {
//                    mechanical_error = true;
//                }
//            }
//        }
//    };
//    std::thread t1(plc_fun);

    robot_client_.MoveJ(watch_points, fixed_joint_speed);
    std::cout << "移动至watch point" << std::endl;


    robot_client_.MoveJ(transition_point, fixed_joint_speed);
    std::cout << "移动至transition point" << std::endl;

//    t1.join();
    if(mechanical_error)
        return false;


    std::cout << "########################################################" << std::endl;
    std::cout << "            将档案盒放入档案柜中结束!         " << std::endl;
    std::cout << "########################################################" << std::endl;
    return place_result;
}

bool PickAndPlace::PickWindow(int position, bool& mechanical_error) {

    mechanical_error = false;
    bool pick_result = true;
    auto plc_fun = [&]() -> void{
        if(!this->plc_.ControlPLC(1)) {
            if(!this->plc_.ControlPLC(1)) {
                if(!this->plc_.ControlPLC(1)) {
                    mechanical_error = true;
                }
            }
        }
    };
    std::thread t1(plc_fun);

    // 基准位置
    int benchmark_position = 3;

    std::vector<float> transition_point = config_.get<std::vector<float> >(
            "transition_point");
    robot_client_.MoveJ(transition_point, fixed_joint_speed);
    std::cout << "移动至transition point" << std::endl;

    std::vector<float> watch_points;
    watch_points = config_.get<std::vector<float> >("watch_cab0_1");
    robot_client_.MoveJ(watch_points, fixed_joint_speed);
    std::cout << "移动至watch point" << std::endl;

   /* //沿末端坐标系Z轴方向偏移量50mm
    robot_client_.Jog(2, 80, 50);*/

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

        if(!vision_detection_.GetPose(0, pose, times)) {
            robot_client_.Jog(0, 80, 40);
            if(!vision_detection_.GetPose(0, pose, times)) {
                robot_client_.Jog(0, 80, -80);
                if(!vision_detection_.GetPose(0, pose, times)) {
                    LOG_ERROR << "无法估计aruco码的位姿";
                    pick_result = false;
                    break;
                }
            }
        }

        GetTargetCart(robot_client_, eye_hand_matrix, pose, target_pose_transform, cart);
        robot_client_.MoveE(cart, adjust_move_speed);

        std::cout << "---------------视觉调整结束一次------------------------" << std::endl;

        times++;
    }

    if(pick_result) {
        //针对手眼之间偏置调整
        robot_client_.Jog(5, fixed_joint_speed, 0.8);

        //沿末端坐标系x轴方向偏移量，44.2mm为固定值:档案盒间距
        float offset = 42.8 * (position - benchmark_position) - 7;
        robot_client_.Jog(0, 80, offset);
    }


    t1.join();
    if(mechanical_error)
        return false;

    if(pick_result) {
        ExecuteMotionList(robot_client_, plc_, motion_list_.pick_window_motion_list_);
    }

    if(!plc_.ControlPLC(2)) {
        if(!plc_.ControlPLC(2)) {
            if(!plc_.ControlPLC(2)) {
                mechanical_error = true;
                LOG_ERROR << "PickWindow发生不可修复的机械故障，原地保护性停止！";
                return false;
            }
        }
    }

    // 档案不在手中
    if(!plc_.ArchiveInHand())
        pick_result = false;

    plc_.ControlPLC(8);


       // 沿末端坐标系Y轴方向运动40mm
    robot_client_.Jog(1, 80, 40);

    robot_client_.MoveJ(watch_points, fixed_joint_speed);
    std::cout << "移动至watch point" << std::endl;

    robot_client_.MoveJ(transition_point, fixed_joint_speed);
    std::cout << "移动至transition point" << std::endl;


    std::cout << "########################################################" << std::endl;
    std::cout << "                 从窗口中取档结束!                   " << std::endl;
    std::cout << "########################################################" << std::endl;
    return pick_result;
}

bool PickAndPlace::PlaceWindow(int position, bool &mechanical_error) {

    mechanical_error = false;
    bool place_result = true;
    // 基准位置
    int benchmark_position = 3;

    std::vector<float> transition_point = config_.get<std::vector<float> >(
            "transition_point");
    robot_client_.MoveJ(transition_point, fixed_joint_speed);
    std::cout << "移动至transition point" << std::endl;

    std::vector<float> watch_points;
    watch_points = config_.get<std::vector<float> >("watch_cab0_1");
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

        if(!vision_detection_.GetPose(0, pose, times)) {
            robot_client_.Jog(0, 80, 40);
            if(!vision_detection_.GetPose(0, pose, times)) {
                robot_client_.Jog(0, 80, -80);
                if(!vision_detection_.GetPose(0, pose, times)) {
                    LOG_ERROR << "无法估计aruco码的位姿";
                    place_result = false;
                    break;
                }
            }
        }

        GetTargetCart(robot_client_, eye_hand_matrix, pose, target_pose_transform, cart);
        robot_client_.MoveE(cart, adjust_move_speed);

        std::cout << "---------------视觉调整结束一次------------------------" << std::endl;

        times++;
    }

    if(place_result) {
        //针对手眼之间偏置调整
        robot_client_.Jog(5, fixed_joint_speed, 0.8);

        //沿末端坐标系x轴方向偏移量，44.2mm为固定值:档案盒间距
        float offset = 42.8 * (position - benchmark_position) - 7;
        robot_client_.Jog(0, 80, offset);

        robot_client_.Jog(3, fixed_joint_speed, 1.8);

        ExecuteMotionList(robot_client_, plc_, motion_list_.place_window_motion_list_);

        if(plc_.ControlPLC(1) == false) {
            //沿末端坐标系Z轴方向运动-5mm
            robot_client_.Jog(1, 80, 15);
            // 不可修复的故障
            if(!plc_.ControlPLC(1)) {
                if(!plc_.ControlPLC(1)) {
                    mechanical_error = true;
                    LOG_ERROR << "PlaceWindow不可修复的机械故障，原地保护性停止！";
                    return false;

                }
            } else {
                robot_client_.Jog(1, 80, -15);
            }
        }
    }
    robot_client_.Jog(2, 80, 5);
//    robot_client_.Jog(3, 80, -2);
    robot_client_.Jog(1, 80, 40);

//    auto plc_fun = [&]() -> void{
//        if(!this->plc_.ControlPLC(2)) {
//            if(!this->plc_.ControlPLC(2)) {
//                if(!this->plc_.ControlPLC(2)) {
//                    mechanical_error = true;
//                }
//            }
//        }
//    };
//    std::thread t1(plc_fun);

    robot_client_.MoveJ(watch_points, fixed_joint_speed);
    std::cout << "移动至watch point" << std::endl;

    robot_client_.MoveJ(transition_point, fixed_joint_speed);
    std::cout << "移动至transition point" << std::endl;

//    t1.join();
    if(mechanical_error)
        return false;

    std::cout << "########################################################" << std::endl;
    std::cout << "                 向窗口中放入档案结束!                   " << std::endl;
    std::cout << "########################################################" << std::endl;
    return true;
}

bool PickAndPlace::PickStorage(int position, bool &mechanical_error) {

    mechanical_error = false;
    bool pick_result = true;
    auto plc_fun = [&]() -> void{
        if(!this->plc_.ControlPLC(1)) {
            if(!this->plc_.ControlPLC(1)) {
                if(!this->plc_.ControlPLC(1)) {
                    mechanical_error = true;
                }
            }
        }
    };
    std::thread t1(plc_fun);

    std::vector<float> storage_transition_point = config_.get<std::vector<float> >(
            "storage_transition_point");
    robot_client_.MoveJ(storage_transition_point, fixed_joint_speed);
    std::cout << "移动至storage_transition_point" << std::endl;

    std::vector<float> storage_point = config_.get<std::vector<float> >(
            "storage_point_" + std::to_string(position));
    robot_client_.MoveJ(storage_point, fixed_joint_speed);
    std::cout << "移动至storage_point" << std::endl;

    t1.join();
    if(mechanical_error)
        return false;

    ExecuteMotionList(robot_client_, plc_, motion_list_.pick_storage_motion_list_);

    if(!plc_.ControlPLC(2)) {
        if(!plc_.ControlPLC(2)) {
            if(!plc_.ControlPLC(2)) {
                mechanical_error = true;
                LOG_ERROR << "Pickcab发生不可修复的机械故障，原地保护性停止！";
                return false;
            }
        }
    }
    // 档案不在手中
    if(!plc_.ArchiveInHand())
        pick_result = false;

    plc_.ControlPLC(8);

    robot_client_.MoveJ(storage_transition_point, fixed_joint_speed);
    std::cout << "移动至storage_transition_point" << std::endl;

    return pick_result;
}

bool PickAndPlace::PlaceStorage(int position, bool &mechanical_error) {

    mechanical_error = false;

    std::vector<float> storage_transition_point = config_.get<std::vector<float> >(
            "storage_transition_point");
    robot_client_.MoveJ(storage_transition_point, fixed_joint_speed);
    std::cout << "移动至storage_transition_point" << std::endl;

    std::vector<float> storage_point = config_.get<std::vector<float> >(
            "storage_point_" + std::to_string(position));
    robot_client_.MoveJ(storage_point, fixed_joint_speed);
    std::cout << "移动至storage_point" << std::endl;


    ExecuteMotionList(robot_client_, plc_, motion_list_.place_storage_motion_list_);

    if(plc_.ControlPLC(1) == false) {
        //沿末端坐标系Y轴方向运动15mm
        robot_client_.Jog(1, 80, 15);
        // 不可修复的故障
        if(!plc_.ControlPLC(1)) {
            if(!plc_.ControlPLC(1)) {
                mechanical_error = true;
                LOG_ERROR << "PlaceWindow不可修复的机械故障，原地保护性停止！";
                return false;

            }
        } else {
            robot_client_.Jog(1, 80, -15);
        }
    }

    robot_client_.Jog(2, 80, 20);
    robot_client_.Jog(1, 80, 50);

    robot_client_.MoveJ(storage_transition_point, fixed_joint_speed);
    std::cout << "移动至storage_transition_point" << std::endl;

    if(mechanical_error)
        return false;

    return true;
}

void PickAndPlace::Contraction() {
    std::vector<float> contraction_point_1 = config_.get<std::vector<float> >(
            "contraction_point_1");
    robot_client_.MoveJ(contraction_point_1, fixed_joint_speed);
    std::cout << "移动至contraction_point_1" << std::endl;

    std::vector<float> contraction_point_2 = config_.get<std::vector<float> >(
            "contraction_point_2");
    robot_client_.MoveJ(contraction_point_2, fixed_joint_speed);
    std::cout << "移动至contraction_point_2" << std::endl;
}

void PickAndPlace::Stretch() {
    std::vector<float> contraction_point_2 = config_.get<std::vector<float> >(
            "contraction_point_2");
    robot_client_.MoveJ(contraction_point_2, fixed_joint_speed);
    std::cout << "移动至contraction_point_2" << std::endl;

    std::vector<float> contraction_point_1 = config_.get<std::vector<float> >(
            "contraction_point_1");
    robot_client_.MoveJ(contraction_point_1, fixed_joint_speed);
    std::cout << "移动至contraction_point_1" << std::endl;
}


bool PickAndPlace::PLCAction(int command)
{
    plc_.ControlPLC(command);
    std::cout << "#########################################################" << std::endl;
    std::cout << "                 PLC动作结束!                   " << std::endl;
    std::cout << "#########################################################" << std::endl;
    return true;
}

void PickAndPlace::PLCState()
{
    plc_.InquireState();
}
