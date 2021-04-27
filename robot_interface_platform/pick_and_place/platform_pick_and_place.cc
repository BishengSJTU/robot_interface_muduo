#include "platform_pick_and_place.h"


PickAndPlacePlatform::PickAndPlacePlatform(const std::string &config_file_path, bool isInline):
fixed_config_(config_file_path + "/FixedConfig.YML"),
flexible_config_(config_file_path + "/FlexibleConfig.YML"),
vision_detection_(fixed_config_),
plc_(fixed_config_.get<std::string>("PLC_IP"), fixed_config_.get<int>("PLC_PORT"), isInline)
{

}

PickAndPlacePlatform::~PickAndPlacePlatform()
{
}

void PickAndPlacePlatform::Reset(){
    plc_.Reset();
}

bool PickAndPlacePlatform::ManipulateArchive(int cab_position, int arm_position, int mode) {
    double archive_width = 44.6;
    int benchmark_position;
    if(cab_position >= 1 && cab_position <= 6)
        benchmark_position = 3;
    else if(cab_position >= 7 && cab_position <= 12)
        benchmark_position = 10;
    int times = 0;
    std::vector<double> target_pose;
    if(!vision_detection_.GetPose(0, target_pose, times)) {
        if(!vision_detection_.GetPose(0, target_pose, times)) {
            if(!vision_detection_.GetPose(0, target_pose, times)) {
                std::cout << "Mark is not found" << std::endl;
                return false;
            }
        }
    }
    double x_apriltag = target_pose[0];
    double y_apriltag = target_pose[1];
    double theta_apriltag = target_pose[2] / 180 * M_PI;

    std::cout << "[x, y, theta]: [" << x_apriltag << "," << y_apriltag << "," << theta_apriltag << "]" << std::endl;

    double x_move = x_apriltag + (benchmark_position - cab_position) * archive_width * cos(theta_apriltag) + 2;
    double y_move = y_apriltag + (benchmark_position - cab_position) * archive_width * sin(theta_apriltag) + 16;

    std::cout << "[x_move, y_move]:[" << x_move << "," << y_move << "]" << std::endl;
    plc_.Move(x_move, y_move);


    if(mode == 1) {
        plc_.PickArchive(arm_position);
    } else if(mode == 2) {
        plc_.PlaceArchive(arm_position);
    }

    return true;
}

bool PickAndPlacePlatform::PickArchive(int cab_position, int arm_position)
{
    const int pick = 1;
    bool result = ManipulateArchive(cab_position, arm_position, pick);
    return result;
}

bool PickAndPlacePlatform::PlaceArchive(int cab_position, int arm_position)
{
    const int place = 2;
    bool result = ManipulateArchive(cab_position, arm_position, place);
    return result;
}