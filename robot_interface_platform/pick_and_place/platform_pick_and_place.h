#ifndef PLATFORM_PICK_AND_PLACE_H
#define PLATFORM_PICK_AND_PLACE_H

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <unistd.h>

#include "config.h"
#include "platform_plc.h"
#include "vision_detection_position.h"

#include <thread>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/QR>
#include <sstream>

class PickAndPlacePlatform
{
private:

    // 配置文件
    const Config fixed_config_;
    const Config flexible_config_;

    // PLC动作
    PlatformPLC plc_;

    // Aruco码
    VisionDetection vision_detection_;

public:
    PickAndPlacePlatform(const std::string &config_file_name, bool isInline = true);
    ~PickAndPlacePlatform();
    void InitializePickAndPlace();
    void Reset();
    //　从档案盒中取
    bool ManipulateArchive(int cab_position, int arm_position, int mode);
    bool PickArchive(int cab_position, int arm_position);
    //　放到档案盒中
    bool PlaceArchive(int cab_position, int arm_position);
};

#endif