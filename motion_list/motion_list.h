#ifndef MONTION_LIST_H
#define MOTION_LIST_H
#include <iostream>
#include "robot_client_tcp.h"
#include "my_plc.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <unistd.h>


class MotionList {
public:
    std::vector<std::vector<int> > pick_cab_motion_list_1;
    std::vector<std::vector<int> > pick_cab_motion_list_2;
    std::vector<std::vector<int> > place_cab_motion_list_;
    std::vector<std::vector<int> > pick_window_motion_list_;
    std::vector<std::vector<int> > place_window_motion_list_;
    std::vector<std::vector<int> > pick_storage_motion_list_;
    std::vector<std::vector<int> > place_storage_motion_list_;
public:
    void UpdateMotionLists(const std::string &motion_list_file_name);
};
void UpdateMotionList(cv::FileNodeIterator it, cv::FileNodeIterator it_end, std::vector<std::vector<int> > &motion_list);
void ExecuteMotionList(RobotClient &robot_client, MyPLC &plc, const std::vector<std::vector<int> >& motion_list);
#endif