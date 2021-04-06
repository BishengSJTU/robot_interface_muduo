#ifndef __JAKA_ROBOT_CLIENT_TCP_H_
#define __JAKA_ROBOT_CLIENT_TCP_H_

#define MAXLINE 8192

#include <string>
#include <vector>
#include <memory>
#include <unistd.h>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <json.hpp>
#include <exception>
#include <sstream>
#include "Logging.h"
#include "LogFile.h"

#define JAKAEXITNO 3
#define JAKATCPWAITTIME 1e5
#define JAKAX 0
#define JAKAY 1
#define JAKAZ 2
#define JAKARX 3
#define JAKARY 4
#define JAKARZ 5

class RobotClient {
public:
    RobotClient(std::string address, int port = 10001);

    ~RobotClient();

    //获取机器人关节姿态和末端姿态
    void GetRobotPose(std::vector<float> &jnt, std::vector<float> &cart);

    //关节形式运动
    void MoveJ(const std::vector<float> &joint_vector, const float &velocity);

    //末端形式运动[xyzrpy]
    void MoveE(const std::vector<float> &cart_vector, const float &velocity);

    //末端步进运动
    void Jog(const int &axis, const float &speed, const float &coord);

    //步进运动停止
    void JogStop(const int &jogcoord, const int &axis);

    //关闭机器人和控制器
    void Shutdown();

private:
    int socket_fd;
    struct sockaddr_in addr_;
    char buf[MAXLINE];
    const char *cmd_ptr;
    std::string string_tmp;
    std::string address_;
    int port_;

    void InitializeRobot(std::string address, int port = 10001);
};

#endif
