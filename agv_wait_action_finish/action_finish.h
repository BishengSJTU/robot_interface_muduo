#ifndef ACTION_FINISH_H
#define ACTION_FINISH_H

#define MODBUSEXITNO 4

#include<netinet/in.h>
#include<sys/types.h>
#include<sys/socket.h>
#include<arpa/inet.h>
#include<sys/ioctl.h>
#include<unistd.h>
#include<iostream>
#include<string>
#include<cstdlib>
#include<cstdio>
#include<cstring>
#include<chrono>
#include <unistd.h>
#include <thread>
#include <mutex>
#include "Logging.h"
#include "LogFile.h"

// 注意：使用该类时需要定义局部作用域，离开作用域时程序会自动析构，释放文件描述符，保证下次能再次连接
class ActionFinish
{
private:
    struct sockaddr_in server_addr_;
    int listen_fd_;
    int link_fd_;
    int queue_len_;

public:
    explicit ActionFinish(int port = 502, int queue_len = 20);
    ~ActionFinish();
    bool AgvIsReached();
    bool ArmFinishAction();
};

#endif