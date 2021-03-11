#ifndef ROBOT_H
#define ROBOT_H

#include "TcpClient.h"
#include "EventLoop.h"
#include "ThreadPool.h"
#include "Thread.h"

using namespace muduo;
using namespace muduo::net;

class Robot
{
public:
    Robot(){;}

    void sendAndReceive() //收发线程
    {
    }

    void doTask() //执行任务线程
    {
    }

private:
};

#endif