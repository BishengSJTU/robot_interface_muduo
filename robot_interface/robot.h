#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>
#include <set>
#include <boost/bind.hpp>
#include "TcpClient.h"
#include "EventLoop.h"
#include "ThreadPool.h"
#include "Thread.h"
#include "Buffer.h"
#include "Logging.h"
#include "Logging.h"
#include "codec.h"

using namespace muduo;
using namespace muduo::net;

class Robot : muduo::noncopyable
{
public:
    //机器人自身信息
    struct RobotInfo
    {
        u_char currentState; //当前状态
        u_char currentPower; //当前电量
    };
    enum CURRENT_STATE {
        IS_FREE = 0x00,
        IS_DEPOSITING = 0x01,
        IS_WITHDRAWING = 0x02,
        IS_CHARGING = 0x03,
        IS_CHECKING = 0x04,
        IS_WALKING = 0x05,
        IS_IN_WINDOW = 0x06,
        PREPARE_CHARGING = 0x07,
        CHARGE_FINISHED = 0x08,
        IS_IN_CAB = 0x09,
        IS_INITIALIZING = 0xF0,
        INITIALIZE_FINISH = 0xF1,
        NEED_INITIALIZING = 0xF2,
        DEPOSITE_WITHDRAW_ERROR = 0xE0,
        MECHANICAL_ERROR = 0xE1,
        EMERGENCY_STOP = 0xE2
    };
    //外部设备信息
    struct ExternalInfo
    {
        muduo::string actualRFID; //读卡器传来的RFID
        std::set<int64_t> readyCab; //准备就绪的档案柜
        bool singleArchiveFinshed; //单本动作完成被成功接收
    };
    Robot();
    void eventLoopThread(); //收发消息及定时器函数线程
    void execTaskThread(); //执行任务函数线程
    void write(const StringPiece& message); //往Buffer里写数据
    void onCompleteMessage(const muduo::net::TcpConnectionPtr&,
                         const muduo::string& message,
                         Timestamp); //收到完整指令回调，并执行对应任务
    void onConnection(const TcpConnectionPtr& conn); //连接回调

private:
    string taskServerIP_;
    uint16_t taskServerPort_;

    LengthHeaderCodec codec_;
    MutexLock connectionMutex_;
    TcpConnectionPtr connection_ GUARDED_BY(connectionMutex_);

    MutexLock taskMessageMutex_;
    StringPiece taskMessage_ GUARDED_BY(taskMessageMutex_);
    Condition taskCondition_ GUARDED_BY(taskMessageMutex_);

    MutexLock robotInfoMutex_;
    RobotInfo robotInfo_ GUARDED_BY(robotInfoMutex_);

    MutexLock externalInfoMutex_;
    ExternalInfo externalInfo_ GUARDED_BY(externalInfoMutex_);
    static const char connectMsg_[8];
};

#endif