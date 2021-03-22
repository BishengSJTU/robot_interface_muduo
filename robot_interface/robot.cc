#include "robot.h"

const char Robot::connectMsg_[8] = {static_cast<char>(0xA1), 0x00, 0x00, 0x00, 0x11, 0x00, 0x01 , static_cast<char>(0xAA)};

Robot::Robot():
codec_(std::bind(&Robot::onCompleteMessage, this, muduo::_1, muduo::_2, muduo::_3)),
taskCondition_(taskMessageMutex_),
externalCondition_(externalInfoMutex_)
{

}

void Robot::eventLoopThread() {
    muduo::net::EventLoop loop;
    muduo::net::InetAddress serverAddr("127.0.0.1", 9981);
    muduo::net::TcpClient client(&loop, serverAddr, "TcpClient");
    client.setConnectionCallback(std::bind(&Robot::onConnection, this, muduo::_1));
    client.setMessageCallback(std::bind(&LengthHeaderCodec::onMessage, &codec_, muduo::_1, muduo::_2, muduo::_3));
    client.enableRetry();
    client.connect();
    loop.loop();
}

void Robot::execTaskThread() {
    while(1) {
        StringPiece task;
        string response(task.data(), task.size());
        // 等待任务消息到来，否则此线程阻塞
        {
            MutexLockGuard lock(taskMessageMutex_);
            while (taskMessage_.size() == 0) {
                taskCondition_.wait();
            }
            task = taskMessage_;
            taskMessage_ = "";
        }
        const char *data = task.data();
        char taskType = data[4];
        u_char state;
        u_char power;
        {
            MutexLockGuard lock(robotInfoMutex_);
            state = robotInfo_.currentState;
            power = robotInfo_.currentPower;
        }
        // 判断此时机器人状态是否空闲或者在充电中，若不属于上述两种状态，直接忽略该任务
        if((state != IS_CHARGING) && (state != IS_FREE)) {
            if (taskType == DEPOSIT_TASK || taskType == WITHDRAW_TASK) {
                for(std::size_t i = 0; i < 12; i++) {
                    if(response[7 + 2 * i] = 0xA1)
                    response[7 + 2 * i] = 0x00;
                }
            } else if(taskType == CHARGE_TASK || taskType == ALL_FINISH) {
                response[7] = 0x00;
            }
            StringPiece responseMsg(response);
            write(responseMsg);
            continue;
        }
        // 如果正在充电或准备充电，忽略充电任务
        if(state == IS_CHARGING || state == CHARGE_PREPARE) {
            if(taskType == CHARGE_TASK) {
                response[7] = 0x00;
                StringPiece responseMsg(response);
                write(responseMsg);
                continue;
            }
        }
        // 电量小于最低电量，只接收充电任务
        if(power <= powerLowerLimit) {
            if(taskType == CHARGE_TASK) {
                {
                    MutexLockGuard lock(robotInfoMutex_);
                    robotInfo_.currentState = CHARGE_PREPARE;
                }
                ; //执行充电指令.wait to complete
                {
                    MutexLockGuard lock(robotInfoMutex_);
                    robotInfo_.currentState = IS_CHARGING;
                }
            } else if (taskType == DEPOSIT_TASK || taskType == WITHDRAW_TASK) {
                for(std::size_t i = 0; i < 12; i++) {
                    if(response[7 + 2 * i] = 0xA1)
                        response[7 + 2 * i] = 0x00;
                }
            } else if(taskType == ALL_FINISH) {
                response[7] = 0x00;
            }
            StringPiece responseMsg(response);
            write(responseMsg);
            continue;
        }
        // 可执行任务状态
        switch(taskType) {
            //存档
            case DEPOSIT_TASK: {
                {
                    ; // 跑到窗口.wait to complete
                }
                // 等待接收RFID消息
                muduo::string actualRFID;
                {
                    MutexLockGuard lock(externalInfoMutex_);
                    while (externalInfo_.actualRFID == "") {
                        externalCondition_.wait();
                    }
                    actualRFID = externalInfo_.actualRFID;
                    externalInfo_.actualRFID = "";
                }
                {
                    ;//取出存取口的所有档案
                }

                break;
            }
            default:
                break;
        }
    }
}



void Robot::onConnection(const TcpConnectionPtr& conn) {
    LOG_INFO << conn->localAddress().toIpPort() << " -> "
             << conn->peerAddress().toIpPort() << " is "
             << (conn->connected() ? "UP" : "DOWN");

    {
        MutexLockGuard lock(connectionMutex_);
        if (conn->connected()) {
            connection_ = conn;
        } else {
            connection_.reset();
        }
    }

    if(conn->connected()) {
        StringPiece message(connectMsg_, sizeof(connectMsg_));
        write(message);
    }
}

void Robot::onCompleteMessage(const muduo::net::TcpConnectionPtr&,
                            const muduo::string& message,
                            Timestamp) {
    const char *data = message.c_str();
    const u_char taskType = data[4];
    LOG_INFO << message;
    switch(taskType) {
        //存档任务01
        case DEPOSIT_TASK: {
            char responseData[8];
            for(auto i = 0; i < message.size(); i++) {
                responseData[i] = data[i];
            }
            responseData[6] = 0x01;
            responseData[7] = 0xAA;
            StringPiece responseMsg(responseData, sizeof(responseData));
            write(responseMsg);
            {
                MutexLockGuard lock(taskMessageMutex_);
                taskMessage_ = message;
                taskCondition_.notifyAll();
            }
            break;
        }
        //取档任务02
        case WITHDRAW_TASK: {
            char responseData[8];
            for(auto i = 0; i < message.size(); i++) {
                responseData[i] = data[i];
            }
            responseData[6] = 0x01;
            responseData[7] = 0xAA;
            StringPiece responseMsg(responseData, sizeof(responseData));
            write(responseMsg);
            {
                MutexLockGuard lock(taskMessageMutex_);
                taskMessage_ = message;
                taskCondition_.notifyAll();
            }
            break;
        }
        //充电任务03
        case CHARGE_TASK: {
            char responseData[8];
            for(auto i = 0; i < message.size(); i++) {
                responseData[i] = data[i];
            }
            responseData[6] = 0x01;
            responseData[7] = 0xAA;
            StringPiece responseMsg(responseData, sizeof(responseData));
            write(responseMsg);
            {
                MutexLockGuard lock(taskMessageMutex_);
                taskMessage_ = message;
                taskCondition_.notifyAll();
            }
            break;
        }
        //外部发送查询指令04
        case INQUIRE: {
            //回复收到
            char responseData1[8];
            for(auto i = 0; i < message.size(); i++) {
                responseData1[i] = data[i];
            }
            responseData1[6] = 0x01;
            responseData1[7] = 0xAA;
            StringPiece responseMsg1(responseData1, sizeof(responseData1));
            write(responseMsg1);
            //回复机器人状态
            char responseData2[9];
            for(auto i = 0; i < message.size(); i++) {
                responseData2[i] = data[i];
            }
            responseData2[6] = 0x02;
            {
                MutexLockGuard lock(robotInfoMutex_);
                responseData2[7] = robotInfo_.currentState;
                responseData2[8] = robotInfo_.currentPower;
            }
            StringPiece responseMsg2(responseData2, sizeof(responseData2));
            write(responseMsg2);
            break;
        }
        //外部发送校验的RFID消息05
        case RFID_INFO: {
            char responseData[8];
            for(auto i = 0; i < message.size(); i++) {
                responseData[i] = data[i];
            }
            responseData[6] = 0x01;
            responseData[7] = 0xAA;
            StringPiece responseMsg(responseData, sizeof(responseData));
            write(responseMsg);
            {
                MutexLockGuard lock(externalInfoMutex_);
                externalInfo_.actualRFID = message;
                externalCondition_.notifyAll();
            }
            break;
        }
        //单本动作完成外部接收完成06
        case SINGLE_ARCHIVE_FINISH: {
            char responseData[8];
            for(auto i = 0; i < message.size(); i++) {
                responseData[i] = data[i];
            }
            responseData[6] = 0x01;
            responseData[7] = 0xAA;
            StringPiece responseMsg(responseData, sizeof(responseData));
            write(responseMsg);
            {
                MutexLockGuard lock(externalInfoMutex_);
                externalInfo_.singleArchiveFinshed = true;
                externalCondition_.notifyAll();
            }
            break;
        }
        //外部发送档案柜就绪状态07
        case CAB_READY: {
            char responseData[8];
            for(auto i = 0; i < message.size(); i++) {
                responseData[i] = data[i];
            }
            responseData[6] = 0x01;
            responseData[7] = 0xAA;
            StringPiece responseMsg(responseData, sizeof(responseData));
            write(responseMsg);
            {
                MutexLockGuard lock(externalInfoMutex_);
                externalInfo_.readyCab.insert(data[7]);
                externalCondition_.notifyAll();
            }
            break;
        }
        //结束任务08
        case ALL_FINISH: {
            char responseData[8];
            for(auto i = 0; i < message.size(); i++) {
                responseData[i] = data[i];
            }
            responseData[6] = 0x01;
            responseData[7] = 0xAA;
            StringPiece responseMsg(responseData, sizeof(responseData));
            write(responseMsg);
            {
                MutexLockGuard lock(taskMessageMutex_);
                taskMessage_ = message;
                taskCondition_.notifyAll();
            }
        }
        default:
            break;
    }
}

void Robot::write(const StringPiece& message)
{
    MutexLockGuard lock(connectionMutex_);
    if (connection_)
    {
        codec_.send(get_pointer(connection_), message);
    }
}