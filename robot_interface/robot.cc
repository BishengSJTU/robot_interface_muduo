#include "robot.h"

const char Robot::connectMsg_[8] = {static_cast<char>(0xA1), 0x00, 0x00, 0x00, 0x11, 0x00, 0x01 , static_cast<char>(0xAA)};

Robot::Robot():
codec_(std::bind(&Robot::onCompleteMessage, this, muduo::_1, muduo::_2, muduo::_3)),
taskCondition_(taskMessageMutex_)
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

        // 等待任务消息到来，否则此线程阻塞
        {
            MutexLockGuard lock(taskMessageMutex_);
            while (taskMessage_.size() == 0) {
                taskCondition_.wait();
            }
            task = taskMessage_;
            taskMessage_ = "";
        }
        // 判断此时机器人状态是否空闲
        {
            MutexLockGuard lock(robotInfoMutex_);
            u_char 
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
        case 0x01: {
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
        case 0x02: {
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
        case 0x03: {
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
        case 0x04: {
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
        case 0x05: {
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
            }
            break;
        }
        //单本动作完成外部接收完成06
        case 0x06: {
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
            }
            break;
        }
        //外部发送档案柜就绪状态07
        case 0x07: {
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
            }
            break;
        }
        //结束任务08
        case 0x08: {
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