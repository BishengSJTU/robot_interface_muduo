#include "jaka_robot_interface.h"

RobotInterface::RobotInterface(std::string path):
codec_(std::bind(&RobotInterface::onCompleteMessage, this, muduo::_1, muduo::_2, muduo::_3)),
taskCondition_(taskMessageMutex_),
externalCondition_(externalInfoMutex_),
config_(path + "/FixedConfig.YML"),
pickAndPlace_(path),
agv_(path)
{
    taskServerIP_ = config_.get<std::string>("MIDDLE_LAYER_IP");
    taskServerPort_ = config_.get<int>("MIDDLE_LAYER_PORT");
    {
        MutexLockGuard lock(externalInfoMutex_);
        externalInfo_.readyCab = {};
        externalInfo_.withdrawCheckReceived = false;
        externalInfo_.withrdrawCheckResult = false;
        externalInfo_.singleArchiveFinishedReceived = false;
    }

}

void RobotInterface::eventLoopThread() {
    muduo::net::EventLoop loop;
    muduo::net::InetAddress serverAddr(taskServerIP_, taskServerPort_);
    muduo::net::TcpClient client(&loop, serverAddr, "TcpClient");
    client.setConnectionCallback(std::bind(&RobotInterface::onConnection, this, muduo::_1));
    client.setMessageCallback(std::bind(&LengthHeaderCodec::onMessage, &codec_, muduo::_1, muduo::_2, muduo::_3));
    client.enableRetry();
    client.connect();
    loop.loop();
}

void RobotInterface::execTaskThread() {
    u_char singleFinishMsgArray[8] = {0xA1, 0x00, 0x00, 0x01, 0x06, 0x00, 0x01, 0x00};
    u_char stateMsgArray[9] = {0xA1, 0x00, 0x00, 0x00, 0x05, 0x00, 0x02, 0x00, 0x00};

    while (1) {
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
        string response(task.data(), task.size());

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
        if ((state != IS_CHARGING) && (state != IS_FREE)) {
            if (taskType == DEPOSIT_TASK || taskType == WITHDRAW_TASK) {
                for (int64_t i = 0; i < temPositionTotalNum; i++) {
                    if (response[7 + 18 * i] = 0xA1)
                        response[7 + 18 * i] = 0x00;
                }
            } else if (taskType == CHARGE_TASK || taskType == DEPOSIT_PREPARE_TASK) {
                response[7] = 0x00;
            }
            StringPiece responseMsg(response);
            write(responseMsg);
            {
                std::stringstream ss;
                for(auto i = 0; i < responseMsg.size(); i++) {
                    ss << std::hex << (unsigned int)(unsigned char)responseMsg[i] << ",";
                }
                std::string log = ss.str();
                LOG_INFO << "发送：" << log;
            }
            continue;
        }
        // 如果正在充电或准备充电，忽略充电任务
        if (state == IS_CHARGING || state == CHARGE_PREPARE) {
            if (taskType == CHARGE_TASK) {
                response[7] = 0x00;
                StringPiece responseMsg(response);
                write(responseMsg);
                {
                    std::stringstream ss;
                    for(auto i = 0; i < responseMsg.size(); i++) {
                        ss << std::hex << (unsigned int)(unsigned char)responseMsg[i] << ",";
                    }
                    std::string log = ss.str();
                    LOG_INFO << "发送：" << log;
                }
                continue;
            }
        }
        // 电量小于最低电量，只接收充电任务
        if (power <= powerLowerLimit) {
            if (taskType == CHARGE_TASK) {
                {
                    MutexLockGuard lock(robotInfoMutex_);
                    robotInfo_.currentState = CHARGE_PREPARE;
                }
                ///执行充电指令
                int agv_mission_id;
                agv_.AgvGo(0, 2, agv_mission_id);
                {
                    MutexLockGuard lock(robotInfoMutex_);
                    robotInfo_.currentState = IS_CHARGING;
                }
            } else if (taskType == DEPOSIT_TASK || taskType == WITHDRAW_TASK) {
                for (int64_t i = 0; i < temPositionTotalNum; i++) {
                    if (response[7 + 18 * i] = 0xA1)
                        response[7 + 18 * i] = 0x00;
                }
            } else if (taskType == DEPOSIT_PREPARE_TASK) {
                response[7] = 0x00;
            }
            StringPiece responseMsg(response);
            write(responseMsg);
            {
                std::stringstream ss;
                for(auto i = 0; i < responseMsg.size(); i++) {
                    ss << std::hex << (unsigned int)(unsigned char)responseMsg[i] << ",";
                }
                std::string log = ss.str();
                LOG_INFO << "发送：" << log;
            }
            continue;
        }
        // 可执行任务状态
        switch (taskType) {
            //存档准备
            case DEPOSIT_PREPARE_TASK: {
                ///跑到窗口
                int agv_mission_id;
                agv_.AgvGo(0, 1, agv_mission_id);
                agv_.AgvReached();

                response[7] = 0x01;
                StringPiece responseMsg(response);
                write(responseMsg);
                {
                    std::stringstream ss;
                    for(auto i = 0; i < responseMsg.size(); i++) {
                        ss << std::hex << (unsigned int)(unsigned char)responseMsg[i] << ",";
                    }
                    std::string log = ss.str();
                    LOG_INFO << "发送：" << log;
                }
            }
            //存档
            case DEPOSIT_TASK: {
                {
                    MutexLockGuard lock(robotInfoMutex_);
                    robotInfo_.currentState = IS_DEPOSITING;
                    stateMsgArray[7] = robotInfo_.currentPower;
                    stateMsgArray[8] = IS_DEPOSITING;
                }
                write(stateMsgArray);
                {
                    std::stringstream ss;
                    for(auto i = 0; i < sizeof(stateMsgArray); i++) {
                        ss << std::hex << (unsigned int)(unsigned char)stateMsgArray[i] << ",";
                    }
                    std::string log = ss.str();
                    LOG_INFO << "发送：" << log;
                }

                //机械臂伸展
                pickAndPlace_.JAKAStretch();
                for(int64_t archive = 0; archive < temPositionTotalNum; archive++) {
                    if(data[7 + 18 * archive] == 0xA1) {
                    }
                }
                break;
            }

            // 取档
            case WITHDRAW_TASK: {
                {
                    MutexLockGuard lock(robotInfoMutex_);
                    robotInfo_.currentState = IS_WITHDRAWING;
                    stateMsgArray[7] = robotInfo_.currentPower;
                    stateMsgArray[8] = 0x02;
                }
                write(stateMsgArray);
                {
                    std::stringstream ss;
                    for(auto i = 0; i < sizeof(stateMsgArray); i++) {
                        ss << std::hex << (unsigned int)(unsigned char)stateMsgArray[i] << ",";
                    }
                    std::string log = ss.str();
                    LOG_INFO << "发送：" << log;
                }

                //根据任务清单将所有档案归类
                int64_t turntablePosition = 0;
                std::map<int64_t, std::vector<int64_t> > cabIDTurntablePositionsMap;
                for (int64_t i = 0; i < turntablePositionTotalNum; i++) {
                    if (data[7 + 2 * i] == 0xA1) {
                        turntablePosition++;
                        int64_t cabID = data[8 + 2 * i];
                        if (cabIDTurntablePositionsMap.find(cabID) == cabIDTurntablePositionsMap.end()) {
                            std::vector<int64_t> turntablePositions;
                            turntablePositions.push_back(turntablePosition);
                            cabIDTurntablePositionsMap.insert(std::make_pair(cabID, turntablePositions));
                        } else {
                            cabIDTurntablePositionsMap.find(cabID)->second.push_back(turntablePosition);
                        }
                    }
                }

                for (auto cabIDTurntablePositions : cabIDTurntablePositionsMap) {
                    int64_t cabID = cabIDTurntablePositions.first;
                    std::vector<int64_t> turntablePositions = cabIDTurntablePositions.second;; // 跑到对应柜子前
                    //　等待档案柜就绪
                    {
                        MutexLockGuard lock(externalInfoMutex_);
                        while (externalInfo_.readyCab.find(cabID) == externalInfo_.readyCab.end()) {
                            externalCondition_.wait();
                        }
                        externalInfo_.readyCab.erase(cabID);
                    }

                    //此柜需要操作的总的档案数量
                    int64_t operationArchiveNum = turntablePositions.size();

                    // 额外等待档案柜就绪次数
                    int64_t extraOperationTimes =
                            (operationArchiveNum % cabPositionTotalNum == 0)
                            ? (operationArchiveNum / cabPositionTotalNum - 1)
                            : (operationArchiveNum / cabPositionTotalNum);


                    for (int64_t turntableNum = 1;
                         turntableNum <= operationArchiveNum; turntableNum++) { ; //将档案从柜中放入转盘中
                        // 额外等待次数>0并且本轮操作结束，等待档案柜就绪
                        if (extraOperationTimes > 0 && (turntableNum % cabPositionTotalNum == 0)) {
                            singleFinishMsgArray[7] = cabID;
                            write(singleFinishMsgArray);
                            {
                                std::stringstream ss;
                                for(auto i = 0; i < sizeof(singleFinishMsgArray); i++) {
                                    ss << std::hex << (unsigned int)(unsigned char)singleFinishMsgArray[i] << ",";
                                }
                                std::string log = ss.str();
                                LOG_INFO << "发送：" << log;
                            }
                            {
                                MutexLockGuard lock(externalInfoMutex_);
                                while (externalInfo_.singleArchiveFinshed == false) {
                                    externalCondition_.wait();
                                }
                                externalInfo_.singleArchiveFinshed = false;
                            }
                            {
                                MutexLockGuard lock(externalInfoMutex_);
                                while (externalInfo_.readyCab.find(cabID) == externalInfo_.readyCab.end()) {
                                    externalCondition_.wait();
                                }
                                externalInfo_.readyCab.erase(cabID);
                                extraOperationTimes--;
                            }
                        }
                    }

                    singleFinishMsgArray[7] = cabID;
                    write(singleFinishMsgArray);
                    {
                        std::stringstream ss;
                        for(auto i = 0; i < sizeof(singleFinishMsgArray); i++) {
                            ss << std::hex << (unsigned int)(unsigned char)singleFinishMsgArray[i] << ",";
                        }
                        std::string log = ss.str();
                        LOG_INFO << "发送：" << log;
                    }
                    {
                        MutexLockGuard lock(externalInfoMutex_);
                        while (externalInfo_.singleArchiveFinshed == false) {
                            externalCondition_.wait();
                        }
                        externalInfo_.singleArchiveFinshed = false;
                    }
                }

                ; /// 跑到存取口前，将档案放入

                for (int64_t i = 0; i < turntablePositionTotalNum; i++) {
                    if (response[7 + 2 * i] = 0xA1)
                        response[7 + 2 * i] = 0x01;
                }
                StringPiece responseMsg(response);
                write(responseMsg);
                {
                    std::stringstream ss;
                    for(auto i = 0; i < responseMsg.size(); i++) {
                        ss << std::hex << (unsigned int)(unsigned char)responseMsg[i] << ",";
                    }
                    std::string log = ss.str();
                    LOG_INFO << "发送：" << log;
                }
                break;
            }

            case CHARGE_TASK: {
                if(data[7] == 0xA1) {
                    {
                        MutexLockGuard lock(robotInfoMutex_);
                        robotInfo_.currentState = CHARGE_PREPARE;
                        stateMsgArray[7] = robotInfo_.currentPower;
                        stateMsgArray[8] = 0x07;
                    }
                    write(stateMsgArray);
                    {
                        std::stringstream ss;
                        for(auto i = 0; i < sizeof(stateMsgArray); i++) {
                            ss << std::hex << (unsigned int)(unsigned char)stateMsgArray[i] << ",";
                        }
                        std::string log = ss.str();
                        LOG_INFO << "发送：" << log;
                    }
                    ; ///　跑到充电桩前，完成充电动作

                    response[7] = 0x01;
                    StringPiece responseMsg(response);
                    write(responseMsg);
                    {
                        std::stringstream ss;
                        for(auto i = 0; i < responseMsg.size(); i++) {
                            ss << std::hex << (unsigned int)(unsigned char)responseMsg[i] << ",";
                        }
                        std::string log = ss.str();
                        LOG_INFO << "发送：" << log;
                    }
                    {
                        MutexLockGuard lock(robotInfoMutex_);
                        robotInfo_.currentState = IS_CHARGING;
                        stateMsgArray[7] = robotInfo_.currentPower;
                        stateMsgArray[8] = 0x03;
                    }
                    write(stateMsgArray);
                    {
                        std::stringstream ss;
                        for(auto i = 0; i < sizeof(stateMsgArray); i++) {
                            ss << std::hex << (unsigned int)(unsigned char)stateMsgArray[i] << ",";
                        }
                        std::string log = ss.str();
                        LOG_INFO << "发送：" << log;
                    }
                }
                else if(data[7] == 0xA0) {
                    {
                        MutexLockGuard lock(robotInfoMutex_);
                        robotInfo_.currentState = CHARGE_FINISHED;
                        stateMsgArray[7] = robotInfo_.currentPower;
                        stateMsgArray[8] = 0x08;
                    }
                    write(stateMsgArray);
                    {
                        std::stringstream ss;
                        for(auto i = 0; i < sizeof(stateMsgArray); i++) {
                            ss << std::hex << (unsigned int)(unsigned char)stateMsgArray[i] << ",";
                        }
                        std::string log = ss.str();
                        LOG_INFO << "发送：" << log;
                    }
                    ; ///　跑到休息区

                    response[7] = 0x01;
                    StringPiece responseMsg(response);
                    write(responseMsg);
                    {
                        std::stringstream ss;
                        for(auto i = 0; i < responseMsg.size(); i++) {
                            ss << std::hex << (unsigned int)(unsigned char)responseMsg[i] << ",";
                        }
                        std::string log = ss.str();
                        LOG_INFO << "发送：" << log;
                    }
                    {
                        MutexLockGuard lock(robotInfoMutex_);
                        robotInfo_.currentState = IS_FREE;
                        stateMsgArray[7] = robotInfo_.currentPower;
                        stateMsgArray[8] = 0x00;
                    }
                    write(stateMsgArray);
                    {
                        std::stringstream ss;
                        for(auto i = 0; i < sizeof(stateMsgArray); i++) {
                            ss << std::hex << (unsigned int)(unsigned char)stateMsgArray[i] << ",";
                        }
                        std::string log = ss.str();
                        LOG_INFO << "发送：" << log;
                    }
                }
                break;
            }

            default: {
                break;
            }
        }
    }
}



void RobotInterface::onConnection(const TcpConnectionPtr& conn) {
    u_char connectMsg_[8] = {0xA1, 0x00, 0x00, 0x00, 0x11, 0x00, 0x01 , 0xAA};

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
        StringPiece message(connectMsg_);
        write(message);
        {
            std::stringstream ss;
            for(auto i = 0; i < message.size(); i++) {
                ss << std::hex << (unsigned int)(unsigned char)message[i] << ",";
            }
            std::string log = ss.str();
            LOG_INFO << "发送：" << log;
        }
    }
}

void RobotInterface::onCompleteMessage(const muduo::net::TcpConnectionPtr&,
                            const muduo::string& message,
                            Timestamp) {
    {
        std::stringstream ss;
        for(auto i = 0; i < message.size(); i++) {
            ss << std::hex << (unsigned int)(unsigned char)message[i] << ",";
        }
        std::string log = ss.str();
        LOG_INFO << "接收：" << log;
    }

    const char *data = message.c_str();
    const u_char taskType = data[4];
    switch(taskType) {
        //心跳33
        case HEAR_BEAT: {
            assert(message.size() == 8);
            u_char responseData[8];
            for(auto i = 0; i < sizeof(responseData); i++) {
                responseData[i] = data[i];
            }
            StringPiece responseMsg(responseData);
            write(responseMsg);
            {
                std::stringstream ss;
                for(auto i = 0; i < responseMsg.size(); i++) {
                    ss << std::hex << (unsigned int)(unsigned char)responseMsg[i] << ",";
                }
                std::string log = ss.str();
                LOG_INFO << "发送：" << log;
            }
        }
        //存档任务01，需执行！
        case DEPOSIT_TASK: {
            assert(message.size() == 97);
            u_char responseData[8];
            for(auto i = 0; i < sizeof(responseData); i++) {
                responseData[i] = data[i];
            }
            responseData[6] = 0x01;
            responseData[7] = 0xAA;
            StringPiece responseMsg(responseData);
            write(responseMsg);
            {
                std::stringstream ss;
                for(auto i = 0; i < responseMsg.size(); i++) {
                    ss << std::hex << (unsigned int)(unsigned char)responseMsg[i] << ",";
                }
                std::string log = ss.str();
                LOG_INFO << "发送：" << log;
            }
            {
                MutexLockGuard lock(taskMessageMutex_);
                taskMessage_ = message;
                taskCondition_.notifyAll();
            }
            break;
        }
        //取档任务02，需执行！
        case WITHDRAW_TASK: {
            assert(message.size() == 97);
            u_char responseData[8];
            for(auto i = 0; i < sizeof(responseData); i++) {
                responseData[i] = data[i];
            }
            responseData[6] = 0x01;
            responseData[7] = 0xAA;
            StringPiece responseMsg(responseData);
            write(responseMsg);
            {
                std::stringstream ss;
                for(auto i = 0; i < responseMsg.size(); i++) {
                    ss << std::hex << (unsigned int)(unsigned char)responseMsg[i] << ",";
                }
                std::string log = ss.str();
                LOG_INFO << "发送：" << log;
            }
            {
                MutexLockGuard lock(taskMessageMutex_);
                taskMessage_ = message;
                taskCondition_.notifyAll();
            }
            break;
        }
        //充电任务03，需执行！
        case CHARGE_TASK: {
            assert(message.size() == 8);
            u_char responseData[8];
            for(auto i = 0; i < sizeof(responseData); i++) {
                responseData[i] = data[i];
            }
            responseData[6] = 0x01;
            responseData[7] = 0xAA;
            StringPiece responseMsg(responseData);
            write(responseMsg);
            {
                std::stringstream ss;
                for(auto i = 0; i < responseMsg.size(); i++) {
                    ss << std::hex << (unsigned int)(unsigned char)responseMsg[i] << ",";
                }
                std::string log = ss.str();
                LOG_INFO << "发送：" << log;
            }
            {
                MutexLockGuard lock(taskMessageMutex_);
                taskMessage_ = message;
                taskCondition_.notifyAll();
            }
            break;
        }
        //存档校验指令04
        case DEPOSIT_CHECK: {
            assert(message.size() == 82);
            //回复收到
            u_char responseData1[8];
            for(auto i = 0; i < sizeof(responseData1); i++) {
                responseData1[i] = data[i];
            }
            responseData1[6] = 0x01;
            responseData1[7] = 0xAA;
            StringPiece responseMsg1(responseData1);
            write(responseMsg1);
            {
                std::stringstream ss;
                for(auto i = 0; i < responseMsg1.size(); i++) {
                    ss << std::hex << (unsigned int)(unsigned char)responseMsg1[i] << ",";
                }
                std::string log = ss.str();
                LOG_INFO << "发送：" << log;
            }
            //直接回复存档校验结果
            u_char responseData2[82];
            for(auto i = 0; i < sizeof(responseData2); i++) {
                responseData2[i] = data[i];
            }
            for(int64_t archive = 0; archive < temPositionTotalNum; archive++) {
                if(data[7 + 15 * archive] == 0xA1) {
                    responseData2[7 + 15 * archive] = 0x01;
                }
            }
            StringPiece responseMsg2(responseData2);
            write(responseMsg2);
            {
                std::stringstream ss;
                for(auto i = 0; i < responseMsg2.size(); i++) {
                    ss << std::hex << (unsigned int)(unsigned char)responseMsg2[i] << ",";
                }
                std::string log = ss.str();
                LOG_INFO << "发送：" << log;
            }
            break;
        }
        //外部发送查询指令05
        case INQUIRE: {
            assert(message.size() == 8);
            //回复收到
            u_char responseData1[8];
            for(auto i = 0; i < sizeof(responseData1); i++) {
                responseData1[i] = data[i];
            }
            responseData1[6] = 0x01;
            responseData1[7] = 0xAA;
            StringPiece responseMsg1(responseData1);
            write(responseMsg1);
            {
                std::stringstream ss;
                for(auto i = 0; i < responseMsg1.size(); i++) {
                    ss << std::hex << (unsigned int)(unsigned char)responseMsg1[i] << ",";
                }
                std::string log = ss.str();
                LOG_INFO << "发送：" << log;
            }
            //回复机器人状态
            u_char responseData2[9];
            for(auto i = 0; i < sizeof(responseData2); i++) {
                responseData2[i] = data[i];
            }
            responseData2[6] = 0x02;
            {
                MutexLockGuard lock(robotInfoMutex_);
                responseData2[7] = robotInfo_.currentState;
                responseData2[8] = robotInfo_.currentPower;
            }
            StringPiece responseMsg2(responseData2);
            write(responseMsg2);
            {
                std::stringstream ss;
                for(auto i = 0; i < responseMsg2.size(); i++) {
                    ss << std::hex << (unsigned int)(unsigned char)responseMsg2[i] << ",";
                }
                std::string log = ss.str();
                LOG_INFO << "发送：" << log;
            }
            break;
        }
        //取档校验指令06
        case WITHDRAW_CHECK: {
            assert(message.size() == 20);
            if(data[19] == 0xAA)
            {
                MutexLockGuard lock(externalInfoMutex_);
                externalInfo_.withdrawCheckReceived = true;
                externalCondition_.notifyAll();
            }
            if(data[19] == 0x01)
            {
                MutexLockGuard lock(externalInfoMutex_);
                externalInfo_.withrdrawCheckResult = true;
                externalCondition_.notifyAll();
            }
        }
        // 存档准备任务07，需执行！
        case DEPOSIT_PREPARE_TASK: {
            assert(message.size() == 8);
            u_char responseData[8];
            for(auto i = 0; i < sizeof(responseData); i++) {
                responseData[i] = data[i];
            }
            responseData[6] = 0x01;
            responseData[7] = 0xAA;
            StringPiece responseMsg(responseData);
            write(responseMsg);
            {
                std::stringstream ss;
                for(auto i = 0; i < responseMsg.size(); i++) {
                    ss << std::hex << (unsigned int)(unsigned char)responseMsg[i] << ",";
                }
                std::string log = ss.str();
                LOG_INFO << "发送：" << log;
            }
            {
                MutexLockGuard lock(taskMessageMutex_);
                taskMessage_ = message;
                taskCondition_.notifyAll();
            }
            break;
        }
        //外部接收到单本动作取放完成08
        case SINGLE_ARCHIVE_FINISH: {
            assert(message.size() == 22);
            {
                MutexLockGuard lock(externalInfoMutex_);
                externalInfo_.singleArchiveFinishedReceived = true;
                externalCondition_.notifyAll();
            }
            break;
        }
        //取消任务指令09
        case CANCEL_TASK: {
            assert(message.size() == 10);
            u_char responseData1[8];
            for (auto i = 0; i < sizeof(responseData1); i++) {
                responseData1[i] = data[i];
            }
            responseData1[6] = 0x01;
            responseData1[7] = 0xAA;
            StringPiece responseMsg1(responseData1);
            write(responseMsg1);
            {
                std::stringstream ss;
                for (auto i = 0; i < responseMsg1.size(); i++) {
                    ss << std::hex << (unsigned int) (unsigned char) responseMsg1[i] << ",";
                }
                std::string log = ss.str();
                LOG_INFO << "发送：" << log;
            }

            u_char responseData2[8];
            for (auto i = 0; i < sizeof(responseData2); i++) {
                responseData2[i] = data[i];
            }
            responseData2[6] = 0x01;
            responseData2[7] = 0x01;
            StringPiece responseMsg2(responseData2);
            write(responseMsg2);
            {
                std::stringstream ss;
                for (auto i = 0; i < responseMsg2.size(); i++) {
                    ss << std::hex << (unsigned int) (unsigned char) responseMsg2[i] << ",";
                }
                std::string log = ss.str();
                LOG_INFO << "发送：" << log;
            }
        }
        // 档案柜就绪状态0A
        case CAB_STATE: {
            assert(message.size() == 9);
            u_char responseData[8];
            for (auto i = 0; i < sizeof(responseData); i++) {
                responseData[i] = data[i];
            }
            responseData[6] = 0x01;
            responseData[7] = 0xAA;
            StringPiece responseMsg(responseData);
            write(responseMsg);
            {
                std::stringstream ss;
                for (auto i = 0; i < responseMsg.size(); i++) {
                    ss << std::hex << (unsigned int) (unsigned char) responseMsg[i] << ",";
                }
                std::string log = ss.str();
                LOG_INFO << "发送：" << log;
            }
            {
                MutexLockGuard lock(externalInfoMutex_);
                externalInfo_.readyCab.insert(data[7]);
                externalCondition_.notifyAll();
            }
        }
        default:
            break;
    }
}

void RobotInterface::write(const StringPiece& message)
{
    MutexLockGuard lock(connectionMutex_);
    if (connection_)
    {
        codec_.send(get_pointer(connection_), message);
    }
}