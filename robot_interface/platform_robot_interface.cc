#include "platform_robot_interface.h"

RobotInterface::RobotInterface():
codec_(std::bind(&RobotInterface::onCompleteMessage, this, muduo::_1, muduo::_2, muduo::_3)),
taskCondition_(taskMessageMutex_),
externalCondition_(externalInfoMutex_)
{
    {
        MutexLockGuard lock(externalInfoMutex_);
        externalInfo_.readyCab = {};
        externalInfo_.actualRFID = "";
        externalInfo_.singleArchiveFinishedReceived = false;
    }

}

void RobotInterface::eventLoopThread() {
    muduo::net::EventLoop loop;
    muduo::net::InetAddress serverAddr("127.0.0.1", 9981);
    muduo::net::TcpClient client(&loop, serverAddr, "TcpClient");
    client.setConnectionCallback(std::bind(&RobotInterface::onConnection, this, muduo::_1));
    client.setMessageCallback(std::bind(&LengthHeaderCodec::onMessage, &codec_, muduo::_1, muduo::_2, muduo::_3));
    client.enableRetry();
    client.connect();
    loop.loop();
}

void RobotInterface::execTaskThread() {
    u_char singleFinishMsgArray[8] = {0xA1, 0x00, 0x00, 0x01, 0x06, 0x00, 0x01, 0x00};
    u_char stateMsgArray[9] = {0xA1, 0x00, 0x00, 0x00, 0x04, 0x00, 0x02, 0x00, 0x00};

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
                for (int i = 0; i < turntablePositionTotalNum; i++) {
                    if (data[7 + 2 * i] == 0xA1)
                        response[7 + 2 * i] = 0x00;
                }
            } else if (taskType == CHARGE_TASK || taskType == ALL_FINISH_TASK) {
                response[7] = 0x00;
            }
            StringPiece responseMsg(response);
            write(responseMsg);
            {
                std::stringstream ss;
                for(int i = 0; i < responseMsg.size(); i++) {
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
                if(data[7] == 0xA1) {
                    response[7] = 0x00;
                    StringPiece responseMsg(response);
                    write(responseMsg);
                    {
                        std::stringstream ss;
                        for (int i = 0; i < responseMsg.size(); i++) {
                            ss << std::hex << (unsigned int) (unsigned char) responseMsg[i] << ",";
                        }
                        std::string log = ss.str();
                        LOG_INFO << "发送：" << log;
                    }
                    continue;
                }
            }
        }
        // 电量小于最低电量，只接收充电任务
        if (power <= powerLowerLimit) {
            if (taskType == CHARGE_TASK) {
                {
                    MutexLockGuard lock(robotInfoMutex_);
                    robotInfo_.currentState = CHARGE_PREPARE;
                }
                ; ///执行充电指令
                {
                    MutexLockGuard lock(robotInfoMutex_);
                    robotInfo_.currentState = IS_CHARGING;
                }
            } else if (taskType == DEPOSIT_TASK || taskType == WITHDRAW_TASK) {
                for (int i = 0; i < turntablePositionTotalNum; i++) {
                    if (data[7 + 2 * i] == 0xA1)
                        response[7 + 2 * i] = 0x00;
                }
            } else if (taskType == ALL_FINISH_TASK) {
                response[7] = 0x00;
            }
            StringPiece responseMsg(response);
            write(responseMsg);
            {
                std::stringstream ss;
                for(int i = 0; i < responseMsg.size(); i++) {
                    ss << std::hex << (unsigned int)(unsigned char)responseMsg[i] << ",";
                }
                std::string log = ss.str();
                LOG_INFO << "发送：" << log;
            }
            continue;
        }
        // 可执行任务状态
        switch (taskType) {
            //存档
            case DEPOSIT_TASK: {
                {
                    MutexLockGuard lock(robotInfoMutex_);
                    robotInfo_.currentState = IS_DEPOSITING;
                    stateMsgArray[7] = robotInfo_.currentPower;
                    stateMsgArray[8] = 0x01;
                }
                write(stateMsgArray);
                {
                    std::stringstream ss;
                    for(int i = 0; i < sizeof(stateMsgArray); i++) {
                        ss << std::hex << (unsigned int)(unsigned char)stateMsgArray[i] << ",";
                    }
                    std::string log = ss.str();
                    LOG_INFO << "发送：" << log;
                }

                ; /// 跑到窗口
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
                //根据RFID结果取出存取口的所有档案
                const char *dataRFID = actualRFID.data();
                int turntablePosition = 0;
                std::map<int, std::vector<int> > cabIDTurntablePositionsMap;
                for (int i = 0; i < turntablePositionTotalNum; i++) {
                    if (data[7 + 2 * i] == 0xA1) {
                        turntablePosition++;;//取出档案
                        int cabID = data[8 + 2 * i];
                        if (cabIDTurntablePositionsMap.find(cabID) == cabIDTurntablePositionsMap.end()) {
                            std::vector<int> turntablePositions;
                            turntablePositions.push_back(turntablePosition);
                            cabIDTurntablePositionsMap.insert(std::make_pair(cabID, turntablePositions));
                        } else {
                            cabIDTurntablePositionsMap.find(cabID)->second.push_back(turntablePosition);
                        }
                    }
                }

                for (auto cabIDTurntablePositions : cabIDTurntablePositionsMap) {
                    int cabID = cabIDTurntablePositions.first;
                    std::vector<int> turntablePositions = cabIDTurntablePositions.second;; // 跑到对应柜子前
                    //　等待档案柜就绪
                    {
                        MutexLockGuard lock(externalInfoMutex_);
                        while (externalInfo_.readyCab.find(cabID) == externalInfo_.readyCab.end()) {
                            externalCondition_.wait();
                        }
                        externalInfo_.readyCab.erase(cabID);
                    }

                    //此柜需要操作的总的档案数量
                    int operationArchiveNum = turntablePositions.size();

                    // 额外等待档案柜就绪次数
                    int extraOperationTimes =
                            (operationArchiveNum % cabPositionTotalNum == 0)
                            ? (operationArchiveNum / cabPositionTotalNum - 1)
                            : (operationArchiveNum / cabPositionTotalNum);


                    for (int turntableNum = 1;
                         turntableNum <= operationArchiveNum; turntableNum++) {
                        ; /// 将档案从转盘中放入柜中
                        // 额外等待次数>0并且本轮操作结束，等待档案柜就绪
                        if (extraOperationTimes > 0 && (turntableNum % cabPositionTotalNum == 0)) {
                            singleFinishMsgArray[7] = cabID;
                            write(singleFinishMsgArray);
                            {
                                std::stringstream ss;
                                for(int i = 0; i < sizeof(singleFinishMsgArray); i++) {
                                    ss << std::hex << (unsigned int)(unsigned char)singleFinishMsgArray[i] << ",";
                                }
                                std::string log = ss.str();
                                LOG_INFO << "发送：" << log;
                            }
                            {
                                MutexLockGuard lock(externalInfoMutex_);
                                while (externalInfo_.singleArchiveFinishedReceived == false) {
                                    externalCondition_.wait();
                                }
                                externalInfo_.singleArchiveFinishedReceived = false;
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
                        for(int i = 0; i < sizeof(singleFinishMsgArray); i++) {
                            ss << std::hex << (unsigned int)(unsigned char)singleFinishMsgArray[i] << ",";
                        }
                        std::string log = ss.str();
                        LOG_INFO << "发送：" << log;
                    }

                    {
                        MutexLockGuard lock(externalInfoMutex_);
                        while (externalInfo_.singleArchiveFinishedReceived == false) {
                            externalCondition_.wait();
                        }
                        externalInfo_.singleArchiveFinishedReceived = false;
                    }
                }

                for (int i = 0; i < turntablePositionTotalNum; i++) {
                    if (data[7 + 2 * i] == 0xA1)
                        response[7 + 2 * i] = 0x01;
                }
                StringPiece responseMsg(response);
                write(responseMsg);
                {
                    std::stringstream ss;
                    for(int i = 0; i < responseMsg.size(); i++) {
                        ss << std::hex << (unsigned int)(unsigned char)responseMsg[i] << ",";
                    }
                    std::string log = ss.str();
                    LOG_INFO << "发送：" << log;
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
                    for(int i = 0; i < sizeof(stateMsgArray); i++) {
                        ss << std::hex << (unsigned int)(unsigned char)stateMsgArray[i] << ",";
                    }
                    std::string log = ss.str();
                    LOG_INFO << "发送：" << log;
                }

                //根据任务清单将所有档案归类
                int turntablePosition = 0;
                std::map<int, std::vector<int> > cabIDTurntablePositionsMap;
                for (int i = 0; i < turntablePositionTotalNum; i++) {
                    if (data[7 + 2 * i] == 0xA1) {
                        turntablePosition++;
                        int cabID = data[8 + 2 * i];
                        if (cabIDTurntablePositionsMap.find(cabID) == cabIDTurntablePositionsMap.end()) {
                            std::vector<int> turntablePositions;
                            turntablePositions.push_back(turntablePosition);
                            cabIDTurntablePositionsMap.insert(std::make_pair(cabID, turntablePositions));
                        } else {
                            cabIDTurntablePositionsMap.find(cabID)->second.push_back(turntablePosition);
                        }
                    }
                }

                for (auto cabIDTurntablePositions : cabIDTurntablePositionsMap) {
                    int cabID = cabIDTurntablePositions.first;
                    std::vector<int> turntablePositions = cabIDTurntablePositions.second;; // 跑到对应柜子前
                    //　等待档案柜就绪
                    {
                        MutexLockGuard lock(externalInfoMutex_);
                        while (externalInfo_.readyCab.find(cabID) == externalInfo_.readyCab.end()) {
                            externalCondition_.wait();
                        }
                        externalInfo_.readyCab.erase(cabID);
                    }

                    //此柜需要操作的总的档案数量
                    int operationArchiveNum = turntablePositions.size();

                    // 额外等待档案柜就绪次数
                    int extraOperationTimes =
                            (operationArchiveNum % cabPositionTotalNum == 0)
                            ? (operationArchiveNum / cabPositionTotalNum - 1)
                            : (operationArchiveNum / cabPositionTotalNum);


                    for (int turntableNum = 1;
                         turntableNum <= operationArchiveNum; turntableNum++) { ; //将档案从柜中放入转盘中
                        // 额外等待次数>0并且本轮操作结束，等待档案柜就绪
                        if (extraOperationTimes > 0 && (turntableNum % cabPositionTotalNum == 0)) {
                            singleFinishMsgArray[7] = cabID;
                            write(singleFinishMsgArray);
                            {
                                std::stringstream ss;
                                for(int i = 0; i < sizeof(singleFinishMsgArray); i++) {
                                    ss << std::hex << (unsigned int)(unsigned char)singleFinishMsgArray[i] << ",";
                                }
                                std::string log = ss.str();
                                LOG_INFO << "发送：" << log;
                            }
                            {
                                MutexLockGuard lock(externalInfoMutex_);
                                while (externalInfo_.singleArchiveFinishedReceived == false) {
                                    externalCondition_.wait();
                                }
                                externalInfo_.singleArchiveFinishedReceived = false;
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
                        for(int i = 0; i < sizeof(singleFinishMsgArray); i++) {
                            ss << std::hex << (unsigned int)(unsigned char)singleFinishMsgArray[i] << ",";
                        }
                        std::string log = ss.str();
                        LOG_INFO << "发送：" << log;
                    }
                    {
                        MutexLockGuard lock(externalInfoMutex_);
                        while (externalInfo_.singleArchiveFinishedReceived == false) {
                            externalCondition_.wait();
                        }
                        externalInfo_.singleArchiveFinishedReceived = false;
                    }
                }

                ; /// 跑到存取口前，将档案放入

                for (int i = 0; i < turntablePositionTotalNum; i++) {
                    if (data[7 + 2 * i] == 0xA1)
                        response[7 + 2 * i] = 0x01;
                }
                StringPiece responseMsg(response);
                write(responseMsg);
                {
                    std::stringstream ss;
                    for(int i = 0; i < responseMsg.size(); i++) {
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
                        for(int i = 0; i < sizeof(stateMsgArray); i++) {
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
                        for(int i = 0; i < responseMsg.size(); i++) {
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
                        for(int i = 0; i < sizeof(stateMsgArray); i++) {
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
                        for(int i = 0; i < sizeof(stateMsgArray); i++) {
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
                        for(int i = 0; i < responseMsg.size(); i++) {
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
                        for(int i = 0; i < sizeof(stateMsgArray); i++) {
                            ss << std::hex << (unsigned int)(unsigned char)stateMsgArray[i] << ",";
                        }
                        std::string log = ss.str();
                        LOG_INFO << "发送：" << log;
                    }
                }
                break;
            }

            case ALL_FINISH_TASK: {
                ;/// 跑到休息区
                response[7] = 0x00;
                StringPiece responseMsg(response);
                write(responseMsg);
                {
                    std::stringstream ss;
                    for(int i = 0; i < responseMsg.size(); i++) {
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
                    for(int i = 0; i < sizeof(stateMsgArray); i++) {
                        ss << std::hex << (unsigned int)(unsigned char)stateMsgArray[i] << ",";
                    }
                    std::string log = ss.str();
                    LOG_INFO << "发送：" << log;
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
            for(int i = 0; i < message.size(); i++) {
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
        for(int i = 0; i < message.size(); i++) {
            ss << std::hex << (unsigned int)(unsigned char)message[i] << ",";
        }
        std::string log = ss.str();
        LOG_INFO << "接收：" << log;
    }

    const char *data = message.c_str();
    const u_char taskType = data[4];
    switch(taskType) {
        //存档任务01
        case DEPOSIT_TASK: {
            assert(message.size() == 31);
            u_char responseData[8];
            for(int i = 0; i < sizeof(responseData); i++) {
                responseData[i] = data[i];
            }
            responseData[6] = 0x01;
            responseData[7] = 0xAA;
            StringPiece responseMsg(responseData);
            write(responseMsg);
            {
                std::stringstream ss;
                for(int i = 0; i < responseMsg.size(); i++) {
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
        //取档任务02
        case WITHDRAW_TASK: {
            assert(message.size() == 31);
            u_char responseData[8];
            for(int i = 0; i < sizeof(responseData); i++) {
                responseData[i] = data[i];
            }
            responseData[6] = 0x01;
            responseData[7] = 0xAA;
            StringPiece responseMsg(responseData);
            write(responseMsg);
            {
                std::stringstream ss;
                for(int i = 0; i < responseMsg.size(); i++) {
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
        //充电任务03
        case CHARGE_TASK: {
            assert(message.size() == 8);
            u_char responseData[8];
            for(int i = 0; i < sizeof(responseData); i++) {
                responseData[i] = data[i];
            }
            responseData[6] = 0x01;
            responseData[7] = 0xAA;
            StringPiece responseMsg(responseData);
            write(responseMsg);
            {
                std::stringstream ss;
                for(int i = 0; i < responseMsg.size(); i++) {
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
        //外部发送查询指令04
        case INQUIRE: {
            assert(message.size() == 8);
            //回复收到
            u_char responseData1[8];
            for(int i = 0; i < sizeof(responseData1); i++) {
                responseData1[i] = data[i];
            }
            responseData1[6] = 0x01;
            responseData1[7] = 0xAA;
            StringPiece responseMsg1(responseData1);
            write(responseMsg1);
            {
                std::stringstream ss;
                for(int i = 0; i < responseMsg1.size(); i++) {
                    ss << std::hex << (unsigned int)(unsigned char)responseMsg1[i] << ",";
                }
                std::string log = ss.str();
                LOG_INFO << "发送：" << log;
            }
            //回复机器人状态
            u_char responseData2[9];
            for(int i = 0; i < sizeof(responseData2); i++) {
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
                for(int i = 0; i < responseMsg2.size(); i++) {
                    ss << std::hex << (unsigned int)(unsigned char)responseMsg2[i] << ",";
                }
                std::string log = ss.str();
                LOG_INFO << "发送：" << log;
            }
            break;
        }
        //外部发送校验的RFID消息05
        case RFID_INFO: {
            assert(message.size() == 31);
            u_char responseData[8];
            for(int i = 0; i < sizeof(responseData); i++) {
                responseData[i] = data[i];
            }
            responseData[6] = 0x01;
            responseData[7] = 0xAA;
            StringPiece responseMsg(responseData);
            write(responseMsg);
            {
                std::stringstream ss;
                for(int i = 0; i < responseMsg.size(); i++) {
                    ss << std::hex << (unsigned int)(unsigned char)responseMsg[i] << ",";
                }
                std::string log = ss.str();
                LOG_INFO << "发送：" << log;
            }
            {
                MutexLockGuard lock(externalInfoMutex_);
                externalInfo_.actualRFID = message;
                externalCondition_.notifyAll();
            }
            break;
        }
        //外部接收到单次动作完成06
        case SINGLE_ARCHIVE_FINISH: {
            assert(message.size() == 8);
            {
                MutexLockGuard lock(externalInfoMutex_);
                externalInfo_.singleArchiveFinishedReceived = true;
                externalCondition_.notifyAll();
            }
            break;
        }
        //外部发送档案柜就绪状态07
        case CAB_READY: {
            assert(message.size() == 8);
            u_char responseData[8];
            for(int i = 0; i < sizeof(responseData); i++) {
                responseData[i] = data[i];
            }
            responseData[6] = 0x01;
            responseData[7] = 0xAA;
            StringPiece responseMsg(responseData);
            write(responseMsg);
            {
                std::stringstream ss;
                for(int i = 0; i < responseMsg.size(); i++) {
                    ss << std::hex << (unsigned int)(unsigned char)responseMsg[i] << ",";
                }
                std::string log = ss.str();
                LOG_INFO << "发送：" << log;
            }
            {
                MutexLockGuard lock(externalInfoMutex_);
                externalInfo_.readyCab.insert(data[7]);
                externalCondition_.notifyAll();
            }
            break;
        }
        //结束任务08
        case ALL_FINISH_TASK: {
            assert(message.size() == 8);
            u_char responseData[8];
            for(int i = 0; i < sizeof(responseData); i++) {
                responseData[i] = data[i];
            }
            responseData[6] = 0x01;
            responseData[7] = 0xAA;
            StringPiece responseMsg(responseData);
            write(responseMsg);
            {
                std::stringstream ss;
                for(int i = 0; i < responseMsg.size(); i++) {
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