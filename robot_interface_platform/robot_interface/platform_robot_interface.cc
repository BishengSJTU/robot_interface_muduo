#include "platform_robot_interface.h"

RobotInterface::RobotInterface(std::string path):
codec_(std::bind(&RobotInterface::onCompleteMessage, this, muduo::_1, muduo::_2, muduo::_3)),
taskCondition_(taskMessageMutex_),
externalCondition_(externalInfoMutex_),
connectionCondition_(connectionMutex_),
config_(path + "/FixedConfig.YML"),
agv_(path, ROBOT_INLINE)
{
    taskServerIP_ = config_.get<std::string>("MIDDLE_LAYER_IP");
    taskServerPort_ = config_.get<int>("MIDDLE_LAYER_PORT");
    initialize();
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

        const char *data_ptr = task.data();
        std::vector<u_char> data;
        for(int i = 0; i < task.size(); i++)
            data.push_back((unsigned int)(unsigned char)data_ptr[i]);

        u_char taskType = data[4];
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
            } else if (taskType == DEPOSIT_PREPARE_TASK || taskType == CHARGE_TASK || taskType == ALL_FINISH_TASK) {
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
                LOG_INFO << "--->对方：忽略任务，机器人尚未处于可工作状态" << log;
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
                        LOG_INFO << "--->对方：忽略任务，机器人正在充电中，再次收到充电任务" << log;
                    }
                    continue;
                }
            }
        }
        // 电量小于最低电量，只接收充电任务
        if (power <= powerLowerLimit) {
            if (taskType == CHARGE_TASK) {
                if(data[7] == 0xA0) {
                    response[7] = 0x00;
                    StringPiece responseMsg(response);
                    write(responseMsg);
                    {
                        std::stringstream ss;
                        for(int i = 0; i < responseMsg.size(); i++) {
                            ss << std::hex << (unsigned int)(unsigned char)responseMsg[i] << ",";
                        }
                        std::string log = ss.str();
                        LOG_INFO << "--->对方：忽略停止充电任务，机器人处于低电量状态，只能执行充电任务" << log;
                    }
                    continue;
                }
            } else if (taskType == DEPOSIT_TASK || taskType == WITHDRAW_TASK) {
                for (int i = 0; i < turntablePositionTotalNum; i++) {
                    if (data[7 + 2 * i] == 0xA1)
                        response[7 + 2 * i] = 0x00;
                }
                StringPiece responseMsg(response);
                write(responseMsg);
                {
                    std::stringstream ss;
                    for(int i = 0; i < responseMsg.size(); i++) {
                        ss << std::hex << (unsigned int)(unsigned char)responseMsg[i] << ",";
                    }
                    std::string log = ss.str();
                    LOG_INFO << "--->对方：忽略存取档任务，机器人处于低电量状态，只能执行充电任务" << log;
                }
                continue;
            } else if (taskType == ALL_FINISH_TASK || taskType == DEPOSIT_PREPARE_TASK) {
                response[7] = 0x00;
                StringPiece responseMsg(response);
                write(responseMsg);
                {
                    std::stringstream ss;
                    for(int i = 0; i < responseMsg.size(); i++) {
                        ss << std::hex << (unsigned int)(unsigned char)responseMsg[i] << ",";
                    }
                    std::string log = ss.str();
                    LOG_INFO << "--->对方：忽略存档准备任务/任务结束，机器人处于低电量状态，只能执行充电任务" << log;
                }
                continue;
            }
        }
        // 可执行任务状态
        switch (taskType) {
            //存档准备
            case DEPOSIT_PREPARE_TASK: {
                ///跑到窗口
                if(ROBOT_INLINE) {
                    int agvMissionId;
                    agv_.AgvGo(0, 1, agvMissionId);
                    agv_.AgvReached();
                    agv_.ActionFinishedAgvGo();
                }

                response[7] = 0x01;
                StringPiece responseMsg(response);
                write(responseMsg);
                {
                    std::stringstream ss;
                    for(int i = 0; i < responseMsg.size(); i++) {
                        ss << std::hex << (unsigned int)(unsigned char)responseMsg[i] << ",";
                    }
                    std::string log = ss.str();
                    LOG_INFO << "--->对方：完成任务，存档准备" << log;
                }
                break;
            }
            //存档
            case DEPOSIT_TASK: {
                {
                    MutexLockGuard lock(robotInfoMutex_);
                    robotInfo_.currentState = IS_DEPOSITING;
                    stateMsgArray[7] = robotInfo_.currentPower;
                    stateMsgArray[8] = IS_DEPOSITING;
                }
                string stateMsg(sizeof(stateMsgArray), '0');
                for(int i = 0; i < stateMsg.size(); i++)
                    stateMsg[i] = stateMsgArray[i];
                write(stateMsg);
                {
                    std::stringstream ss;
                    for(int i = 0; i < stateMsg.size(); i++) {
                        ss << std::hex << (unsigned int)(unsigned char)stateMsg[i] << ",";
                    }
                    std::string log = ss.str();
                    LOG_INFO << "--->对方：更新状态，机器人正在存档中" << log;
                }

                // 跑到窗口,机械臂伸展
                if(ROBOT_INLINE) {
                    int agvMissionId;
                    agv_.AgvGo(0, 1, agvMissionId);
                    agv_.AgvReached();
                }

                //将所有在窗口的档案进行排序
                int windowPosition = 0;
                std::map<int, std::vector<int> > cabIDWindowPositionsMap;
                for(int i = 0; i < turntablePositionTotalNum; i++) {
                    if(data[7 + 2 * i] == 0xA1) {
                        windowPosition++;
                        int cabID = data[8 + 2 * i];
                        if (cabIDWindowPositionsMap.find(cabID) == cabIDWindowPositionsMap.end()) {
                            std::vector<int> windowPositions;
                            windowPositions.push_back(windowPosition);
                            cabIDWindowPositionsMap.insert(std::make_pair(cabID, windowPositions));
                        } else {
                            cabIDWindowPositionsMap.find(cabID)->second.push_back(windowPosition);
                        }
                    }
                }

                int turntablePosition = turntablePositionTotalNum;
                std::map<int, std::vector<int> > cabIDTurntablePositionsMap;
                std::map<int, int> turntablePositionCommandPositionMap;
                for(auto cabIDWindowPositionsMapRIt = cabIDWindowPositionsMap.rbegin();
                cabIDWindowPositionsMapRIt != cabIDWindowPositionsMap.rend();
                cabIDWindowPositionsMapRIt++) {
                    int cabID = cabIDWindowPositionsMapRIt->first;
                    std::vector<int> windowPositions = cabIDWindowPositionsMapRIt->second;
                    for(int i = windowPositions.size() - 1; i >= 0 ; i--) {
                        ;///将windowPositions[i]处档案放置到turntablePositionTotalNum处
//                        PlatformPickWindow(windowPositions[i], turntablePosition);
                        if (cabIDTurntablePositionsMap.find(cabID) == cabIDTurntablePositionsMap.end()) {
                            std::vector<int> turntablePositions;
                            turntablePositions.push_back(turntablePosition);
                            cabIDTurntablePositionsMap.insert(std::make_pair(cabID, turntablePositions));
                        } else {
                            cabIDTurntablePositionsMap.find(cabID)->second.push_back(turntablePosition);
                        }
                        turntablePositionCommandPositionMap.insert(std::make_pair(turntablePosition, windowPositions[i] - 1));
                        turntablePosition--;
                    }
                }

                for (auto cabIDTurntablePositions : cabIDTurntablePositionsMap) {
                    int cabID = cabIDTurntablePositions.first;
                    std::vector<int> turntablePositions = cabIDTurntablePositions.second;
                    ; /// 跑到对应柜子前
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

                    // 额外等待档案柜就绪次数(圆盘)
//                    int extraOperationTimes =
//                            (operationArchiveNum % cabPositionTotalNum == 0)
//                            ? (operationArchiveNum / cabPositionTotalNum - 1)
//                            : (operationArchiveNum / cabPositionTotalNum);

                    // 额外等待档案柜就绪次数(直斗)
                    int extraOperationTimes = 0;


                    for (int turntableNum = 1; turntableNum <= operationArchiveNum; turntableNum++) {
//                        PlatformPlaceCab(turntablePositions[turntableNum - 1], turntableNum)
                        ; /// 将档案从转盘中放入柜中
                        // 额外等待次数>0并且本轮操作结束，等待档案柜就绪(圆盘)
//                        if (extraOperationTimes > 0 && (turntableNum % cabPositionTotalNum == 0)) {
//                            singleFinishMsgArray[7] = cabID;
//                            string singleFinishMsg(sizeof(singleFinishMsgArray), '0');
//                            for(int i = 0; i < singleFinishMsg.size(); i++)
//                                singleFinishMsg[i] = singleFinishMsgArray[i];
//                            write(singleFinishMsg);
//                            {
//                                std::stringstream ss;
//                                for(int i = 0; i < singleFinishMsg.size(); i++) {
//                                    ss << std::hex << (unsigned int)(unsigned char)singleFinishMsg[i] << ",";
//                                }
//                                std::string log = ss.str();
//                                LOG_INFO << "--->对方：单轮动作完成" << log;
//                            }
//                            {
//                                MutexLockGuard lock(externalInfoMutex_);
//                                while (externalInfo_.singleArchiveFinishedReceived == false) {
//                                    externalCondition_.wait();
//                                }
//                                externalInfo_.singleArchiveFinishedReceived = false;
//                            }
//                            {
//                                MutexLockGuard lock(externalInfoMutex_);
//                                while (externalInfo_.readyCab.find(cabID) == externalInfo_.readyCab.end()) {
//                                    externalCondition_.wait();
//                                }
//                                externalInfo_.readyCab.erase(cabID);
//                                extraOperationTimes--;
//                            }
//                        }
                    }

                    singleFinishMsgArray[7] = cabID;

                    string singleFinishMsg(sizeof(singleFinishMsgArray), '0');
                    for(int i = 0; i < singleFinishMsg.size(); i++)
                        singleFinishMsg[i] = singleFinishMsgArray[i];
                    write(singleFinishMsg);
                    {
                        std::stringstream ss;
                        for(int i = 0; i < singleFinishMsg.size(); i++) {
                            ss << std::hex << (unsigned int)(unsigned char)singleFinishMsg[i] << ",";
                        }
                        std::string log = ss.str();
                        LOG_INFO << "--->对方：单台动作完成" << log;
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
                    LOG_INFO << "--->对方：完成任务，存档" << log;
                }

                if(ROBOT_INLINE) {
                    int agvMissionId;
                    agv_.AgvGo(0, 0, agvMissionId);
                    agv_.AgvReached();
                    agv_.ActionFinishedAgvGo();
                }

                {
                    MutexLockGuard lock(robotInfoMutex_);
                    robotInfo_.currentState = IS_FREE;
                    stateMsgArray[7] = robotInfo_.currentPower;
                    stateMsgArray[8] = IS_FREE;
                }
                for(int i = 0; i < stateMsg.size(); i++)
                    stateMsg[i] = stateMsgArray[i];
                write(stateMsg);
                {
                    std::stringstream ss;
                    for(int i = 0; i < stateMsg.size(); i++) {
                        ss << std::hex << (unsigned int)(unsigned char)stateMsg[i] << ",";
                    }
                    std::string log = ss.str();
                    LOG_INFO << "--->对方：更新状态，机器人空闲" << log;
                }

                break;
            }

            // 取档
            case WITHDRAW_TASK: {
                {
                    MutexLockGuard lock(robotInfoMutex_);
                    robotInfo_.currentState = IS_WITHDRAWING;
                    stateMsgArray[7] = robotInfo_.currentPower;
                    stateMsgArray[8] = IS_WITHDRAWING;
                }
                string stateMsg(sizeof(stateMsgArray), '0');
                for(int i = 0; i < stateMsg.size(); i++)
                    stateMsg[i] = stateMsgArray[i];
                write(stateMsg);
                {
                    std::stringstream ss;
                    for(int i = 0; i < stateMsg.size(); i++) {
                        ss << std::hex << (unsigned int)(unsigned char)stateMsg[i] << ",";
                    }
                    std::string log = ss.str();
                    LOG_INFO << "--->对方：更新状态，机器人正在取档中" << log;
                }

                //根据任务清单将所有档案归类
                int commandPosition = 0;
                std::map<int, std::vector<int> > cabIDCommandPositionsMap;
                std::map<int, int > turntablePositionCommandPositionMap;
                for (int i = 0; i < turntablePositionTotalNum; i++) {
                    if (data[7 + 2 * i] == 0xA1) {
                        int cabID = data[8 + 2 * i];
                        if (cabIDCommandPositionsMap.find(cabID) == cabIDCommandPositionsMap.end()) {
                            std::vector<int> commandPositions;
                            commandPositions.push_back(commandPosition);
                            cabIDCommandPositionsMap.insert(std::make_pair(cabID, commandPositions));
                        } else {
                            cabIDCommandPositionsMap.find(cabID)->second.push_back(commandPosition);
                        }
                        commandPosition++;
                    }
                }

                int turntablePosition = turntablePositionTotalNum;
                for (auto cabIDCommandPositions : cabIDCommandPositionsMap) {
                    int cabID = cabIDCommandPositions.first;
                    std::vector<int> commandPositions = cabIDCommandPositions.second;
                    ; /// 跑到对应柜子前
                    //　等待档案柜就绪
                    {
                        MutexLockGuard lock(externalInfoMutex_);
                        while (externalInfo_.readyCab.find(cabID) == externalInfo_.readyCab.end()) {
                            externalCondition_.wait();
                        }
                        externalInfo_.readyCab.erase(cabID);
                    }

                    //此柜需要操作的总的档案数量
                    int operationArchiveNum = commandPositions.size();

                    // 额外等待档案柜就绪次数（圆盘）
//                    int extraOperationTimes =
//                            (operationArchiveNum % cabPositionTotalNum == 0)
//                            ? (operationArchiveNum / cabPositionTotalNum - 1)
//                            : (operationArchiveNum / cabPositionTotalNum);
                    int extraOperationTimes = 0;


                    for (int turntableNum = 1; turntableNum <= operationArchiveNum; turntableNum++) {
                        ; ///将档案从柜中放入转盘中
//                        PlatformPickCab(turntableNum, turntablePosition);
                        turntablePositionCommandPositionMap.insert(std::make_pair(turntablePosition, commandPositions[turntableNum - 1]));
                        // 额外等待次数>0并且本轮操作结束，等待档案柜就绪(圆盘)
//                        if (extraOperationTimes > 0 && (turntableNum % cabPositionTotalNum == 0)) {
//                            singleFinishMsgArray[7] = cabID;
//                            string singleFinishMsg(sizeof(singleFinishMsgArray), '0');
//                            for(int i = 0; i < singleFinishMsg.size(); i++)
//                                singleFinishMsg[i] = singleFinishMsgArray[i];
//                            write(singleFinishMsg);
//                            {
//                                std::stringstream ss;
//                                for(int i = 0; i < singleFinishMsg.size(); i++) {
//                                    ss << std::hex << (unsigned int)(unsigned char)singleFinishMsg[i] << ",";
//                                }
//                                std::string log = ss.str();
//                                LOG_INFO << "发送：" << log;
//                            }
//                            {
//                                MutexLockGuard lock(externalInfoMutex_);
//                                while (externalInfo_.singleArchiveFinishedReceived == false) {
//                                    externalCondition_.wait();
//                                }
//                                externalInfo_.singleArchiveFinishedReceived = false;
//                            }
//                            {
//                                MutexLockGuard lock(externalInfoMutex_);
//                                while (externalInfo_.readyCab.find(cabID) == externalInfo_.readyCab.end()) {
//                                    externalCondition_.wait();
//                                }
//                                externalInfo_.readyCab.erase(cabID);
//                                extraOperationTimes--;
//                            }
//                        }
                        turntablePosition--;
                    }

                    singleFinishMsgArray[7] = cabID;
                    string singleFinishMsg(sizeof(singleFinishMsgArray), '0');
                    for(int i = 0; i < singleFinishMsg.size(); i++)
                        singleFinishMsg[i] = singleFinishMsgArray[i];
                    write(singleFinishMsg);
                    {
                        std::stringstream ss;
                        for(int i = 0; i < singleFinishMsg.size(); i++) {
                            ss << std::hex << (unsigned int)(unsigned char)singleFinishMsg[i] << ",";
                        }
                        std::string log = ss.str();
                        LOG_INFO << "--->对方：单台动作完成" << log;
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
                int windowPosition = 1;
                for(int i = turntablePosition + 1; i <= turntablePositionTotalNum; i++) {
//                    PlatformPlaceWindow(i, windowPosition);
                    windowPosition++;
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
                    LOG_INFO << "--->对方：完成任务，取档" << log;
                }

                if(ROBOT_INLINE) {
                    int agvMissionId;
                    agv_.AgvGo(0, 0, agvMissionId);
                    agv_.AgvReached();
                    agv_.ActionFinishedAgvGo();
                }

                {
                    MutexLockGuard lock(robotInfoMutex_);
                    robotInfo_.currentState = IS_FREE;
                    stateMsgArray[7] = robotInfo_.currentPower;
                    stateMsgArray[8] = IS_FREE;
                }
                for(int i = 0; i < stateMsg.size(); i++)
                    stateMsg[i] = stateMsgArray[i];
                write(stateMsg);
                {
                    std::stringstream ss;
                    for(int i = 0; i < stateMsg.size(); i++) {
                        ss << std::hex << (unsigned int)(unsigned char)stateMsg[i] << ",";
                    }
                    std::string log = ss.str();
                    LOG_INFO << "--->对方：更新状态，机器人空闲" << log;
                }
                break;
            }

            case CHARGE_TASK: {
                if(data[7] == 0xA1) {
                    {
                        MutexLockGuard lock(robotInfoMutex_);
                        robotInfo_.currentState = CHARGE_PREPARE;
                        stateMsgArray[7] = robotInfo_.currentPower;
                        stateMsgArray[8] = CHARGE_PREPARE;
                    }
                    string stateMsg(sizeof(stateMsgArray), '0');
                    for(int i = 0; i < stateMsg.size(); i++)
                        stateMsg[i] = stateMsgArray[i];
                    write(stateMsg);
                    {
                        std::stringstream ss;
                        for(int i = 0; i < stateMsg.size(); i++) {
                            ss << std::hex << (unsigned int)(unsigned char)stateMsg[i] << ",";
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
                        LOG_INFO << "--->对方：完成任务，充电" << log;
                    }
                    {
                        MutexLockGuard lock(robotInfoMutex_);
                        robotInfo_.currentState = IS_CHARGING;
                        stateMsgArray[7] = robotInfo_.currentPower;
                        stateMsgArray[8] = IS_CHARGING;
                    }

                    for(int i = 0; i < stateMsg.size(); i++)
                        stateMsg[i] = stateMsgArray[i];
                    write(stateMsg);
                    {
                        std::stringstream ss;
                        for(int i = 0; i < stateMsg.size(); i++) {
                            ss << std::hex << (unsigned int)(unsigned char)stateMsg[i] << ",";
                        }
                        std::string log = ss.str();
                        LOG_INFO << "--->对方：更新状态，机器人正在充电中" << log;
                    }
                }
                else if(data[7] == 0xA0) {
                    {
                        MutexLockGuard lock(robotInfoMutex_);
                        robotInfo_.currentState = CHARGE_FINISHED;
                        stateMsgArray[7] = robotInfo_.currentPower;
                        stateMsgArray[8] = CHARGE_FINISHED;
                    }
                    string stateMsg(sizeof(stateMsgArray), '0');
                    for(int i = 0; i < stateMsg.size(); i++)
                        stateMsg[i] = stateMsgArray[i];
                    write(stateMsg);
                    {
                        std::stringstream ss;
                        for(int i = 0; i < stateMsg.size(); i++) {
                            ss << std::hex << (unsigned int)(unsigned char)stateMsg[i] << ",";
                        }
                        std::string log = ss.str();
                        LOG_INFO << "--->对方：更新状态，机器人充电完成" << log;
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

                    for(int i = 0; i < stateMsg.size(); i++)
                        stateMsg[i] = stateMsgArray[i];
                    write(stateMsg);
                    {
                        std::stringstream ss;
                        for(int i = 0; i < stateMsg.size(); i++) {
                            ss << std::hex << (unsigned int)(unsigned char)stateMsg[i] << ",";
                        }
                        std::string log = ss.str();
                        LOG_INFO << "--->对方：完成任务，取消充电" << log;
                    }
                    {
                        MutexLockGuard lock(robotInfoMutex_);
                        robotInfo_.currentState = IS_FREE;
                        stateMsgArray[7] = robotInfo_.currentPower;
                        stateMsgArray[8] = IS_FREE;
                    }

                    for(int i = 0; i < stateMsg.size(); i++)
                        stateMsg[i] = stateMsgArray[i];
                    write(stateMsg);
                    {
                        std::stringstream ss;
                        for(int i = 0; i < stateMsg.size(); i++) {
                            ss << std::hex << (unsigned int)(unsigned char)stateMsg[i] << ",";
                        }
                        std::string log = ss.str();
                        LOG_INFO << "--->对方：更新状态，机器人空闲中" << log;
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

                string stateMsg(sizeof(stateMsgArray), '0');
                for(int i = 0; i < stateMsg.size(); i++)
                    stateMsg[i] = stateMsgArray[i];
                write(stateMsg);
                {
                    std::stringstream ss;
                    for(int i = 0; i < stateMsg.size(); i++) {
                        ss << std::hex << (unsigned int)(unsigned char)stateMsg[i] << ",";
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

void RobotInterface::onCompleteMessage(const muduo::net::TcpConnectionPtr&,
                            const muduo::string& message,
                            Timestamp) {
    {
        std::stringstream ss;
        for(int i = 0; i < message.size(); i++) {
            ss << std::hex << (unsigned int)(unsigned char)message[i] << ",";
        }
        std::string log = ss.str();
        LOG_INFO << "对方--->：" << log;
    }

    const char *data_ptr = message.c_str();
    std::vector<u_char > data;
    for(int i = 0; i < message.size(); i++)
        data.push_back((unsigned int)(unsigned char)data_ptr[i]);

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
            string responseMsg(sizeof(responseData), '0');
            for(int i = 0; i < responseMsg.size(); i++)
                responseMsg[i] = responseData[i];
            write(responseMsg);
            {
                std::stringstream ss;
                for(int i = 0; i < responseMsg.size(); i++) {
                    ss << std::hex << (unsigned int)(unsigned char)responseMsg[i] << ",";
                }
                std::string log = ss.str();
                LOG_INFO << "--->对方：收到存档任务" << log;
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

            string responseMsg(sizeof(responseData), '0');
            for(int i = 0; i < responseMsg.size(); i++)
                responseMsg[i] = responseData[i];
            write(responseMsg);
            {
                std::stringstream ss;
                for(int i = 0; i < responseMsg.size(); i++) {
                    ss << std::hex << (unsigned int)(unsigned char)responseMsg[i] << ",";
                }
                std::string log = ss.str();
                LOG_INFO << "--->对方：收到取档任务" << log;
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
            string responseMsg(sizeof(responseData), '0');
            for(int i = 0; i < responseMsg.size(); i++)
                responseMsg[i] = responseData[i];
            write(responseMsg);
            {
                std::stringstream ss;
                for(int i = 0; i < responseMsg.size(); i++) {
                    ss << std::hex << (unsigned int)(unsigned char)responseMsg[i] << ",";
                }
                std::string log = ss.str();
                LOG_INFO << "--->对方：收到充电任务" << log;
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
            string responseMsg1(sizeof(responseData1), '0');
            for(int i = 0; i < responseMsg1.size(); i++)
                responseMsg1[i] = responseData1[i];
            write(responseMsg1);
            {
                std::stringstream ss;
                for(int i = 0; i < responseMsg1.size(); i++) {
                    ss << std::hex << (unsigned int)(unsigned char)responseMsg1[i] << ",";
                }
                std::string log = ss.str();
                LOG_INFO << "--->对方：收到查询指令" << log;
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
            string responseMsg2(sizeof(responseData2), '0');
            for(int i = 0; i < responseMsg2.size(); i++)
                responseMsg2[i] = responseData2[i];
            write(responseMsg2);
            {
                std::stringstream ss;
                for(int i = 0; i < responseMsg2.size(); i++) {
                    ss << std::hex << (unsigned int)(unsigned char)responseMsg2[i] << ",";
                }
                std::string log = ss.str();
                LOG_INFO << "--->对方：机器人状态" << log;
            }
            break;
        }
        //存档准备任务05
        case DEPOSIT_PREPARE_TASK: {
            assert(message.size() == 8);
            u_char responseData[8];
            for(int i = 0; i < sizeof(responseData); i++) {
                responseData[i] = data[i];
            }
            responseData[6] = 0x01;
            responseData[7] = 0xAA;
            string responseMsg(sizeof(responseData), '0');
            for(int i = 0; i < responseMsg.size(); i++)
                responseMsg[i] = responseData[i];
            write(responseMsg);
            {
                std::stringstream ss;
                for(int i = 0; i < responseMsg.size(); i++) {
                    ss << std::hex << (unsigned int)(unsigned char)responseMsg[i] << ",";
                }
                std::string log = ss.str();
                LOG_INFO << "--->对方：收到存档准备任务" << log;
            }
            {
                MutexLockGuard lock(taskMessageMutex_);
                taskMessage_ = message;
                taskCondition_.notifyAll();
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
            string responseMsg(sizeof(responseData), '0');
            for(int i = 0; i < responseMsg.size(); i++)
                responseMsg[i] = responseData[i];
            write(responseMsg);
            {
                std::stringstream ss;
                for(int i = 0; i < responseMsg.size(); i++) {
                    ss << std::hex << (unsigned int)(unsigned char)responseMsg[i] << ",";
                }
                std::string log = ss.str();
                LOG_INFO << "--->对方：收到档案柜就绪状态" << log;
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
            string responseMsg(sizeof(responseData), '0');
            for(int i = 0; i < responseMsg.size(); i++)
                responseMsg[i] = responseData[i];
            write(responseMsg);
            {
                std::stringstream ss;
                for(int i = 0; i < responseMsg.size(); i++) {
                    ss << std::hex << (unsigned int)(unsigned char)responseMsg[i] << ",";
                }
                std::string log = ss.str();
                LOG_INFO << "--->对方：所有任务完成" << log;
            }
            {
                MutexLockGuard lock(taskMessageMutex_);
                taskMessage_ = message;
                taskCondition_.notifyAll();
            }
            break;
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

void RobotInterface::onConnection(const TcpConnectionPtr& conn) {
    u_char connectArray[8] = {0xA1, 0x00, 0x00, 0x00, 0x11, 0x00, 0x01, 0xAA};
    string connectMsg(sizeof(connectArray), '0');
    for(int i = 0; i < connectMsg.size(); i++)
        connectMsg[i] = connectArray[i];

    LOG_INFO << conn->localAddress().toIpPort() << " -> "
             << conn->peerAddress().toIpPort() << "已"
             << (conn->connected() ? "连接" : "断连");

    {
        MutexLockGuard lock(connectionMutex_);
        if (conn->connected()) {
            connection_ = conn;
            connectionCondition_.notifyAll();
        } else {
            connection_.reset();
            initialize();
        }
    }

    if(conn->connected()) {
        write(connectMsg);
        {
            std::stringstream ss;
            for(int i = 0; i < connectMsg.size(); i++) {
                ss << std::hex << (unsigned int)(unsigned char)connectMsg[i] << ",";
                std::cout << std::hex << (unsigned int)(unsigned char)connectMsg[i] << ",";
            }
            std::cout << std::endl;
            std::string log = ss.str();
            LOG_INFO << "--->对方：连接" << log;
        }
    }
}

void RobotInterface::initialize()
{
    {
        MutexLockGuard lock(taskMessageMutex_);
        taskMessage_ = "";
    }
    {
        MutexLockGuard lock(externalInfoMutex_);
        externalInfo_.readyCab = {};
        externalInfo_.singleArchiveFinishedReceived = false;
    }
    int agvState = AGV_READY;
    int agvPower = powerLowerLimit;

    if(ROBOT_INLINE) {
        agv_.GetAgvPowerAndState(agvPower, agvState);
    }
    if( agvState == AGV_FAILURE || agvState == AGV_MT) {
        MutexLockGuard lock(robotInfoMutex_);
        robotInfo_.currentPower = agvPower;
        robotInfo_.currentState = IS_INITIALIZING;
    } else if(agvState == AGV_BUSY) {
        MutexLockGuard lock(robotInfoMutex_);
        robotInfo_.currentPower = agvPower;
        robotInfo_.currentState = NEED_INITIALIZING;
    } else if(agvState == AGV_CHARGING) {
        MutexLockGuard lock(robotInfoMutex_);
        robotInfo_.currentPower = agvPower;
        robotInfo_.currentState = IS_CHARGING;
    } else if(agvState == AGV_READY) {
        MutexLockGuard lock(robotInfoMutex_);
        robotInfo_.currentPower = agvPower;
        robotInfo_.currentState = IS_FREE;
    }
}

void RobotInterface::inquireRobotStateThread() {

    while (1) {
        if(!ROBOT_INLINE) {
            sleep(5);
            continue;
        } else {
            {
                MutexLockGuard lock(taskMessageMutex_);
                while (connection_ == nullptr) {
                    connectionCondition_.wait();
                }
            }
            sleep(5);
            if (ROBOT_INLINE) {
                int agvState;
                int agvPower;
                int currentAgvState = AGV_READY;
                int currentAgvPower = powerLowerLimit;
                {
                    MutexLockGuard lock(robotInfoMutex_);
                    agvState = robotInfo_.currentState;
                    agvPower = robotInfo_.currentPower;
                }

                if (ROBOT_INLINE) {
                    agv_.GetAgvPowerAndState(currentAgvPower, currentAgvState);
                }
                // 没有机械故障
                if (agvState != MECHANICAL_ERROR) {
                    if (currentAgvState == AGV_READY) {
                        if (agvState == NEED_INITIALIZING
                            || agvState == INITIALIZE_FINISH
                            || agvState == IS_CHARGING
                            || agvState == IS_INITIALIZING) {
                            agvState = IS_FREE;
                        }
                    } else if (currentAgvState == AGV_CHARGING) {
                        agvState = IS_CHARGING;
                    } else if (currentAgvState == AGV_FAILURE || currentAgvState == AGV_MT) {
                        agvState = NEED_INITIALIZING;
                    }
                } else {
                    agvState = MECHANICAL_ERROR;
                }
                {
                    MutexLockGuard lock(robotInfoMutex_);
                    robotInfo_.currentState = agvState;
                    robotInfo_.currentPower = agvPower;
                }
                if ((agvState != IS_WITHDRAWING) && (agvState != IS_DEPOSITING)) {
                    if (ROBOT_INLINE) {
                        ; ///PLC保活
                    }
                }
            }
        }
    }
}