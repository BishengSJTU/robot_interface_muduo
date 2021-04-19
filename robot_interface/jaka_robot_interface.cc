#include "jaka_robot_interface.h"

RobotInterface::RobotInterface(std::string path):
codec_(std::bind(&RobotInterface::onCompleteMessage, this, muduo::_1, muduo::_2, muduo::_3)),
taskCondition_(taskMessageMutex_),
externalCondition_(externalInfoMutex_),
connectionCondition_(connectionMutex_),
config_(path + "/FixedConfig.YML"),
jakaPickAndPlace_(path, ROBOT_INLINE),
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
                for (int i = 0; i < storagePositionTotalNum; i++) {
                    if (data[7 + 18 * i] == 0xA1)
                        response[7 + 18 * i] = 0x00;
                }
            } else if (taskType == CHARGE_TASK || taskType == DEPOSIT_PREPARE_TASK) {
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
                LOG_INFO << "忽略任务：机器人尚未处于可工作状态" << log;
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
                        LOG_INFO << "忽略任务：机器人正在充电中，再次收到充电任务" << log;
                    }
                    continue;
                }
            }
        }
        // 电量小于最低电量，只接收充电任务，忽略其它任务
        if (power <= powerLowerLimit) {
            if (taskType == CHARGE_TASK) {
                if(data[7] == 0xA0) {
                    response[7] = 0x00;
                }
            } else if (taskType == DEPOSIT_TASK || taskType == WITHDRAW_TASK) {
                for (int i = 0; i < storagePositionTotalNum; i++) {
                    if (data[7 + 18 * i] == 0xA1)
                        response[7 + 18 * i] = 0x00;
                }
            } else if (taskType == DEPOSIT_PREPARE_TASK) {
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
                LOG_INFO << "忽略任务：机器人处于低电量状态，只能执行充电任务" << log;
            }
            continue;
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
                    LOG_INFO << "完成任务：存档准备" << log;
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
                    LOG_INFO << "更新状态：机器人正在存档中" << log;
                }

                //窗口和柜子每本取放结果是否成功
                std::vector<bool> windowResults;
                std::vector<bool> cabResults;
                for(int archive = 0; archive < storagePositionTotalNum; archive++) {
                    windowResults.push_back(true);
                    cabResults.push_back(true);
                }
                //窗口、暂存架和柜子机械是否故障
                bool windowMechanicalError = false;
                bool storageMechanicalError = false;
                bool cabMechanicalError = false;
                //是否连接
                bool isConnecting;

                //机械臂伸展
                if(ROBOT_INLINE) {
                    jakaPickAndPlace_.JAKAStretch();
                }
                for(int archive = 0; archive < storagePositionTotalNum; archive++) {
                    // 判断是否断连
                    {
                        MutexLockGuard lock(connectionMutex_);
                        if (connection_) {
                            isConnecting = true;
                        } else {
                            isConnecting = false;
                        }
                    }
                    // 机械故障或者掉线，结束任务
                    if(windowMechanicalError || storageMechanicalError || !isConnecting) break;
                    if(data[7 + 18 * archive] == 0xA1) {
                        if(ROBOT_INLINE) {
                            windowResults[archive] = jakaPickAndPlace_.JAKAPickWindow(archive + 1,
                                                                                      windowMechanicalError);
                        }
                        if(windowMechanicalError) {
                            {
                                MutexLockGuard lock(robotInfoMutex_);
                                robotInfo_.currentState = MECHANICAL_ERROR;
                            }
                            LOG_ERROR << "窗口取档机械故障";
                            for(int i = archive; i < storagePositionTotalNum; i++) {
                                if(data[7 + 18 * i] == 0xA1) {
                                    response[7 + 18 * i] = 0x00;
                                }
                            }
                            StringPiece responseMsg(response);
                            write(responseMsg);
                            {
                                std::stringstream ss;
                                for(int i = 0; i < responseMsg.size(); i++) {
                                    ss << std::hex << (unsigned int)(unsigned char)responseMsg[i] << ",";
                                }
                                std::string log = ss.str();
                                LOG_INFO << "结束任务：发生机械故障" << log;
                            }
                            continue;
                        }
                        if(windowResults[archive]) {
                            if(ROBOT_INLINE) {
                                jakaPickAndPlace_.JAKAPlaceStorage(archive + 1, storageMechanicalError);
                            }
                            if(storageMechanicalError) {
                                {
                                    MutexLockGuard lock(robotInfoMutex_);
                                    robotInfo_.currentState = MECHANICAL_ERROR;
                                }
                                LOG_ERROR << "暂存架放档案机械故障";
                                for(int i = archive; i < storagePositionTotalNum; i++) {
                                    if(data[7 + 18 * i] == 0xA1) {
                                        response[7 + 18 * i] = 0x00;
                                    }
                                }
                                StringPiece responseMsg(response);
                                write(responseMsg);
                                {
                                    std::stringstream ss;
                                    for(int i = 0; i < responseMsg.size(); i++) {
                                        ss << std::hex << (unsigned int)(unsigned char)responseMsg[i] << ",";
                                    }
                                    std::string log = ss.str();
                                    LOG_INFO << "结束任务：发生机械故障" << log;
                                }
                                continue;
                            }
                        }
                        LOG_INFO  << "从窗口到暂存架中成功一次";
                    }
                }
                if(windowMechanicalError || storageMechanicalError || !isConnecting)
                    break;

                if(ROBOT_INLINE) {
                    jakaPickAndPlace_.JAKAContraction();
                    agv_.ActionFinishedAgvGo();
                }

                for(int archive = 0; archive < storagePositionTotalNum; archive++) {
                    // 判断是否断连
                    {
                        MutexLockGuard lock(connectionMutex_);
                        if (connection_) {
                            isConnecting = true;
                        } else {
                            isConnecting = false;
                        }
                    }
                    // 机械故障或者掉线，结束任务
                    if(cabMechanicalError || storageMechanicalError || !isConnecting) break;
                    if(data[7 + 18 * archive] == 0xA1) {
                        if(windowResults[archive]) {
                            //柜号
                            int cabId = data[20 + 18 * archive];
                            //位置号
                            int positionId = data[21 + 18 * archive];
                            //档案号
                            int archiveId = data[22 + 18 * archive] * 256 + data[23 + 18 * archive];
                            //二维码号
                            int markerId = data[24 + 18 * archive];

                            if(ROBOT_INLINE) {
                                int agvMissionId;
                                agv_.AgvGo(cabId, positionId, agvMissionId);
                                agv_.AgvReached();
                                jakaPickAndPlace_.JAKAStretch();
                            }

                            //　等待档案柜就绪
                            {
                                MutexLockGuard lock(externalInfoMutex_);
                                while (externalInfo_.readyCab.find(cabId) == externalInfo_.readyCab.end()) {
                                    externalCondition_.wait();
                                }
                                externalInfo_.readyCab.erase(cabId);
                            }
                            if(ROBOT_INLINE) {
                                jakaPickAndPlace_.JAKAPickStorage(archive + 1, storageMechanicalError);
                            }
                            if(storageMechanicalError) {
                                {
                                    MutexLockGuard lock(robotInfoMutex_);
                                    robotInfo_.currentState = MECHANICAL_ERROR;
                                }
                                LOG_ERROR << "暂存架取档案机械故障";
                                for(int i = archive; i < storagePositionTotalNum; i++) {
                                    if(data[7 + 18 * i] == 0xA1) {
                                        response[7 + 18 * i] = 0x00;
                                    }
                                }
                                StringPiece responseMsg(response);
                                write(responseMsg);
                                {
                                    std::stringstream ss;
                                    for(int i = 0; i < responseMsg.size(); i++) {
                                        ss << std::hex << (unsigned int)(unsigned char)responseMsg[i] << ",";
                                    }
                                    std::string log = ss.str();
                                    LOG_INFO << "结束任务：发生机械故障" << log;
                                }
                                continue;
                            }
                            if(ROBOT_INLINE) {
                                cabResults[archive] = jakaPickAndPlace_.JAKAPlaceCab(cabId, positionId,
                                                                                     cabMechanicalError);
                            }
                            if(cabMechanicalError) {
                                {
                                    MutexLockGuard lock(robotInfoMutex_);
                                    robotInfo_.currentState = MECHANICAL_ERROR;
                                }
                                LOG_ERROR << "档案柜放档案机械故障";
                                for(int i = archive; i < storagePositionTotalNum; i++) {
                                    if(data[7 + 18 * i] == 0xA1) {
                                        response[7 + 18 * i] = 0x00;
                                    }
                                }
                                StringPiece responseMsg(response);
                                write(responseMsg);
                                {
                                    std::stringstream ss;
                                    for(int i = 0; i < responseMsg.size(); i++) {
                                        ss << std::hex << (unsigned int)(unsigned char)responseMsg[i] << ",";
                                    }
                                    std::string log = ss.str();
                                    LOG_INFO << "结束任务：发生机械故障" << log;
                                }
                                continue;
                            }
                        }
                        u_char singleFinishData[22] = {
                                0xA1, 0x00, 0x00, 0x00, 0x08, 0x00, 0x0F, 0x01,
                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                0x00, 0x00};
                        for(int i = 0; i < 2; i++) {
                            singleFinishData[i + 2] = data[i + 2];
                        }
                        for(int i = 0; i < 12; i++) {
                            singleFinishData[i + 8] = data[8 + 18 * archive + i];
                        }
                        for(int i = 0; i < 2; i++) {
                            singleFinishData[i + 20] = data[22 + 18 * archive + i];
                        }
                        string singleFinishMsg(sizeof(singleFinishData), '0');
                        for(int i = 0; i < singleFinishMsg.size(); i++)
                            singleFinishMsg[i] = singleFinishData[i];
                        write(singleFinishMsg);
                        {
                            std::stringstream ss;
                            for(int i = 0; i < singleFinishMsg.size(); i++) {
                                ss << std::hex << (unsigned int)(unsigned char)singleFinishMsg[i] << ",";
                            }
                            std::string log = ss.str();
                            LOG_INFO << "发送状态：单本完成" << log;
                        }
                        //　等待单本完成被接收
                        {
                            MutexLockGuard lock(externalInfoMutex_);
                            while (externalInfo_.singleArchiveFinishedReceived == false) {
                                externalCondition_.wait();
                            }
                            externalInfo_.singleArchiveFinishedReceived = false;
                        }
                    }
                    LOG_INFO  << "从暂存架到档案柜中成功一次";
                }

                if(cabMechanicalError || storageMechanicalError || !isConnecting)
                    break;

                for(int archive = 0; archive < storagePositionTotalNum; archive++) {
                    if(data[7 + 18 * archive] == 0xA1) {
                        if(cabResults[archive] && windowResults[archive]) {
                            response[7 + 18 * archive] = 0x01;
                        }
                    } else {
                        response[7 + 18 * archive] = 0x00;
                    }
                }
                StringPiece responseMsg(response);
                write(responseMsg);
                {
                    std::stringstream ss;
                    for(int i = 0; i < responseMsg.size(); i++) {
                        ss << std::hex << (unsigned int)(unsigned char)responseMsg[i] << ",";
                    }
                    std::string log = ss.str();
                    LOG_INFO << "完成任务：存档" << log;
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
                    LOG_INFO << "更新状态：机器人空闲" << log;
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
                    LOG_INFO << "更新状态：机器人正在取档中" << log;
                }

                //窗口和柜子每本取放结果是否成功
                std::vector<bool> windowResults;
                std::vector<bool> cabResults;
                std::vector<bool> checkResults;
                for(int i = 0; i < storagePositionTotalNum; i++) {
                    windowResults.push_back(true);
                    cabResults.push_back(true);
                    checkResults.push_back(true);
                }
                //窗口、暂存架和柜子机械是否故障
                bool windowMechanicalError = false;
                bool storageMechanicalError = false;
                bool cabMechanicalError = false;
                //是否连接
                bool isConnecting;

                for(int archive = 0; archive < storagePositionTotalNum; archive++) {
                    // 判断是否断连
                    {
                        MutexLockGuard lock(connectionMutex_);
                        if (connection_) {
                            isConnecting = true;
                        } else {
                            isConnecting = false;
                        }
                    }
                    // 机械故障或者掉线，结束任务
                    if(cabMechanicalError || storageMechanicalError || !isConnecting) break;
                    if(data[7 + 18 * archive] == 0xA1) {
                        int cabId = data[20 + 18 * archive];
                        int positionId = data[21 + 18 * archive];
                        int archiveId = data[22 + 18 * archive] * 256 +
                                data[23 + 18 * archive];
                        int markerId = data[24 + 18 * archive];

                        if(ROBOT_INLINE) {
                            int agvMissionId;
                            agv_.AgvGo(cabId, positionId, agvMissionId);
                            agv_.AgvReached();
                        }

                        {
                            MutexLockGuard lock(externalInfoMutex_);
                            while (externalInfo_.readyCab.find(cabId) == externalInfo_.readyCab.end()) {
                                externalCondition_.wait();
                            }
                            externalInfo_.readyCab.erase(cabId);
                        }

                        if(ROBOT_INLINE) {
                            jakaPickAndPlace_.JAKAStretch();
                            cabResults[archive] = jakaPickAndPlace_.JAKAPickCab(cabId, positionId, cabMechanicalError);
                        }

                        if(cabMechanicalError) {
                            {
                                MutexLockGuard lock(robotInfoMutex_);
                                robotInfo_.currentState = MECHANICAL_ERROR;
                            }
                            LOG_ERROR << "档案柜取档案机械故障";
                            for(int i = archive; i < storagePositionTotalNum; i++) {
                                if(data[7 + 18 * i] == 0xA1) {
                                    response[7 + 18 * i] = 0x00;
                                }
                            }
                            StringPiece responseMsg(response);
                            write(responseMsg);
                            {
                                std::stringstream ss;
                                for(int i = 0; i < responseMsg.size(); i++) {
                                    ss << std::hex << (unsigned int)(unsigned char)responseMsg[i] << ",";
                                }
                                std::string log = ss.str();
                                LOG_INFO << "结束任务：发生机械故障" << log;
                            }
                            continue;
                        }

                        u_char withdrawCheckData[21] = {
                                0xA1, 0x00, 0x00, 0x01, 0x06, 0x00, 0x0E,
                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                0x00, 0x00};
                        for(int i = 0; i < 2; i++) {
                            withdrawCheckData[2 + i] = data[2 + i];
                        }
                        for(int i = 0; i < 12; i++) {
                            withdrawCheckData[7 + i] = data[8 + 18 * archive];
                        }

                        //如果成功取出，进行校验
                        if(cabResults[archive]) {
                            for (int i = 0; i < 2; i++) {
                                withdrawCheckData[19 + i] = data[22 + 18 * archive];
                            }
                            string withdrawCheckMsg(sizeof(withdrawCheckData), '0');
                            for(int i = 0; i < withdrawCheckMsg.size(); i++)
                                withdrawCheckMsg[i] = withdrawCheckData[i];
                            write(withdrawCheckMsg);
                            {
                                std::stringstream ss;
                                for(int i = 0; i < withdrawCheckMsg.size(); i++) {
                                    ss << std::hex << (unsigned int)(unsigned char)withdrawCheckMsg[i] << ",";
                                }
                                std::string log = ss.str();
                                LOG_INFO << "发送任务：取档校验" << log;
                            }

                            {
                                MutexLockGuard lock(externalInfoMutex_);
                                while (externalInfo_.withdrawCheckReceived == false) {
                                    externalCondition_.wait();
                                }
                                externalInfo_.withdrawCheckReceived = false;
                            }

                            {
                                MutexLockGuard lock(externalInfoMutex_);
                                while (externalInfo_.withdrawCheckResultGiven == false) {
                                    externalCondition_.wait();
                                }
                                externalInfo_.withdrawCheckResultGiven = false;
                                checkResults[archive] = externalInfo_.withdrawCheckResult;
                            }
                            if(ROBOT_INLINE) {
                                jakaPickAndPlace_.JAKAPlaceStorage(archive + 1, storageMechanicalError);
                            }
                            if(storageMechanicalError) {
                                {
                                    MutexLockGuard lock(robotInfoMutex_);
                                    robotInfo_.currentState = MECHANICAL_ERROR;
                                }
                                LOG_ERROR << "暂存架放档案机械故障";
                                for(int i = archive; i < storagePositionTotalNum; i++) {
                                    if(data[7 + 18 * i] == 0xA1) {
                                        response[7 + 18 * i] = 0x00;
                                    }
                                }
                                StringPiece responseMsg(response);
                                write(responseMsg);
                                {
                                    std::stringstream ss;
                                    for(int i = 0; i < responseMsg.size(); i++) {
                                        ss << std::hex << (unsigned int)(unsigned char)responseMsg[i] << ",";
                                    }
                                    std::string log = ss.str();
                                    LOG_INFO << "结束任务：发生机械故障" << log;
                                }
                                continue;
                            }
                        } else {
                            for (int i = 0; i < 2; i++) {
                                withdrawCheckData[19 + i] = 0x00;
                            }
                            string withdrawCheckMsg(sizeof(withdrawCheckData), '0');
                            for(int i = 0; i < withdrawCheckMsg.size(); i++)
                                withdrawCheckMsg[i] = withdrawCheckData[i];
                            write(withdrawCheckMsg);
                            {
                                std::stringstream ss;
                                for(int i = 0; i < withdrawCheckMsg.size(); i++) {
                                    ss << std::hex << (unsigned int)(unsigned char)withdrawCheckMsg[i] << ",";
                                }
                                std::string log = ss.str();
                                LOG_INFO << "取消任务：取档校验，未取档成功" << log;
                            }

                            {
                                MutexLockGuard lock(externalInfoMutex_);
                                while (externalInfo_.withdrawCheckReceived == false) {
                                    externalCondition_.wait();
                                }
                                externalInfo_.withdrawCheckReceived = false;
                            }

                            {
                                MutexLockGuard lock(externalInfoMutex_);
                                while (externalInfo_.withdrawCheckResultGiven == false) {
                                    externalCondition_.wait();
                                }
                                externalInfo_.withdrawCheckResultGiven = false;
                                checkResults[archive] = externalInfo_.withdrawCheckResult;
                            }
                        }

                        u_char singleFinishData[22] = {
                                0xA1, 0x00, 0x00, 0x00, 0x08, 0x00, 0x0F, 0x00,
                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                0x00, 0x00};
                        for(int i = 0; i < 2; i++) {
                            singleFinishData[i + 2] = data[i + 2];
                        }
                        for(int i = 0; i < 12; i++) {
                            singleFinishData[i + 8] = data[8 + 18 * archive + i];
                        }
                        for(int i = 0; i < 2; i++) {
                            singleFinishData[i + 20] = data[22 + 18 * archive + i];
                        }

                        string singleFinishMsg(sizeof(singleFinishData), '0');
                        for(int i = 0; i < singleFinishMsg.size(); i++)
                            singleFinishMsg[i] = singleFinishData[i];
                        write(singleFinishMsg);
                        {
                            std::stringstream ss;
                            for(int i = 0; i < singleFinishMsg.size(); i++) {
                                ss << std::hex << (unsigned int)(unsigned char)singleFinishMsg[i] << ",";
                            }
                            std::string log = ss.str();
                            LOG_INFO << "发送状态：单本完成" << log;
                        }
                        //　等待单本完成被接收
                        {
                            MutexLockGuard lock(externalInfoMutex_);
                            while (externalInfo_.singleArchiveFinishedReceived == false) {
                                externalCondition_.wait();
                            }
                            externalInfo_.singleArchiveFinishedReceived = false;
                        }

                        if(ROBOT_INLINE) {
                            jakaPickAndPlace_.JAKAContraction();
                            agv_.ActionFinishedAgvGo();
                        }
                    }
                }
                if(cabMechanicalError || storageMechanicalError || !isConnecting)
                    break;

                //只要有一本取档成功，就去窗口
                for(int archive = 0; archive < storagePositionTotalNum; archive++) {
                    if(cabResults[archive]) {
                        if(ROBOT_INLINE) {
                            int agvMissionId;
                            agv_.AgvGo(0, 1, agvMissionId);
                            agv_.AgvReached();
                            jakaPickAndPlace_.JAKAStretch();
                            break;
                        }
                    }
                }

                for(int archive = 0; archive < storagePositionTotalNum; archive++) {
                    // 判断是否断连
                    {
                        MutexLockGuard lock(connectionMutex_);
                        if (connection_) {
                            isConnecting = true;
                        } else {
                            isConnecting = false;
                        }
                    }
                    // 机械故障或者掉线，结束任务
                    if(windowMechanicalError || storageMechanicalError || !isConnecting) break;

                    if(data[7 + 18 * archive] == 0xA1) {
                        if(cabResults[archive]) {
                            if(ROBOT_INLINE) {
                                jakaPickAndPlace_.JAKAPickStorage(archive + 1, storageMechanicalError);
                            }
                            if(storageMechanicalError) {
                                {
                                    MutexLockGuard lock(robotInfoMutex_);
                                    robotInfo_.currentState = MECHANICAL_ERROR;
                                }
                                LOG_ERROR << "暂存架取档案机械故障";
                                for(int i = archive; i < storagePositionTotalNum; i++) {
                                    if(data[7 + 18 * i] == 0xA1) {
                                        response[7 + 18 * i] = 0x00;
                                    }
                                }
                                StringPiece responseMsg(response);
                                write(responseMsg);
                                {
                                    std::stringstream ss;
                                    for(int i = 0; i < responseMsg.size(); i++) {
                                        ss << std::hex << (unsigned int)(unsigned char)responseMsg[i] << ",";
                                    }
                                    std::string log = ss.str();
                                    LOG_INFO << "结束任务：发生机械故障" << log;
                                }
                                continue;
                            }

                            if(ROBOT_INLINE) {
                                windowResults[archive] = jakaPickAndPlace_.JAKAPlaceWindow(archive + 1,
                                                                                           windowMechanicalError);
                            }
                            if(windowMechanicalError) {
                                {
                                    MutexLockGuard lock(robotInfoMutex_);
                                    robotInfo_.currentState = MECHANICAL_ERROR;
                                }
                                LOG_ERROR << "窗口放档案机械故障";
                                for(int i = archive; i < storagePositionTotalNum; i++) {
                                    if(data[7 + 18 * i] == 0xA1) {
                                        response[7 + 18 * i] = 0x00;
                                    }
                                }
                                StringPiece responseMsg(response);
                                write(responseMsg);
                                {
                                    std::stringstream ss;
                                    for(int i = 0; i < responseMsg.size(); i++) {
                                        ss << std::hex << (unsigned int)(unsigned char)responseMsg[i] << ",";
                                    }
                                    std::string log = ss.str();
                                    LOG_INFO << "结束任务：发生机械故障" << log;
                                }
                                continue;
                            }
                        }
                    }
                }

                if(windowMechanicalError || storageMechanicalError || !isConnecting)
                    break;

                for(int archive = 0; archive < storagePositionTotalNum; archive++) {
                    if(data[7 + 18 * archive] == 0xA1) {
                        if(cabResults[archive] && windowResults[archive] && checkResults[archive]) {
                            response[7 + 18 * archive] = 0x01;
                        }
                    } else {
                        response[7 + 18 * archive] = 0x00;
                    }
                }
                StringPiece responseMsg(response);
                write(responseMsg);
                {
                    std::stringstream ss;
                    for(int i = 0; i < responseMsg.size(); i++) {
                        ss << std::hex << (unsigned int)(unsigned char)responseMsg[i] << ",";
                    }
                    std::string log = ss.str();
                    LOG_INFO << "完成任务：取档" << log;
                }

                for(int archive = 0; archive < storagePositionTotalNum; archive++) {
                    if(cabResults[archive]) {
                        if (ROBOT_INLINE) {
                            jakaPickAndPlace_.JAKAContraction();
                            agv_.ActionFinishedAgvGo();
                        }
                        break;
                    }
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
                    LOG_INFO << "更新状态：机器人空闲" << log;
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
                        LOG_INFO << "更新状态：机器人准备充电中" << log;
                    }
                    ///　跑到充电桩前，完成充电动作
                    if(ROBOT_INLINE) {
                        int agvMissionId;
                        agv_.AgvGo(0, 2, agvMissionId);
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
                        LOG_INFO << "完成任务：充电" << log;
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
                        LOG_INFO << "更新状态：机器人正在充电中" << log;
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
                        LOG_INFO << "更新状态：机器人充电完成" << log;
                    }
                    ///　跑到休息区
                    if(ROBOT_INLINE) {
                        int agvMissionId;
                        agv_.AgvGo(0, 3, agvMissionId);
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
                        LOG_INFO << "完成任务：取消充电" << log;
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
                        LOG_INFO << "更新状态：机器人空闲中" << log;
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

void RobotInterface::onCompleteMessage(const muduo::net::TcpConnectionPtr&,
                            const muduo::string& message,
                            Timestamp) {
    {
        std::stringstream ss;
        for(int i = 0; i < message.size(); i++) {
            ss << std::hex << (unsigned int)(unsigned char)message[i] << ",";
        }
        std::string log = ss.str();
        LOG_INFO << "接收消息：" << log;
    }

    const char *data = message.c_str();
    const u_char taskType = data[4];
    switch(taskType) {
        //心跳33
        case HEAR_BEAT: {
            assert(message.size() == 8);
            u_char responseData[8];
            for(int i = 0; i < sizeof(responseData); i++) {
                responseData[i] = data[i];
            }

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
                LOG_INFO << "发送消息：心跳" << log;
            }
        }
        //存档任务01，需执行！
        case DEPOSIT_TASK: {
            assert(message.size() == 97);
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
                LOG_INFO << "发送消息：收到存档任务" << log;
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
                LOG_INFO << "发送消息：收到取档任务" << log;
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
                LOG_INFO << "发送消息：收到充电任务" << log;
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
                LOG_INFO << "发送消息：收到存档校验任务" << log;
            }
            //直接回复存档校验结果
            u_char responseData2[82];
            for(int i = 0; i < sizeof(responseData2); i++) {
                responseData2[i] = data[i];
            }
            for(int archive = 0; archive < storagePositionTotalNum; archive++) {
                if(data[7 + 15 * archive] == 0xA1) {
                    responseData2[7 + 15 * archive] = 0x01;
                }
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
                LOG_INFO << "完成任务：存档校验" << log;
            }
            break;
        }
        //外部发送查询指令05
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
                LOG_INFO << "发送消息：收到查询指令" << log;
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
                LOG_INFO << "发送消息：机器人状态" << log;
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
            if(data[19] == 0x01) {
                MutexLockGuard lock(externalInfoMutex_);
                externalInfo_.withdrawCheckResultGiven = true;
                externalInfo_.withdrawCheckResult = true;
                externalCondition_.notifyAll();
            } else if(data[19] == 0x00) {
                MutexLockGuard lock(externalInfoMutex_);
                externalInfo_.withdrawCheckResultGiven = true;
                externalInfo_.withdrawCheckResult = false;
                externalCondition_.notifyAll();
            }
        }
        // 存档准备任务07，需执行！
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
                LOG_INFO << "发送消息：收到存档准备任务" << log;
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
            for (int i = 0; i < sizeof(responseData1); i++) {
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
                LOG_INFO << "发送消息：收到取消操作任务" << log;
            }

            u_char responseData2[8];
            for (int i = 0; i < sizeof(responseData2); i++) {
                responseData2[i] = data[i];
            }
            responseData2[6] = 0x01;
            responseData2[7] = 0x01;

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
                LOG_INFO << "完成任务：取消操作" << log;
            }
        }
        // 档案柜就绪状态0A
        case CAB_STATE: {
            assert(message.size() == 9);
            u_char responseData[8];
            for (int i = 0; i < sizeof(responseData); i++) {
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
                LOG_INFO << "发送消息：收到档案柜状态" << log;
            }
            if(data[8] == 0x01) {
                MutexLockGuard lock(externalInfoMutex_);
                externalInfo_.readyCab.insert(data[7]);
                externalCondition_.notifyAll();
            } else {
                MutexLockGuard lock(externalInfoMutex_);
                externalInfo_.readyCab.erase(data[7]);
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

void RobotInterface::onConnection(const TcpConnectionPtr& conn) {
    u_char connectData[8] = {0xA1, 0x00, 0x00, 0x00, 0x11, 0x00, 0x01 , 0xAA};
    string connectMsg(sizeof(connectData), '0');
    for(int i = 0; i < connectMsg.size(); i++)
        connectMsg[i] = connectData[i];

    LOG_INFO << conn->localAddress().toIpPort() << " -> "
             << conn->peerAddress().toIpPort() << " is "
             << (conn->connected() ? "UP" : "DOWN");

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
            }
            std::string log = ss.str();
            LOG_INFO << "发送状态：连接" << log;
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
        externalInfo_.withdrawCheckResultGiven = false;
        externalInfo_.withdrawCheckReceived = false;
        externalInfo_.withdrawCheckResult = false;
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
        }
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

            if(ROBOT_INLINE) {
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
                if(ROBOT_INLINE) {
                    jakaPickAndPlace_.JAKAPLCState(); //PLC保活
                }
            }
        }
    }
}