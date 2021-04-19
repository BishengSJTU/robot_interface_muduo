#include "jaka_plc.h"

JAKAPLC::JAKAPLC(std::string address, int port, bool isInline)
{
    if(!isInline)
        return;
    InitializePLC(address, port);
    LOG_INFO << "MyPLC构造";
}

void JAKAPLC::InitializePLC(std::string address, int port)
{
    address_ = address;
    port_ = port;
    std::stringstream io;
    std::string log;
    LOG_INFO << "连接到IP地址：" << address << "端口号:" << port;
    const char *address_ptr = address.c_str();
    //创建socketrqt
    if ((clientSocket_ = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        LOG_ERROR << "PLC创建客户端端口失败" << strerror(errno) << "（错误号：" << errno << "）";
        exit(PLCEXITNUM);
    }

    memset(&clientAddr_, 0, sizeof(clientAddr_));
    //指定IP地址版本为IPV4
    clientAddr_.sin_family = AF_INET;
    //设置端口
    clientAddr_.sin_port = htons(port);
    //IP地址转换函数
    if (inet_pton(AF_INET, address_ptr, &clientAddr_.sin_addr) <= 0) {
        LOG_ERROR << "PLC地址转换函数出错 " << address_ptr;
        exit(PLCEXITNUM);
    }

    while(1) {
        if (connect(clientSocket_, (struct sockaddr *) &clientAddr_, sizeof(clientAddr_)) < 0) {
            LOG_ERROR << "PLC客户端等待连接中...";
            sleep(1);
        }
        else {
            LOG_INFO << "PLC客户端连接成功！";
            break;
        }
    }
}

JAKAPLC::~JAKAPLC(){
    LOG_INFO << "MyPLC析构";
    close(clientSocket_);
}

// PLC动作
bool JAKAPLC::ControlPLC(int command) {
    unsigned char plcAction[RW_NUM] =
            {0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x01, 0x05, 0x08, 0x00, 0xFF, 0x00};
    unsigned char actualPLCAction[RW_NUM];
    unsigned char actualPLCState[STATE_NUM];
    unsigned char getPLCState[12] =
            {0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x01, 0x03, 0x10, 0x00, 0x00, 0x02};
    unsigned char stateFront = 0x02;
    unsigned char stateBack = 0x01;
    unsigned char stateMidFront = 0x05;
    unsigned char stateError = 0x04;

    //针对指令类型发送命令
    switch (command) {
        case MoveF:
            plcAction[9] = 0x01;
            break;
        case MoveB:
            plcAction[9] = 0x00;
            break;
        case MoveMF:
            plcAction[9] = 0x03;
        case AirpumpO:
            plcAction[9] = 0x02;
            break;
        case AirpumpC:
            plcAction[9] = 0x02;
            plcAction[10] = 0x00;
            break;
        default:
            return false;
    }

    // 发送动作指令
    bool quit;
    while(1) {
        quit = true;
        while(send(clientSocket_, plcAction, sizeof(plcAction), 0) <= 0) {
            close(clientSocket_);
            InitializePLC(address_, port_);
            LOG_ERROR << "PLC客户端发送数据错误，尝试重新建立连接";
        }
        usleep(PLCTCPWAITTIME);
        int recvLen = recv(clientSocket_, actualPLCAction, sizeof(actualPLCAction), 0);
        usleep(PLCTCPWAITTIME);

        std::stringstream io;
        std::string log;
        io << "Step1: 发送运动指令，从PLC服务器接收：";
        for (std::size_t i = 0; i < recvLen; i++)
            io << std::hex << (unsigned int) (unsigned char) actualPLCAction[i] << ",";
        io >> log;
        LOG_INFO << log;

        if(recvLen != sizeof(plcAction)) {
            quit = false;
            continue;
        }
        for(int i = 0; i < recvLen; i++) {
            if(actualPLCAction[i] != plcAction[i])
                quit = false;
        }
        if(quit == true)
            break;
        sleep(1);
    }
    if(command == AirpumpO || command == AirpumpC)
        return true;
    //　电机前移
    if(command == MoveF) {
        while(1) {
            quit = true;
            if (send(clientSocket_, getPLCState, sizeof(getPLCState), 0) < 0) {
                LOG_ERROR << "PLC客户端发送数据错误，异常退出！";
                exit(PLCEXITNUM);
            }
            usleep(PLCTCPWAITTIME);
            int recvLen = recv(clientSocket_, actualPLCState, sizeof(actualPLCState), 0);
            usleep(PLCTCPWAITTIME);

            std::stringstream io;
            std::string log;
            io << "Step2: 发送读取状态指令　从PLC服务器接收:";
            for (std::size_t i = 0; i < recvLen; i++)
                io << std::hex << (unsigned int) (unsigned char) actualPLCState[i] << ",";
            io >> log;
            LOG_INFO << log;

            if(recvLen != STATE_NUM) {
                quit = false;
                continue;
            }

            if(actualPLCState[10] != stateFront)
                quit = false;

            bool quit2 = true;
            if(actualPLCState[10] != stateError)
                quit2 = false;

            // 电机前移到位
            if(quit == true) {
                return true;
            }
            // 电机前移卡住
            if(quit2 == true) {
                return false;
            }
            sleep(1);
        }
    }

    //　电机前移
    if(command == MoveMF) {
        while(1) {
            quit = true;
            if (send(clientSocket_, getPLCState, sizeof(getPLCState), 0) < 0) {
                LOG_ERROR << "PLC客户端发送数据错误，异常退出！";
                exit(PLCEXITNUM);
            }
            usleep(PLCTCPWAITTIME);
            int recvLen = recv(clientSocket_, actualPLCState, sizeof(actualPLCState), 0);
            usleep(PLCTCPWAITTIME);

            std::stringstream io;
            std::string log;
            io << "Step2: 发送读取状态指令　从PLC服务器接收:";
            for (std::size_t i = 0; i < recvLen; i++)
                io << std::hex << (unsigned int) (unsigned char) actualPLCState[i] << ",";
            io >> log;
            LOG_INFO << log;

            if(recvLen != STATE_NUM) {
                quit = false;
                continue;
            }

            if(actualPLCState[10] != stateMidFront)
                quit = false;

            bool quit2 = true;
            if(actualPLCState[10] != stateError)
                quit2 = false;

            // 电机前移到位
            if(quit == true) {
                return true;
            }
            // 电机前移卡住
            if(quit2 == true) {
                return false;
            }
            sleep(1);
        }
    }

    // 电机缩回
    if(command == MoveB) {
        while(1) {
            quit = true;
            if (send(clientSocket_, getPLCState, sizeof(getPLCState), 0) < 0) {
                LOG_ERROR << "PLC客户端发送数据错误，异常退出！";
                exit(PLCEXITNUM);
            }
            usleep(PLCTCPWAITTIME);
            int recvLen = recv(clientSocket_, actualPLCState, sizeof(actualPLCState), 0);
            usleep(PLCTCPWAITTIME);

            std::stringstream io;
            std::string log;
            io << "Step2: 发送读取状态指令　从PLC服务器接收:";
            for (std::size_t i = 0; i < recvLen; i++)
                io << std::hex << (unsigned int) (unsigned char) actualPLCState[i] << ",";
            io >> log;
            LOG_INFO << log;

            if(recvLen != STATE_NUM) {
                quit = false;
                continue;
            }

            if(actualPLCState[10] != stateBack)
                quit = false;

            bool quit2 = true;
            if(actualPLCState[10] != stateError)
                quit2 = false;

            // 电机后移到位
            if(quit == true) {
                return true;
            }
            // 电机后移卡住
            if(quit2 == true) {
                return false;
            }
            sleep(1);
        }
    }
}

// 判断档案是否取到末端中
bool JAKAPLC::ArchiveInHand() {
    unsigned char getArchiveState[RW_NUM] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x01, 0x03, 0x10, 0x00, 0x00, 0x02};
    unsigned char actualArchiveState[STATE_NUM];
    if (send(clientSocket_, getArchiveState, sizeof(getArchiveState), 0) < 0) {
        LOG_ERROR << "PLC客户端发送数据错误，异常退出！";
        exit(PLCEXITNUM);
    }
    usleep(PLCTCPWAITTIME);
    int recvLen = recv(clientSocket_, actualArchiveState, sizeof(actualArchiveState), 0);
    usleep(PLCTCPWAITTIME);

    std::stringstream io;
    std::string log;
    io << "档案是否在手中:";
    for (std::size_t i = 0; i < recvLen; i++)
        io << std::hex << (unsigned int) (unsigned char) actualArchiveState[i] << ",";
    io >> log;
    LOG_INFO << log;

    if(recvLen != STATE_NUM) {
        return false;
    }
    if(actualArchiveState[12] == 0x01)
        return true;
    else
        return false;
}

void JAKAPLC::InquireState() {
    unsigned char getPLCState[RW_NUM] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x01, 0x03, 0x10, 0x00, 0x00, 0x02};
    unsigned char actualPLCState[STATE_NUM];
    if (send(clientSocket_, getPLCState, sizeof(getPLCState), 0) < 0) {
        LOG_ERROR << "PLC客户端发送数据错误，异常退出！";
        exit(PLCEXITNUM);
    }
    usleep(PLCTCPWAITTIME);
    int recvLen = recv(clientSocket_, actualPLCState, sizeof(actualPLCState), 0);
    usleep(PLCTCPWAITTIME);
}