#include "my_plc.h"

MyPLC::MyPLC()
{
    LOG_INFO << "MyPLC::MyPLC构造";
}

void MyPLC::InitializePLC(std::string address, int port)
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
        exit(EXITNUM);
    }

    memset(&clientAddr_, 0, sizeof(clientAddr_));
    //指定IP地址版本为IPV4
    clientAddr_.sin_family = AF_INET;
    //设置端口
    clientAddr_.sin_port = htons(port);
    //IP地址转换函数
    if (inet_pton(AF_INET, address_ptr, &clientAddr_.sin_addr) <= 0) {
        LOG_ERROR << "PLC地址转换函数出错 " << address_ptr;
        exit(EXITNUM);
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

MyPLC::~MyPLC(){
    LOG_INFO << "MyPLC析构";
    close(clientSocket_);
}

// PLC动作：1前移 2缩回 7气泵开 8气泵关
bool MyPLC::ControlPLC(int command) {
    unsigned char plcAction[RW_NUM] =
            {0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x01, 0x05, 0x08, 0x00, 0xFF, 0x00};
    unsigned char actualPLCAction[RW_NUM];
    unsigned char actualPLCState[STATE_NUM];
    unsigned char getPLCState[12] =
            {0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x01, 0x03, 0x10, 0x00, 0x00, 0x02};
    unsigned char stateFront = 0x02;
    unsigned char stateBack = 0x01;
    unsigned char stateError = 0x04;

    //针对指令类型发送命令
    switch (command) {
        case 1:
            plcAction[9] = 0x01;
            break;
        case 2:
            plcAction[9] = 0x00;
            break;
        case 7:
            plcAction[9] = 0x02;
            break;
        case 8:
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
        int recvLen = recv(clientSocket_, actualPLCAction, sizeof(actualPLCAction), 0);

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
    if(command == 7 || command == 8)
        return true;
    //　电机前移
    if(command == 1) {
        while(1) {
            quit = true;
            if (send(clientSocket_, getPLCState, sizeof(getPLCState), 0) < 0) {
                LOG_ERROR << "PLC客户端发送数据错误，异常退出！";
                exit(EXITNUM);
            }

            int recvLen = recv(clientSocket_, actualPLCState, sizeof(actualPLCState), 0);

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

    // 电机缩回
    if(command == 2) {
        while(1) {
            quit = true;
            if (send(clientSocket_, getPLCState, sizeof(getPLCState), 0) < 0) {
                LOG_ERROR << "PLC客户端发送数据错误，异常退出！";
                exit(EXITNUM);
            }
            int recvLen = recv(clientSocket_, actualPLCState, sizeof(actualPLCState), 0);

            std::stringstream io;
            std::string log;
            io << "2.从PLC服务器接收:";
            for (std::size_t i = 0; i < recvLen; i++)
                io << std::hex << (unsigned int) (unsigned char) actualPLCState[i] << ",";
            io << "等待动作完成...";
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
bool MyPLC::ArchiveInHand() {
    unsigned char getArchiveState[RW_NUM] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x01, 0x03, 0x10, 0x00, 0x00, 0x02};
    unsigned char actualArchiveState[STATE_NUM];
    if (send(clientSocket_, getArchiveState, sizeof(getArchiveState), 0) < 0) {
        LOG_ERROR << "PLC客户端发送数据错误，异常退出！";
        exit(EXITNUM);
    }
    int recvLen = recv(clientSocket_, actualArchiveState, sizeof(actualArchiveState), 0);

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

void MyPLC::InquireState() {
    unsigned char getPLCState[RW_NUM] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x01, 0x03, 0x10, 0x00, 0x00, 0x02};
    unsigned char actualPLCState[STATE_NUM];
    if (send(clientSocket_, getPLCState, sizeof(getPLCState), 0) < 0) {
        LOG_ERROR << "PLC客户端发送数据错误，异常退出！";
        exit(EXITNUM);
    }
    int recvLen = recv(clientSocket_, actualPLCState, sizeof(actualPLCState), 0);
}

void MyPLC::MovePlatform(int x_delta, int y_delta) {
    int x_current, y_current, reset_state, archive_state, sensor_state;
    std::cout << "x_delta:" << x_delta << ", y_delta:" << y_delta << std::endl;
    GetPlatformState(x_current, y_current, reset_state, archive_state, sensor_state);

    int x_move = x_delta  * 107;
    int y_move = y_delta  * 3200;

    int x_target = x_current + x_move;
    int y_target = y_current + y_move;

    std::cout << "x_target:" << x_target << ", y_target:" << y_target << std::endl;

    int x_move_abs = (x_move < 0) ? (-x_move) : x_move;
    int y_move_abs = (y_move < 0) ? (-y_move) : y_move;

    unsigned char x_H4 = x_move_abs >> 24;
    unsigned char x_H3 = x_move_abs << 8 >> 24;
    unsigned char x_H2 = x_move_abs << 16 >> 24;
    unsigned char x_H1 = x_move_abs << 24 >> 24;
    std::cout <<  std::hex << (unsigned int)(unsigned char)x_H4 << "," << (unsigned int)(unsigned char)x_H3 << "," << (unsigned int)(unsigned char)x_H2 << ","<< (unsigned int)(unsigned char)x_H1 << ","<< std::endl;


    unsigned char y_H4 = y_move_abs >> 24;
    unsigned char y_H3 = y_move_abs << 8 >> 24;
    unsigned char y_H2 = y_move_abs << 16 >> 24;
    unsigned char y_H1 = y_move_abs << 24 >> 24;
    std::cout <<  std::hex << (unsigned int)(unsigned char)y_H4 << "," << (unsigned int)(unsigned char)y_H3 << "," << (unsigned int)(unsigned char)y_H2 << ","<< (unsigned int)(unsigned char)y_H1 << ","<< std::endl;

    unsigned char set_x[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x0B, 0x01, 0x10, 0xA0, 0x98, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00};
    unsigned char set_x_feedback[12];
    set_x[15] = x_H4;
    set_x[16] = x_H3;
    set_x[13] = x_H2;
    set_x[14] = x_H1;

    send(clientSocket_, set_x, sizeof(set_x), 0);
    std::cout << "send x_abs:";
    for(int i = 0; i < sizeof(set_x); i++)
        std::cout <<  std::hex << (unsigned int)(unsigned char)set_x[i] << ",";
    std::cout << std::endl;

    int rec_len = recv(clientSocket_, set_x_feedback, sizeof(set_x_feedback), 0);
    assert(rec_len == sizeof(set_x_feedback));
    usleep(100000);

    unsigned char set_y[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x0B, 0x01, 0x10, 0xA0, 0x94, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00};
    unsigned char set_y_feedback[12];
    set_y[15] = y_H4;
    set_y[16] = y_H3;
    set_y[13] = y_H2;
    set_y[14] = y_H1;
    send(clientSocket_, set_y, sizeof(set_y), 0);
    std::cout << "send y_abs:";
    for(int i = 0; i < sizeof(set_y); i++)
        std::cout <<  std::hex << (unsigned int)(unsigned char)set_y[i] << ",";
    std::cout << std::endl;

    rec_len = recv(clientSocket_, set_y_feedback, sizeof(set_y_feedback), 0);
    assert(rec_len == sizeof(set_y_feedback));
    usleep(100000);

    unsigned char set_direction[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x01, 0x05, 0x00, 0x00, 0xFF, 0x00};
    unsigned char set_direction_feedback[12];
    set_direction[9] = (x_move > 0) ? 0x0D : 0x0E;
    send(clientSocket_, set_direction, sizeof(set_direction), 0);
    rec_len = recv(clientSocket_, set_direction_feedback, sizeof(set_direction_feedback), 0);
    assert(rec_len == sizeof(set_direction_feedback));
    usleep(100000);
    std::cout << "send set_direction:";
    for(int i = 0; i < sizeof(set_direction); i++)
        std::cout  << std::hex << (unsigned int)(unsigned char)set_direction[i] << ",";
    std::cout << std::endl;

    set_direction[9] = (y_move > 0) ? 0x00 : 0x01;
    send(clientSocket_, set_direction, sizeof(set_direction), 0);
    rec_len = recv(clientSocket_, set_direction_feedback, sizeof(set_direction_feedback), 0);
    assert(rec_len == sizeof(set_direction_feedback));
    usleep(100000);
    std::cout << "send set_direction:";
    for(int i = 0; i < sizeof(set_direction); i++)
        std::cout  << std::hex << (unsigned int)(unsigned char)set_direction[i] << ",";
    std::cout << std::endl;

    int x, y, reset, archive, sensor;
    while(1)
    {
        GetPlatformState(x, y, reset, archive, sensor);
        if(x == x_target && y == y_target)
            break;
    }
    return;
}

void MyPLC::PickArchive(int arm_position)
{
    ManipulateArchive(arm_position, 1);
}
void MyPLC::PlaceArchive(int arm_position)
{
    ManipulateArchive(arm_position, 2);
}

void MyPLC::ManipulateArchive(int arm_position, int mode)
{
    unsigned char set_position[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x01, 0x10, 0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x00};
    set_position[14] = (unsigned char)arm_position;
    unsigned char set_position_feedback[12];
    send(clientSocket_, set_position, sizeof(set_position), 0);
    int rec_len = recv(clientSocket_, set_position_feedback, sizeof(set_position_feedback), 0);
    assert(rec_len == sizeof(set_position_feedback));
    usleep(100000);

    unsigned char set_mode[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x01, 0x05, 0x00, 0x00, 0xFF, 0x00};
    unsigned char set_mode_feedback[12];
    set_mode[9] = (mode == 1) ? 0x28 : 0x32;
    send(clientSocket_, set_mode, sizeof(set_mode), 0);
    rec_len = recv(clientSocket_, set_mode_feedback, sizeof(set_mode_feedback), 0);
    assert(rec_len == sizeof(set_mode_feedback));
    usleep(100000);

    int manipulation_finished = (mode == 1) ? 0x02 : 0x04;

    int x, y, reset, archive, sensor;
    while(1)
    {
        GetPlatformState(x, y, reset, archive, sensor);
        if(archive == manipulation_finished && sensor != 0x01)
            break;
    }
    return;
}

void MyPLC::ResetPlatform()
{
    unsigned char reset[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x01, 0x05, 0x00, 0x3C, 0xFF, 0x00};
    unsigned char reset_feedback[12];
    send(clientSocket_, reset, sizeof(reset), 0);

    std::cout << "send:";
    for(int i = 0; i < sizeof(reset); i++)
        std::cout <<  std::hex << (unsigned int)(unsigned char)reset[i] << ",";
    std::cout << std::endl;

    int rec_len = recv(clientSocket_, reset_feedback, sizeof(reset_feedback), 0);

    std::cout << "receive:";
    for(int i = 0; i < sizeof(reset_feedback); i++)
        std::cout  << std::hex << (unsigned int)(unsigned char)reset_feedback[i] << ",";
    std::cout << std::endl;

    assert(rec_len == sizeof(reset_feedback));
    usleep(100000);

    int x, y, reset_state, archive, sensor;
    while(1) {
        GetPlatformState(x, y, reset_state, archive, sensor);
        if(reset_state == 0x02)
            break;
    }
}

void MyPLC::GetPlatformState(int &x_current, int &y_current, int &reset_state, int &archive_state, int &sensor)
{
    unsigned char buff_get_state[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x01, 0x03, 0x00, 0x0A, 0x00, 0x08};
    unsigned char buff_state[25];
    send(clientSocket_, buff_get_state, sizeof(buff_get_state), 0);
    int rec_len = recv(clientSocket_, buff_state, sizeof(buff_state), 0);

    std::cout << "receive rec_len " << (int)rec_len << ":";
    for(int i = 0; i < rec_len; i++)
        std::cout << std::hex << (unsigned int)(unsigned char)buff_state[i] << ",";
    std::cout << std::endl;


    assert(rec_len == sizeof(buff_state));

    unsigned char yH4 = buff_state[15];
    unsigned char yH3 = buff_state[16];
    unsigned char yH2 = buff_state[13];
    unsigned char yH1 = buff_state[14];
    y_current = (yH4 << 24) + (yH3 << 16) + (yH2 << 8) + yH1;
    unsigned char xH4 = buff_state[19];
    unsigned char xH3 = buff_state[20];
    unsigned char xH2 = buff_state[17];
    unsigned char xH1 = buff_state[18];
    x_current = (xH4 << 24) + (xH3 << 16) + (xH2 << 8) + xH1;
    reset_state = (unsigned int)buff_state[10];
    archive_state = (unsigned int)buff_state[12];
    sensor = (unsigned int)buff_state[24];
    usleep(100000);
}


