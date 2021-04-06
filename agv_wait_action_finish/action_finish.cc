#include "action_finish.h"

ActionFinish::ActionFinish(int port, int queue_len)
{
    queue_len_ = queue_len;
    bzero(&server_addr_, sizeof(server_addr_));
    server_addr_.sin_family = AF_INET;
    server_addr_.sin_addr.s_addr = htons(INADDR_ANY);
    server_addr_.sin_port = htons(port);
    listen_fd_ = socket(PF_INET, SOCK_STREAM, 0);

    if(listen_fd_ < 0)
    {
        LOG_ERROR << "MODBUS创建端口失败";
        std::cout << "Create Socket Failed!";
        exit(MODBUSEXITNO);
    }
    std::cout << "Create Socket Successfully" << std::endl;
    int opt = 1;
    setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    if(bind(listen_fd_, (struct sockaddr*)&server_addr_, sizeof(server_addr_)) < 0)
    {
        LOG_ERROR << "MODBUS绑定失败";
        exit(MODBUSEXITNO);
    }
    std::cout << "Bind Successfully.\n";

    if(listen(listen_fd_, queue_len_) < 0)
    {
        LOG_ERROR << "MODBUS监听失败";
        exit(MODBUSEXITNO);
    }
    std::cout << "Listen Successfully.\n";

    struct sockaddr_in client_addr;
    socklen_t client_addr_len = sizeof(client_addr);

    link_fd_ = accept(listen_fd_, (struct sockaddr*)&client_addr, &client_addr_len);
    if(link_fd_ < 0)
    {
        LOG_ERROR << "MODBUS接受连接失败";
        exit(MODBUSEXITNO);
    }

    std::string ip(inet_ntoa(client_addr.sin_addr));    // 获取客户端IP

    std::cout << ip << " new connection was accepted.\n";
}

ActionFinish::~ActionFinish()
{
    close(listen_fd_);
    close(link_fd_);
}

bool ActionFinish::AgvIsReached()
{
    while(1) {
        unsigned char buff[12];
        int n = recv(link_fd_, &buff, sizeof(buff), 0);
        while(n <= 0) {
            close(listen_fd_);
            close(link_fd_);

            listen_fd_ = socket(PF_INET, SOCK_STREAM, 0);

            if(listen_fd_ < 0)
            {
                LOG_ERROR << "MODBUS创建端口失败";
                exit(MODBUSEXITNO);
            }
            std::cout << "Create Socket Successfully" << std::endl;
            int opt = 1;
            setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

            if(bind(listen_fd_, (struct sockaddr*)&server_addr_, sizeof(server_addr_)) < 0)
            {
                LOG_ERROR << "MODBUS绑定失败";
                exit(MODBUSEXITNO);
            }
            std::cout << "Bind Successfully.\n";

            if(listen(listen_fd_, queue_len_) < 0)
            {
                LOG_ERROR << "MODBUS监听失败";
                exit(MODBUSEXITNO);
            }
            std::cout << "Listen Successfully.\n";

            struct sockaddr_in client_addr;
            socklen_t client_addr_len = sizeof(client_addr);

            link_fd_ = accept(listen_fd_, (struct sockaddr*)&client_addr, &client_addr_len);
            if(link_fd_ < 0)
            {
                LOG_ERROR << "MODBUS接受连接失败";
                exit(MODBUSEXITNO);
            }

            std::string ip(inet_ntoa(client_addr.sin_addr));    // 获取客户端IP
            std::cout << ip << " new connection was accepted.\n";

            n = recv(link_fd_, &buff, sizeof(buff), 0);
        }
        std::cout << "receive:";
        for (int i = 0; i < n; i++) {
            std::cout << std::hex << (unsigned int) (unsigned char) buff[i] << ",";
        }
        std::cout << std::endl;

        if (n == 12 && buff[7] == 0x06) {
            unsigned char send_buff[12];
            send_buff[0] = buff[0];
            send_buff[1] = buff[1];
            send_buff[2] = buff[2];
            send_buff[3] = buff[3];
            send_buff[4] = buff[4];
            send_buff[5] = buff[5];
            send_buff[6] = buff[6];
            send_buff[7] = buff[7];
            send_buff[8] = buff[8];
            send_buff[9] = buff[9];
            send_buff[10] = buff[10];
            send_buff[11] = buff[11];
            send(link_fd_, send_buff, sizeof(send_buff), 0);
            std::cout << "send:   ";
            for (int i = 0; i < sizeof(send_buff); i++)
                std::cout << (unsigned int) (unsigned char) send_buff[i] << ",";
            std::cout << std::endl;
        } else if (n == 12 && buff[7] == 0x03) {
            unsigned char send_buff[11];
            send_buff[0] = buff[0];
            send_buff[1] = buff[1];
            send_buff[2] = buff[2];
            send_buff[3] = buff[3];
            send_buff[4] = buff[4];
            send_buff[5] = 0x05;
            send_buff[6] = 0x01;
            send_buff[7] = 0x03;
            send_buff[8] = 0x02;
            send_buff[9] = 0x00;
            send_buff[10] = 0x01;
            send(link_fd_, send_buff, sizeof(send_buff), 0);
            std::cout << "send:   ";
            for (int i = 0; i < sizeof(send_buff); i++)
                std::cout << (unsigned int) (unsigned char) send_buff[i] << ",";
            std::cout << std::endl;
            break;
        }
    }
    return true;
}

bool ActionFinish::ArmFinishAction()
{
    while(1) {
        unsigned char buff[12];
        int n = recv(link_fd_, &buff, sizeof(buff), 0);

        while(n <= 0) {
            close(listen_fd_);
            close(link_fd_);
            listen_fd_ = socket(PF_INET, SOCK_STREAM, 0);
            if(listen_fd_ <= 0)
            {
                LOG_ERROR << "MODBUS创建端口失败";
                exit(MODBUSEXITNO);
            }
            std::cout << "Create Socket Successfully" << std::endl;
            int opt = 1;
            setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

            if(bind(listen_fd_, (struct sockaddr*)&server_addr_, sizeof(server_addr_)) < 0)
            {
                LOG_ERROR << "MODBUS绑定失败";
                exit(MODBUSEXITNO);
            }
            std::cout << "Bind Successfully.\n";

            if(listen(listen_fd_, queue_len_) < 0)
            {
                LOG_ERROR << "MODBUS监听失败";
                exit(MODBUSEXITNO);
            }
            std::cout << "Listen Successfully.\n";

            struct sockaddr_in client_addr;
            socklen_t client_addr_len = sizeof(client_addr);

            link_fd_ = accept(listen_fd_, (struct sockaddr*)&client_addr, &client_addr_len);
            if(link_fd_ < 0)
            {
                LOG_ERROR << "MODBUS接受连接失败";
                exit(MODBUSEXITNO);
            }
            std::string ip(inet_ntoa(client_addr.sin_addr));    // 获取客户端IP
            std::cout << ip << " new connection was accepted.\n";

            n = recv(link_fd_, &buff, sizeof(buff), 0);
        }

        std::cout << "receive:";
        for (int i = 0; i < n; i++) {
            std::cout << std::hex << (unsigned int) (unsigned char) buff[i] << ",";
        }
        std::cout << std::endl;

        if (n == 12 && buff[7] == 0x06) {
            unsigned char send_buff[12];
            send_buff[0] = buff[0];
            send_buff[1] = buff[1];
            send_buff[2] = buff[2];
            send_buff[3] = buff[3];
            send_buff[4] = buff[4];
            send_buff[5] = buff[5];
            send_buff[6] = buff[6];
            send_buff[7] = buff[7];
            send_buff[8] = buff[8];
            send_buff[9] = buff[9];
            send_buff[10] = buff[10];
            send_buff[11] = buff[11];
            send(link_fd_, send_buff, sizeof(send_buff), 0);
            std::cout << "send:   ";
            for (int i = 0; i < sizeof(send_buff); i++)
                std::cout << (unsigned int) (unsigned char) send_buff[i] << ",";
            std::cout << std::endl;
        } else if (n == 12 && buff[7] == 0x03) {
            unsigned char send_buff[11];
            send_buff[0] = buff[0];
            send_buff[1] = buff[1];
            send_buff[2] = buff[2];
            send_buff[3] = buff[3];
            send_buff[4] = buff[4];
            send_buff[5] = 0x05;
            send_buff[6] = 0x01;
            send_buff[7] = 0x03;
            send_buff[8] = 0x02;
            send_buff[9] = 0x00;
            send_buff[10] = 0x01;
            send(link_fd_, send_buff, sizeof(send_buff), 0);
            std::cout << "send:   ";
            for (int i = 0; i < sizeof(send_buff); i++)
                std::cout << (unsigned int) (unsigned char) send_buff[i] << ",";
            std::cout << std::endl;
            break;
        }
    }
    return true;
}