#ifndef MY_PLC_H
#define MY_PLC_H

#define RW_NUM 12
#define STATE_NUM 13

#include <string>
#include <vector>
#include <memory>
#include <unistd.h>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sstream>
#include <assert.h>
#include "Logging.h"
#include "Logging.h"

#define EXITNUM -1

class MyPLC {
private:
    int clientSocket_;
    struct sockaddr_in clientAddr_;
    std::string address_;
    int port_;
public:
    MyPLC();
    ~MyPLC();
    void InitializePLC(std::string address, int port = 502);
    bool ControlPLC(int command);
    void InquireState();
    bool ArchiveInHand();
    void MovePlatform(int x_distance, int y_distance);
    void ManipulateArchive(int arm_position, int mode);
    void PickArchive(int arm_position);
    void PlaceArchive(int arm_position);
    void GetPlatformState(int &x_current, int &y_current, int &reset_state, int &archive_state, int &sensor);
    void ResetPlatform();
};
#endif
