#ifndef PLATFORM_PLC_H
#define PLATFORM_PLC_H

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
#include "LogFile.h"

#define PLCEXITNUM 1
#define PLCTCPWAITTIME 1e5
#define MoveF 1
#define MoveB 2
#define MoveMF 3
#define AirpumpO 7
#define AirpumpC 8

class PlatformPLC {
private:
    int clientSocket_;
    struct sockaddr_in clientAddr_;
    std::string address_;
    int port_;
    void InitializePLC(std::string address, int port = 502);
public:
    PlatformPLC(std::string address, int port = 502, bool isInline = true);
    ~PlatformPLC();
    bool ControlPLC(int command);
    void InquireState();
    bool ArchiveInHand();
    void Move(int x_distance, int y_distance);
    void ManipulateArchive(int arm_position, int mode);
    void PickArchive(int arm_position);
    void PlaceArchive(int arm_position);
    void GetState(int &x_current, int &y_current, int &reset_state, int &archive_state, int &sensor);
    void Reset();
};
#endif
