#ifndef JAKA_PLC_H
#define JAKA_PLC_H

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

class JAKAPLC {
private:
    int clientSocket_;
    struct sockaddr_in clientAddr_;
    std::string address_;
    int port_;
    void InitializePLC(std::string address, int port = 502);
public:
    JAKAPLC(std::string address, int port = 502, bool isInline = true);
    ~JAKAPLC();
    bool ControlPLC(int command);
    void InquireState();
    bool ArchiveInHand();
};
#endif
