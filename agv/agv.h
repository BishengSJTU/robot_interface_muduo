#ifndef AGV_H
#define AGV_H

#include "http_client.h"
#include "config.h"
#include "action_finish.h"
#include "websocket_endpoint.h"
#include <stdlib.h>
#include <time.h>

class AGV
{
private:
    std::string agv_ip_;
    int port_;
    std::string url_pre_;
    std::string map_name_;
    const Config config_;
    websocket_endpoint endpoint_;

public:
    AGV(const std::string config_file_name);
    int InitializeAGV();
    bool AgvGo(const int& cab_id, const int& position, int &mission_id);
    bool AgvReached();
    bool ActionFinishedAgvGo();
    void GetAgvPowerAndState(int& power, int& state);
    void CancelTask(const int &agv_mission_id);
};

#endif