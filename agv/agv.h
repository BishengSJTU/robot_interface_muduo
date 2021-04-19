#ifndef AGV_H
#define AGV_H

#define AGVEXITNO 5

#include "http_client.h"
#include "config.h"
#include "action_finish.h"
#include "websocket_endpoint.h"
#include "mapping_table.h"
#include <stdlib.h>
#include <time.h>
#include "Logging.h"
#include "LogFile.h"

#define AGV_MT 0 //维护
#define AGV_READY 1
#define AGV_BUSY 2
#define AGV_CHARGING 3
#define AGV_FAILURE 4

namespace muyi {
    class AGV {
    private:
        std::string agv_ip_;
        int port_;
        std::string url_pre_;
        std::string map_name_;

        // 配置文件
        const Config fixed_config_;
        const Config flexible_config_;

        websocket_endpoint endpoint_;
        void InitializeAGV();

    public:
        AGV(const std::string config_file_path, bool isInline = true);

        bool AgvGo(const int &cab_id, const int &position, int &mission_id);

        bool AgvReached();

        bool ActionFinishedAgvGo();

        void GetAgvPowerAndState(int &power, int &state);

        void CancelTask(const int &agv_mission_id);
    };
}

namespace jiazhi {
    class AGV {
    private:
        std::string agv_ip_;
        int port_;
        std::string url_pre_;
        std::string map_name_;

        // 配置文件
        const Config fixed_config_;
        const Config flexible_config_;

        void InitializeAGV();

    public:
        AGV(const std::string config_file_path, bool isInline = true);

        bool AgvGo(const int &cab_id, const int &position, int &mission_id);

        bool AgvReached();

        bool ActionFinishedAgvGo();

        void GetAgvPowerAndState(int &power, int &state);

        void CancelTask(const int &agv_mission_id);
    };
}

#endif