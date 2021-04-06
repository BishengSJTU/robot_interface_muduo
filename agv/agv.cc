#include "agv.h"

namespace muyi {
    AGV::AGV(const std::string config_file_name) : config_(config_file_name) {
        agv_ip_ = config_.get<std::string>("AGV_IP");
        port_ = config_.get<int>("AGV_PORT");
        url_pre_ = "ws://" + agv_ip_ + ":" + std::to_string(port_);
        map_name_ = config_.get<std::string>("map_name");
        InitializeAGV();
    }

    bool AGV::AgvGo(const int &cab_id, const int &position, int &mission_id) {
        int all_position_num = config_.get<int>("cab" + std::to_string(cab_id) + "_all_position_num");
        int index;
        int benchmark_position;

        MappingTable(all_position_num, position, index, benchmark_position);

        if (cab_id == 0) {
            index = position;
        }


        std::vector<float> map_position_info =
                config_.get<std::vector<float>>(
                        "map_position_cab" + std::to_string(cab_id) + "_" + std::to_string(index));

        double x, y, z, w;
        x = map_position_info[0];
        y = map_position_info[1];
        z = map_position_info[2];
        w = map_position_info[3];

        std::string code;
        srand((unsigned) time(NULL));
        int a = (rand() % (999999999 - 100000000 + 1)) + 100000000;
        code = std::string("43adfc2c-8c2d-4f41-9593-123") + std::to_string(a);

        // 订阅导航消息
        std::string message;
        message = "{\"op\":\"subscribe\",\"id\":\"subscribe:/move_base/result\",\"type\":\"move_base_msgs/MoveBaseActionResult\",\"topic\":\"/move_base/result\",\"compression\":\"none\",\"throttle_rate\":0,\"queue_length\":0}";
        bool send_result = endpoint_.send(message);
        sleep(1);
        if (!send_result)
            exit(AGVEXITNO);


        //　发送导航命令
        message = std::string(
                "{\"id\":\"publish:/move_base/goal\",\"latch\":false,\"msg\":{\"goal\":{\"target_pose\":{\"header\":{\"frame_id\":\"/map\",\"seq\":0,\"stamp\":{\"nsecs\":0,\"secs\":0}},\"pose\":{\"orientation\":") +
                  "{\"w\":" + std::to_string(w) + ",\"x\":0,\"y\":0,\"z\":" + std::to_string(z) +
                  "},\"position\":{\"x\":" +
                  std::to_string(x) + ",\"y\":" + std::to_string(y) + ",\"z\":0" +
                  "}}}},\"goal_id\":{\"id\":\"dymove_{" + code +
                  "}\",\"stamp\":{\"nsecs\":0,\"secs\":0}},\"header\":{\"frame_id\":\"/map\",\"seq\":0,\"stamp\":{\"nsecs\":0,\"secs\":0}}},\"op\":\"publish\",\"topic\":\"/move_base/goal\"}";

        std::cout << "Send:" << message << std::endl;
        send_result = endpoint_.send(message);
        sleep(1);
        if (!send_result)
            exit(AGVEXITNO);


        while (1) {
            std::map<std::string, int> &info = endpoint_.get_set_info();
            if (info.find("status") == info.end())
                continue;
            else {
                std::cout << "status:" << info["status"] << std::endl;
                if (info["status"] == 3) {
                    LOG_INFO << "机器人到达目的地";
                    // 达到目的地，状态为置-1，方便下次的判断
                    info.erase("status");
                    return true;
                } else {
                    LOG_ERROR << "机器人无法到达目的地";
                    continue;
                }
            }
        }
    }

// 判断能不能ping 通
    bool getCmdResult(const string Cmd)  // 这个是获取命令执行的结果， 类似于system, 之前我已经说过了
    {
        std::string strCmd = "ping " + Cmd + " -w 1";
        char buf[10240] = {0};
        FILE *pf = NULL;

        if ((pf = popen(strCmd.c_str(), "r")) == NULL) {
            return false;
        }
        string strResult;
        while (fgets(buf, sizeof buf, pf)) {
            strResult += buf;
        }
        pclose(pf);
        unsigned int iSize = strResult.size();
        if (iSize > 0 && strResult[iSize - 1] == '\n')  // linux
        {
            strResult = strResult.substr(0, iSize - 1);
        }

        if (strResult.find("received") != string::npos && strResult.find(", 0 received") == string::npos)
            return true;
        else
            return false;
    }

    void AGV::InitializeAGV() {
        while (1) {
            if (getCmdResult(agv_ip_)) {
                sleep(1);
                endpoint_.connect(url_pre_);
                sleep(1);
                if (endpoint_.get_status() == "Open") {
                    std::string message = "{\"op\":\"subscribe\",\"id\":\"subscribe:/bottom_byteinfo\",\"type\":\"std_msgs/ByteMultiArray\",\"topic\":\"/bottom_byteinfo\",\"compression\":\"none\",\"throttle_rate\":0,\"queue_length\":0}";
                    bool send_result = endpoint_.send(message);
                    sleep(1);
                    if (send_result) {
                        message = "{\"id\":\"publish:/mb_control\",\"topic\":\"/mb_control\",\"latch\":false,\"op\":\"publish\",\"msg\":{\"data\":\"CMD_GetBottomMsg\"}}";
                        bool send_result = endpoint_.send(message);
                        sleep(1);

                        std::map<std::string, int> info = endpoint_.get_set_info();
                        if (info.find("power") == info.end()) {
                            int close_code = websocketpp::close::status::normal;
                            // 没有相关消息，关闭连接，重新尝试
                            endpoint_.close(close_code);
                            continue;
                        } else {
                            return;
                        }
                    }
                }
            } else {
                LOG_ERROR << "Muyi网络故障,不可达";
            }
            sleep(1);
        }
    }

    void AGV::GetAgvPowerAndState(int &power, int &state) {
        std::string message = "{\"op\":\"subscribe\",\"id\":\"subscribe:/bottom_byteinfo\",\"type\":\"std_msgs/ByteMultiArray\",\"topic\":\"/bottom_byteinfo\",\"compression\":\"none\",\"throttle_rate\":0,\"queue_length\":0}";
        bool send_result = endpoint_.send(message);
        if (send_result) {
            sleep(1);
            message = "{\"id\":\"publish:/mb_control\",\"topic\":\"/mb_control\",\"latch\":false,\"op\":\"publish\",\"msg\":{\"data\":\"CMD_GetBottomMsg\"}}";
            bool send_result = endpoint_.send(message);
            sleep(1);

            std::map<std::string, int> info = endpoint_.get_set_info();
            if (info.find("power") == info.end()) {
                power = 99;
                state = AGV_FAILURE;
            } else {
                power = info["power"];
                state = AGV_READY;
            }
        }
    }

    bool AGV::AgvReached() {
        return true;
    }

    bool AGV::ActionFinishedAgvGo() {
        return true;
    }

    void AGV::CancelTask(const int &agv_mission_id) {
        return ;
    }
}

namespace jiazhi {
    AGV::AGV(const std::string config_file_name) : config_(config_file_name) {
        agv_ip_ = config_.get<std::string>("AGV_IP");
        port_ = config_.get<int>("AGV_PORT");
        url_pre_ = "http://" + agv_ip_ + ":" + std::to_string(port_);
        map_name_ = config_.get<std::string>("map_name");
        InitializeAGV();
    }

    bool AGV::AgvGo(const int &cab_id, const int &position, int &mission_id) {
        int all_position_num = config_.get<int>("cab" + std::to_string(cab_id) + "_all_position_num");
        int index;
        int benchmark_position;

        MappingTable(all_position_num, position, index, benchmark_position);

        if (cab_id == 0) {
            index = position;
        }


        std::vector<std::string> map_position_info =
                config_.get<std::vector<std::string>>(
                        "map_position_cab" + std::to_string(cab_id) + "_" + std::to_string(index));
        std::string target_code = map_position_info[0];
        std::string action = map_position_info[1];

        HttpRequest req_;
        HttpResponse res_;
        req_.method = HTTP_POST;
        req_.url = url_pre_ + "/api/v1/missions";
        req_.json = nlohmann::json{
                {"ref_uuid",      ""},
                {"src",           ""},
                {"description",   ""},
                {"req_robot",     0},
                {"req_robgroups", ""},
                {"id",            0},
                {"robot_id",      0},
                {"state",         0},
                {"err_msg",       ""},
                {"steps",         {{
                                           {"map_name", map_name_},
                                           {"target_code", target_code},
                                           {"action", action},
                                           {"args", ""}
                                   }}
                }
        };
        http_client_send(&req_, &res_);
        string result(res_.body);
        try {
            auto js = Json::parse(result);
            mission_id = js["id"];
        } catch (...) {
            LOG_ERROR << "Jiazhi 任务发送失败";
        }

        if (res_.status_code == 201)
            return true;
        else
            return false;
    }

    void AGV::InitializeAGV() {
        HttpRequest req2;
        req2.method = HTTP_GET;
        req2.url = url_pre_ + "/api/v1/robots";
        HttpResponse res2;
        while (1) {
            try {
                int ret2 = http_client_send(&req2, &res2);
                string result(res2.body);
                try {
                    auto js = Json::parse(result);
                    int run_state = js[0]["run_state"];
                    std::cout << "run_state:" << run_state << std::endl;
                    // 就绪中、充电中
                    if (run_state == AGV_READY || run_state == AGV_BUSY || run_state == AGV_CHARGING || run_state == AGV_FAILURE) {
                        return;
                    } else {
                        LOG_INFO << "Jiazhi初始化中... 状态仍在初始化中";
                    }
                } catch (...) {
                    LOG_ERROR << "Jiazhi初始化中... Json字符串解析不正确";
                }
            } catch (...) {
                LOG_INFO << "Jiazhi初始化中... HTTP发送未就绪";
            }
            sleep(5);
        }
    }

    bool AGV::AgvReached() {
        ActionFinish action_finish;
        return action_finish.AgvIsReached();
    }

    bool AGV::ActionFinishedAgvGo() {
        ActionFinish action_finish;
        return action_finish.ArmFinishAction();
    }

    void AGV::GetAgvPowerAndState(int &power, int &state) {
        HttpRequest req2;
        req2.method = HTTP_GET;
        req2.url = url_pre_ + "/api/v1/robots";
        HttpResponse res2;
        try {
            http_client_send(&req2, &res2);
            string result(res2.body);
            try {
                auto js = Json::parse(result);
                power = js[0]["battery"];
                state = js[0]["run_state"];
            } catch (...) {
                power = 99;
                state = AGV_FAILURE;
                return;
            }
        }
        catch (...) {
            power = 99;
            state = AGV_FAILURE;
            return;
        }
    }

    void AGV::CancelTask(const int &agv_mission_id) {
        HttpRequest req_;
        HttpResponse res_;
        req_.method = HTTP_POST;
        req_.url = url_pre_ + "/api/v1/mscmds";
        req_.json = nlohmann::json{
                {"mission_id",   agv_mission_id},
                {"mission_uuid", ""},
                {"cmd",          2},
                {"sync",         true},
                {"id",           0},
                {"result",       0},
                {"err_msg",      ""},
        };
        http_client_send(&req_, &res_);
    }
}