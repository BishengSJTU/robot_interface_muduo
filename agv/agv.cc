#include "agv.h"

AGV::AGV(const std::string config_file_name): config_(config_file_name)
{
    agv_ip_ = config_.get<std::string> ("AGV_IP");
    port_ = config_.get<int> ("AGV_PORT");
    url_pre_ = "ws://" + agv_ip_ + ":" + std::to_string(port_);
    map_name_ = config_.get<std::string> ("map_name");
}

bool AGV::AgvGo(const int& cab_id, const int& position, int &mission_id) {
    int all_position_num = config_.get<int>("cab" + std::to_string(cab_id) + "_all_position_num");
    int index;

    if (all_position_num == 26) {
        if (position >= 1 && position <= 7) {
            index = 1;
        } else if (position >= 8 && position <= 14) {
            index = 2;
        } else if (position >= 15 && position <= 21) {
            index = 3;
        } else if (position >= 22 && position <= 26) {
            index = 4;
        }
    } else if(all_position_num == 32) {
        if (position >= 1 && position <= 5) {
            index = 1;
        } else if (position >= 6 && position <= 10) {
            index = 2;
        } else if (position >= 11 && position <= 16) {
            index = 3;
        } else if (position >= 17 && position <= 22) {
            index = 4;
        } else if (position >= 23 && position <= 27) {
            index = 5;
        } else if (position >= 28 && position <= 32) {
            index = 6;
        }
    } else if (all_position_num == 35) {
        if (position >= 1 && position <= 6) {
            index = 1;
        } else if (position >= 7 && position <= 12) {
            index = 2;
        } else if (position >= 13 && position <= 18) {
            index = 3;
        } else if (position >= 19 && position <= 24) {
            index = 4;
        } else if (position >= 25 && position <= 30) {
            index = 5;
        } else if (position >= 31 && position <= 35) {
            index = 6;
        }
    } else if (all_position_num == 33) {
        if (position >= 1 && position <= 5) {
            index = 1;
        } else if (position >= 6 && position <= 11) {
            index = 2;
        } else if (position >= 12 && position <= 17) {
            index = 3;
        } else if (position >= 18 && position <= 23) {
            index = 4;
        } else if (position >= 24 && position <= 28) {
            index = 5;
        } else if (position >= 29 && position <= 33) {
            index = 6;
        }
    }

    if (cab_id == 0) {
        index = position;
    }


    std::vector<float> map_position_info =
            config_.get<std::vector<float>>("map_position_cab" + std::to_string(cab_id) + "_" + std::to_string(index));

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
        exit(-1);


    //　发送导航命令
    message = std::string(
            "{\"id\":\"publish:/move_base/goal\",\"latch\":false,\"msg\":{\"goal\":{\"target_pose\":{\"header\":{\"frame_id\":\"/map\",\"seq\":0,\"stamp\":{\"nsecs\":0,\"secs\":0}},\"pose\":{\"orientation\":") +
              "{\"w\":" + std::to_string(w) + ",\"x\":0,\"y\":0,\"z\":" + std::to_string(z) + "},\"position\":{\"x\":" +
              std::to_string(x) + ",\"y\":" + std::to_string(y) + ",\"z\":0" +
              "}}}},\"goal_id\":{\"id\":\"dymove_{" + code +
              "}\",\"stamp\":{\"nsecs\":0,\"secs\":0}},\"header\":{\"frame_id\":\"/map\",\"seq\":0,\"stamp\":{\"nsecs\":0,\"secs\":0}}},\"op\":\"publish\",\"topic\":\"/move_base/goal\"}";

    std::cout << "send:" << message << std::endl;
    send_result = endpoint_.send(message);
    sleep(1);
    if (!send_result)
        exit(-1);


    while (1) {
        std::map<std::string, int>& info = endpoint_.get_set_info();
        if (info.find("status") == info.end())
            continue;
        else {
            std::cout << "info_status:" << info["status"] << std::endl;
            if (info["status"] == 3) {
                // 达到目的地，状态为置-1，方便下次的判断
                info.erase("status");
                return true;
            } else {
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

    if( (pf = popen(strCmd.c_str(), "r")) == NULL )
    {
        return false;
    }
    string strResult;
    while(fgets(buf, sizeof buf, pf))
    {
        strResult += buf;
    }
    pclose(pf);
    unsigned int iSize =  strResult.size();
    if(iSize > 0 && strResult[iSize - 1] == '\n')  // linux
    {
        strResult = strResult.substr(0, iSize - 1);
    }

    if(strResult.find("received") != string::npos && strResult.find(", 0 received") == string::npos)
        return true;
    else
        return false;
}

int AGV::InitializeAGV()
{
    while(1) {
        if(getCmdResult(agv_ip_)) {
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
                        return info["power"];
                    }
                }
            }
        } else {
            std::cout << "Network not reachable" << std::endl;
        }
        sleep(1);
    }
}

bool AGV::AgvReached()
{
    return true;
}

bool AGV::ActionFinishedAgvGo()
{
    return true;
}

void AGV::GetAgvPowerAndState(int& power, int& state){
    std::string message = "{\"op\":\"subscribe\",\"id\":\"subscribe:/bottom_byteinfo\",\"type\":\"std_msgs/ByteMultiArray\",\"topic\":\"/bottom_byteinfo\",\"compression\":\"none\",\"throttle_rate\":0,\"queue_length\":0}";
    bool send_result = endpoint_.send(message);
    if (send_result) {
        sleep(1);
        message = "{\"id\":\"publish:/mb_control\",\"topic\":\"/mb_control\",\"latch\":false,\"op\":\"publish\",\"msg\":{\"data\":\"CMD_GetBottomMsg\"}}";
        bool send_result = endpoint_.send(message);
        sleep(1);

        std::map<std::string, int> info = endpoint_.get_set_info();
        if(info.find("power") == info.end()) {
            power = 99;
            state = 1;
        }
        else {
            power = info["power"];
            state = 1;
        }
    }
}

void AGV::CancelTask(const int &agv_mission_id)
{
    ;
}