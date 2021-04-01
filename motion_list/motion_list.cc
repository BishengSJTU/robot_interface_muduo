#include "motion_list.h"

void MotionList::UpdateMotionLists(const std::string &motion_list_file_name) {
    cv::FileStorage fs(motion_list_file_name, cv::FileStorage::READ);

    cv::FileNode PickCabMotionList1 = fs["PickCabMotionList1"];
    cv::FileNodeIterator it = PickCabMotionList1.begin(), it_end = PickCabMotionList1.end();
    UpdateMotionList(it, it_end, pick_cab_motion_list_1);

    cv::FileNode PickCabMotionList2 = fs["PickCabMotionList2"];
    it = PickCabMotionList2.begin(), it_end = PickCabMotionList2.end();
    UpdateMotionList(it, it_end, pick_cab_motion_list_2);

    cv::FileNode PlaceCabMotionList = fs["PlaceCabMotionList"];
    it = PlaceCabMotionList.begin(), it_end = PlaceCabMotionList.end();
    UpdateMotionList(it, it_end, place_cab_motion_list_);

    cv::FileNode PickWindowMotionList = fs["PickWindowMotionList"];
    it = PickWindowMotionList.begin(), it_end = PickWindowMotionList.end();
    UpdateMotionList(it, it_end, pick_window_motion_list_);

    cv::FileNode PlaceWindowMotionList = fs["PlaceWindowMotionList"];
    it = PlaceWindowMotionList.begin(), it_end = PlaceWindowMotionList.end();
    UpdateMotionList(it, it_end, place_window_motion_list_);

    cv::FileNode PickStorageMotionList = fs["PickStorageMotionList"];
    it = PickStorageMotionList.begin(), it_end = PickStorageMotionList.end();
    UpdateMotionList(it, it_end, pick_storage_motion_list_);

    cv::FileNode PlaceStorageMotionList = fs["PlaceStorageMotionList"];
    it = PlaceStorageMotionList.begin(), it_end = PlaceStorageMotionList.end();
    UpdateMotionList(it, it_end, place_storage_motion_list_);

    fs.release();
}

void UpdateMotionList(cv::FileNodeIterator it, cv::FileNodeIterator it_end, std::vector<std::vector<int> > &motion_list) {
    //使用FileNodeIterator遍历序列
    int idx = 0;
    for (; it != it_end; ++it, idx++) {
        std::vector<int> motion;
        motion.push_back(idx);
        if ((std::string)(*it)["Device"] == "Robot") {

            motion.push_back(0);


            if ((std::string)(*it)["Mode"] == "Inc")
                motion.push_back(0);
            else if ((std::string)(*it)["Mode"] == "Abs")
                motion.push_back(1);
            else if ((std::string)(*it)["Mode"] == "Con")
                motion.push_back(2);
            else {
                std::cout << "Unknow Mode index in [parameter/MotionList.yaml] " << std::endl;
                exit(-1);
            }


            if ((std::string)(*it)["Coord"] == "Joint")
                motion.push_back(0);
            else if ((std::string)(*it)["Coord"] == "World")
                motion.push_back(1);
            else if ((std::string)(*it)["Coord"] == "Tool")
                motion.push_back(2);
            else {
                std::cout << "Unknow Coord index in [parameter/MotionList.yaml] " << std::endl;
                exit(-1);
            }

            if ((std::string)(*it)["Aixs"] == "X")
                motion.push_back(0);
            else if ((std::string)(*it)["Aixs"] == "Y")
                motion.push_back(1);
            else if ((std::string)(*it)["Aixs"] == "Z")
                motion.push_back(2);
            else if ((std::string)(*it)["Aixs"] == "RX")
                motion.push_back(3);
            else if ((std::string)(*it)["Aixs"] == "RY")
                motion.push_back(4);
            else if ((std::string)(*it)["Aixs"] == "RZ")
                motion.push_back(5);
            else {
                std::cout << "Unknow Aixs index in [parameter/MotionList.yaml] " << std::endl;
                exit(-1);
            }
            motion.push_back((int) (*it)["Speed"]);
            motion.push_back((int) (*it)["Dist"]);
        } else if ((std::string)(*it)["Device"] == "PLC") {
            motion.push_back(1);
            if ((std::string)(*it)["IoType"] == "ElectricalMachinery")
                motion.push_back(0);
            else if ((std::string)(*it)["IoType"] == "Electromagnet")
                motion.push_back(1);
            else if ((std::string)(*it)["IoType"] == "AirPump")
                motion.push_back(2);
            else {
                std::cout << "Unknow IoType index in [parameter/MotionList.yaml] " << std::endl;
                exit(-1);
            }
            motion.push_back((int) (*it)["IoValue"]);
        } else {
            std::cout << "Unknow Device index in [parameter/MotionList.yaml] " << std::endl;
            exit(-1);
        }
        motion_list.push_back(motion);
    }
}

void ExecuteMotionList(RobotClient &robot_client, MyPLC &plc, const std::vector<std::vector<int> >& motion_list) {
    for (int i = 0; i < motion_list.size(); ++i) {
        for (int j = 0; j < motion_list[i].size(); ++j) {
            std::cout << motion_list[i][j] << " ";
        }
        std::cout << std::endl;
    }
    for (int i = 0; i < motion_list.size(); ++i) {
        if (motion_list[i][1] == 0) {
            std::cout << "Operating Device is Robot" << std::endl;
            if (motion_list[i].size() != 7) {
                std::cout << "Error: Incorrect size of JOG in yaml file." << std::endl;
                exit(-1);
            }
            robot_client.Jog(motion_list[i][2], motion_list[i][3], motion_list[i][4], motion_list[i][5],
                             motion_list[i][6]);
            switch (motion_list[i][2]) {
                case 0:
                    std::cout << "Mode:Increase; ";
                    break;
                case 1:
                    std::cout << "Mode:Abslute; ";
                    break;
                case 2:
                    std::cout << "Mode:Continue; ";
                    break;
            }
            switch (motion_list[i][3]) {
                case 0:
                    std::cout << "Coord:Joint; ";
                    break;
                case 1:
                    std::cout << "Coord:World; ";
                    break;
                case 2:
                    std::cout << "Coord:Tool; ";
                    break;
            }
            switch (motion_list[i][4]) {
                case 0:
                    std::cout << "Aixs:X; ";
                    break;
                case 1:
                    std::cout << "Aixs:Y; ";
                    break;
                case 2:
                    std::cout << "Aixs:Z; ";
                    break;
                case 3:
                    std::cout << "Aixs:RX; ";
                    break;
                case 4:
                    std::cout << "Aixs:RY; ";
                    break;
                case 5:
                    std::cout << "Aixs:RZ; ";
                    break;
            }

            std::cout << "Speed:" << motion_list[i][5] << " ;";
            std::cout << "Dist :" << motion_list[i][6] << std::endl;;
        } else if (motion_list[i][1] == 1) {
            std::cout << "Operating Device is PLC" << std::endl;
            if (motion_list[i].size() != 4) {
                std::cout << "Error: Incorrect size of IO Controll in yaml file." << std::endl;
                exit(-1);
            }
            if (motion_list[i][2] == 0) {
                if (motion_list[i][3] == 0) {
                    plc.ControlPLC(1);
                } else if (motion_list[i][3] == 1) {
                    plc.ControlPLC(2);
                } else {
                    std::cout << "Error: Incorrect value of PLC IOvalue." << std::endl;
                    exit(-1);
                }
            } else if (motion_list[i][2] == 1) {
                if (motion_list[i][3] == 0) {
                    plc.ControlPLC(6);
                } else if (motion_list[i][3] == 1) {
                    plc.ControlPLC(5);
                } else {
                    std::cout << "Error: Incorrect value of PLC IOvalue." << std::endl;
                    exit(-1);
                }
            } else if (motion_list[i][2] == 2) {
                if (motion_list[i][3] == 0) {
                    plc.ControlPLC(8);
                    usleep(5e5);

                } else if (motion_list[i][3] == 1) {
                    plc.ControlPLC(7);
                    usleep(5e5);
                } else {
                    std::cout << "Error: Incorrect value of PLC Iovalue." << std::endl;
                    exit(-1);
                }
            } else {
                std::cout << "Error: Incorrect value of PLC IoType." << std::endl;
                exit(-1);
            }
        }
    }
}