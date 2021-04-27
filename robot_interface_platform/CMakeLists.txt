cmake_minimum_required(VERSION 3.5.1)
project(archive_robot)

set(CMAKE_CXX_STANDARD 11)

set(JSON_DIR /usr/include/jsoncpp)
set(ZBAR_DIR /usr/include/zbar)
set(HV_DIR /usr/local/include/hv)
set(LIB_HV_DIR /usr/local/lib)
set(WS_DIR /usr/local/include/websocketpp)
set(APRILTAG_DIR /usr/local/include/apriltag)
set(LIB_APRILTAG_DIR /usr/local/lib)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED filesystem program_options regex timer chrono)

FILE (
        GLOB ALL_SOURCES
        "datetime/*.cc"
        "logging/*.cc"
        "net/*.cc"
        "thread/*.cc"
        "plc/*.cc"
        "config/*.cc"
        "vision_detection/*.cc"
        "pick_and_place/*.cc"
        "vision_detection/*.cc"
        "mapping_table/*.cc"
        "agv_wait_action_finish/*.cc"
        "agv/*.cc"
        "pick_and_place/*.cc"
)

include_directories(
        datetime
        logging
        net
        thread
        plc
        config
        vision_detection
        pick_and_place
        vision_detection
        mapping_table
        agv_wait_action_finish
        agv
        websocket
        pick_and_place
)
include_directories(${JSON_DIR} ${ZBAR_DIR} ${OpenCV_INCLUDE_DIRS} ${HV_DIR} ${APRILTAG_DIR})
link_directories(${LIB_HV_DIR} ${LIB_APRILTAG_DIR})

add_executable(PlatformRobotInterface
        robot_interface/platform.cc
        robot_interface/platform_robot_interface.cc
        robot_interface/platform_robot_interface.h
        robot_interface/codec.h
        ${ALL_SOURCES})
target_link_libraries(PlatformRobotInterface pthread ${OpenCV_LIBS} libhv.so zbar pthread boost_system libapriltag.so.3)