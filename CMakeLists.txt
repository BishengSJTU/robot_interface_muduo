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
        "robotic_arm/*.cc"
        "kinematics/*.cc"
        "pick_and_place/*.cc"
        "vision_detection/*.cc"
        "mapping_table/*.cc"
        "agv_wait_action_finish/*.cc"
        "agv/*.cc"
)

include_directories(
        datetime
        logging
        net
        thread
        plc
        config
        vision_detection
        robotic_arm
        kinematics
        pick_and_place
        vision_detection
        mapping_table
        agv_wait_action_finish
        agv
        websocket
)
include_directories(${JSON_DIR} ${ZBAR_DIR} ${OpenCV_INCLUDE_DIRS} ${HV_DIR} ${APRILTAG_DIR})
link_directories(${LIB_HV_DIR} ${LIB_APRILTAG_DIR})

add_executable(JakaRobotInterface
        robot_interface/jaka.cc
        robot_interface/jaka_robot_interface.cc
        robot_interface/jaka_robot_interface.h
        robot_interface/codec.h
        ${ALL_SOURCES})
target_link_libraries(JakaRobotInterface pthread ${OpenCV_LIBS} libhv.so zbar pthread boost_system libapriltag.so.3)

#add_executable(PlatformRobotInterface
#        robot_interface/platform.cc
#        robot_interface/platform_robot_interface.cc
#        robot_interface/platform_robot_interface.h
#        robot_interface/codec.h
#        ${ALL_SOURCES})
#target_link_libraries(PlatformRobotInterface pthread ${OpenCV_LIBS} libhv.so zbar pthread boost_system libapriltag.so.3)


#add_executable(Channel_test
#        test/Channel_test.cc
#        ${ALL_SOURCES})
#target_link_libraries(Channel_test pthread)

#add_executable(EventLoop_test1
#        test/EventLoop_test1.cc
#        ${ALL_SOURCES})
#target_link_libraries(EventLoop_test1 pthread)
#
#add_executable(EventLoop_test2
#        test/EventLoop_test2.cc
#        ${ALL_SOURCES})
#target_link_libraries(EventLoop_test2 pthread)

#add_executable(EventLoop_test3
#        test/EventLoop_test3.cc
#        ${ALL_SOURCES})
#target_link_libraries(EventLoop_test3 pthread)
#
#add_executable(Timer_test1
#        test/Timer_test1.cc
#        ${ALL_SOURCES})
#target_link_libraries(Timer_test1 pthread)
#
#add_executable(Timer_test2
#        test/Timer_test2.cc
#        ${ALL_SOURCES})
#target_link_libraries(Timer_test2 pthread)
#
#add_executable(TcpConnection_test
#        test/TcpConnection_test.cc
#        ${ALL_SOURCES})
#target_link_libraries(TcpConnection_test pthread)
#
#add_executable(LogFile_test
#        test/LogFile_test.cc
#        ${ALL_SOURCES})
#target_link_libraries(LogFile_test pthread)

#add_executable(Logging_test
#        test/Logging_test.cc
#        ${ALL_SOURCES})
#target_link_libraries(Logging_test pthread)

add_executable(TcpServer_test
        test/TcpServer_test.cc
        ${ALL_SOURCES})
target_link_libraries(TcpServer_test pthread ${OpenCV_LIBS} libhv.so zbar pthread boost_system libapriltag.so.3)

#add_executable(TcpClient_test
#        test/TcpClient_test.cc
#        ${ALL_SOURCES})
#target_link_libraries(TcpClient_test pthread ${OpenCV_LIBS} libhv.so zbar pthread boost_system libapriltag.so.3)

#add_executable(Buffer_test
#        test/Buffer_test.cc
#        ${ALL_SOURCES})
#target_link_libraries(Buffer_test pthread)
