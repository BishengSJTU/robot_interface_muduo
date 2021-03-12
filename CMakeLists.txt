cmake_minimum_required(VERSION 3.5.1)
project(archive_robot)

set(CMAKE_CXX_STANDARD 11)

find_package(Boost REQUIRED filesystem program_options regex timer chrono)

FILE (GLOB ALL_SOURCES "datetime/*.cc" "logging/*.cc" "net/*.cc" "thread/*.cc")

include_directories(datetime logging net thread robot_interface)

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
target_link_libraries(TcpServer_test pthread)

add_executable(TcpClient_test
        test/TcpClient_test.cc
        ${ALL_SOURCES})
target_link_libraries(TcpClient_test pthread)

add_executable(RobotInterface
        robot_interface/robot.cc
        robot_interface/robot_interface.cc
        robot_interface/codec.h
        ${ALL_SOURCES})
target_link_libraries(RobotInterface pthread)

add_executable(Buffer_test
        test/Buffer_test.cc
        ${ALL_SOURCES})
target_link_libraries(Buffer_test pthread)
