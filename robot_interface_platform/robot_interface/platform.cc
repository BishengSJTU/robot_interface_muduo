#include "platform_robot_interface.h"
#include "Thread.h"
#include "LogFile.h"
#include "Logging.h"

std::unique_ptr<muduo::LogFile> g_logFile;
std::unique_ptr<muduo::LogFile> g_logStdout;

void outputFunc(const char* msg, int len)
{
    g_logFile->append(msg, len);
    g_logFile->flush();
}

int main(int argc, char* argv[]) {
    char name[256] = { '\0' };
    strncpy(name, argv[0], sizeof name - 1);
    std::string path = ::dirname(name);

//    g_logFile.reset(new muduo::LogFile(::basename(name), 600*1000));
//    muduo::Logger::setOutput(outputFunc);

    RobotInterface robotInterface(path + "/config");
    muduo::Thread receive_task(std::bind(&RobotInterface::eventLoopThread, &robotInterface));
    muduo::Thread execute_task(std::bind(&RobotInterface::execTaskThread, &robotInterface));
    muduo::Thread inquireThread(std::bind(&RobotInterface::inquireRobotStateThread, &robotInterface));

    receive_task.start();
    execute_task.start();
    inquireThread.start();

    receive_task.join();
    execute_task.join();
    inquireThread.join();

    return 0;
}