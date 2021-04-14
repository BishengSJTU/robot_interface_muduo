#include "jaka_robot_interface.h"
#include "Thread.h"
#include "LogFile.h"
#include "Logging.h"
#include "robot_client_tcp.h"

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

    g_logFile.reset(new muduo::LogFile(::basename(name), 600*1000));
    muduo::Logger::setOutput(outputFunc);

    RobotInterface robotInterface(path);
    muduo::Thread receive_task(std::bind(&RobotInterface::eventLoopThread, &robotInterface));
    muduo::Thread execute_task(std::bind(&RobotInterface::execTaskThread, &robotInterface));

    receive_task.start();
    execute_task.start();

    receive_task.join();
    execute_task.join();

    return 0;
}