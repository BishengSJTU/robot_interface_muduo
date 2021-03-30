#include "robot.h"
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
    g_logFile.reset(new muduo::LogFile(::basename(name), 600*1000));
    muduo::Logger::setOutput(outputFunc);

    Robot robot;
    muduo::Thread receive_task(std::bind(&Robot::eventLoopThread, &robot));
    muduo::Thread execute_task(std::bind(&Robot::execTaskThread, &robot));

    receive_task.start();
    execute_task.start();

    receive_task.join();
    execute_task.join();

    return 0;
}