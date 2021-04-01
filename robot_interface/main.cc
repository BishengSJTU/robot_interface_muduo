#include "robot_interface.h"
#include "Thread.h"
#include "LogFile.h"
#include "Logging.h"
#include "robot_client_tcp.h"

std::unique_ptr<muduo::LogFile> g_logFile;
std::unique_ptr<muduo::LogFile> g_logStdout;
char name[256] = { '\0' };

void outputFunc(const char* msg, int len)
{
    g_logFile->append(msg, len);
    g_logFile->flush();
}

int main(int argc, char* argv[]) {
    RobotClient robotClient;
    robotClient.InitializeRobot("10.5.5.100", 10001);
    std::vector<float> jointA = {80,92,100,165,170,-183};
    std::vector<float> jointB = {53,60,123,172,144,-184};
    std::vector<float> jointC = {-27,85,106,167,63,-181};
    while(1) {
        robotClient.MoveJ(jointA, 20);
        std::cout << "1" << std::endl;
        robotClient.MoveJ(jointB, 20);
        std::cout << "2" << std::endl;
        robotClient.MoveJ(jointC, 20);
        std::cout << "3" << std::endl;
        robotClient.Jog(0, 20, 20);
        std::cout << "4" << std::endl;
        robotClient.Jog(0, 20, -20);
        std::cout << "5" << std::endl;
    }

    strncpy(name, argv[0], sizeof name - 1);
    g_logFile.reset(new muduo::LogFile(::basename(name), 600*1000));
    muduo::Logger::setOutput(outputFunc);

    RobotInterface robotInterface;
    muduo::Thread receive_task(std::bind(&RobotInterface::eventLoopThread, &robotInterface));
    muduo::Thread execute_task(std::bind(&RobotInterface::execTaskThread, &robotInterface));

    receive_task.start();
    execute_task.start();

    receive_task.join();
    execute_task.join();

    return 0;
}