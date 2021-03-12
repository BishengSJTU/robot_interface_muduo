#include "robot.h"
#include "Thread.h"

int main() {
    Robot robot;
    muduo::Thread receive_task(std::bind(&Robot::eventLoopThread, &robot));
    muduo::Thread execute_task(std::bind(&Robot::execTaskThread, &robot));

    receive_task.start();
    execute_task.start();

    receive_task.join();
    execute_task.join();
    return 0;
}