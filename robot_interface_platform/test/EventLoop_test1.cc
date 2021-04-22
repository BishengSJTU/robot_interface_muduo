#include "EventLoop.h"
#include "Thread.h"
#include "Logging.h"
#include <stdio.h>

void threadFunc() {
    printf("threadFunc(): pid = %d, tid = %d\n",
           getpid(), muduo::CurrentThread::tid());

    muduo::net::EventLoop loop;
    loop.loop();
}

int main() {
    int a = 11111;
    LOG_DEBUG << a;
    printf("main(): pid = %d, tid = %d\n",
           getpid(), muduo::CurrentThread::tid());

    muduo::net::EventLoop loop;

    muduo::Thread thread(threadFunc);
    thread.start();

    loop.loop();
    pthread_exit(NULL);
}
