#include "EventLoop.h"
#include "Thread.h"

muduo::net::EventLoop* g_loop;

void threadFunc()
{
  g_loop->loop();
}

int main()
{
  muduo::net::EventLoop loop;
  g_loop = &loop;
  muduo::Thread t(threadFunc);
  t.start();
  t.join();
}
