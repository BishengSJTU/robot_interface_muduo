#include "LogFile.h"
#include "Logging.h"

#include <unistd.h>

std::unique_ptr<muduo::LogFile> g_logFile;

void outputFunc(const char* msg, int len)
{
  g_logFile->append(msg, len);
}

void flushFunc()
{
  g_logFile->flush();
}

int main(int argc, char* argv[])
{
  char name[256] = { '\0' };
  strncpy(name, argv[0], sizeof name - 1);
  printf("name:%s", ::basename(name));
  g_logFile.reset(new muduo::LogFile(::basename(name), 200*1000));
  muduo::Logger::setOutput(outputFunc);
  muduo::Logger::setFlush(flushFunc);

  muduo::string line = "ABCDEFGHIJKLMNOPQRSTUVWXYZ ";

  for (int i = 0; i < 10000; ++i)
  {
    LOG_INFO << line;

    usleep(1000);
  }
    while(1);
}
