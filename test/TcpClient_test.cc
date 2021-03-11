#include "EventLoop.h"
#include "InetAddress.h"
#include "TcpClient.h"
#include <stdio.h>


void onConnection(const muduo::net::TcpConnectionPtr &conn) {
  if (conn->connected()) {
    printf("onConnection(): new connection [%s] from %s\n",
           conn->name().c_str(), conn->peerAddress().toIpPort().c_str());
  } else {
      printf("onConnection(): connection [%s] is down\n", conn->name().c_str());
  }
}

void onMessage(const muduo::net::TcpConnectionPtr &conn, muduo::net::Buffer *buf,
               muduo::Timestamp receiveTime) {
  printf("onMessage(): received %zd bytes from connection [%s] at %s\n",
         buf->readableBytes(), conn->name().c_str(),
         receiveTime.toFormattedString().c_str());

  // buf->retrieveAsString() 读取所有的消息
  printf("onMessage(): [%s]\n", buf->retrieveAllAsString().c_str());
}

int main() {
    printf("main(): pid = %d\n", getpid());
    muduo::net::EventLoop loop;
    muduo::net::InetAddress serverAddr("127.0.0.1", 9981);
    muduo::net::TcpClient client(&loop, serverAddr, "TcpClient");
    client.setConnectionCallback(onConnection);
    client.setMessageCallback(onMessage);
    client.enableRetry(); // 断线重连
    client.connect();
    printf("conn...\n");
    loop.loop();
}