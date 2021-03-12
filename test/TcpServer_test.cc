#include "EventLoop.h"
#include "InetAddress.h"
#include "TcpServer.h"
#include <stdio.h>
#include <iostream>

void onConnection(const muduo::net::TcpConnectionPtr &conn) {
    if (conn->connected()) {
        printf("onConnection(): new connection [%s] from %s\n",
               conn->name().c_str(), conn->peerAddress().toIpPort().c_str());
    } else {
        printf("onConnection(): connection [%s] is down\n", conn->name().c_str());
    }
    char* data = new char[9];
    data[0] = 0x00;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = 0x00;
    data[5] = 0x00;
    data[6] = 0x01;
    data[7] = 0x0a;
    data[8] = 0xff;
    conn->send(data, 9);
}

void onMessage(const muduo::net::TcpConnectionPtr &conn, muduo::net::Buffer *buf,
               muduo::Timestamp receiveTime) {
//    printf("onMessage(): received %zd bytes from connection [%s] at %s\n",
//           buf->readableBytes(), conn->name().c_str(),
//           receiveTime.toFormattedString().c_str());

    std::cout << "onMessage():";
    muduo::string message = buf->retrieveAllAsString();
    const char* data = message.c_str();
    for(auto i = 0; i < message.size(); i++) {
        std::cout << std::hex << (unsigned int)(unsigned char)data[i] << ",";
    }
    std::cout << std::endl;

}

int main() {
    printf("main(): pid = %d\n", getpid());

    muduo::net::InetAddress listenAddr(9981);
    muduo::net::EventLoop loop;

    muduo::net::TcpServer server(&loop, listenAddr, "TcpServer");
    server.setConnectionCallback(onConnection);
    server.setMessageCallback(onMessage);
    server.start();

    loop.loop();
}