#ifndef CODEC_H
#define CODEC_H

#include "Logging.h"
#include "Buffer.h"
#include "Endian.h"
#include "TcpConnection.h"

class LengthHeaderCodec : muduo::noncopyable {
public:
    typedef std::function<void(const muduo::net::TcpConnectionPtr &,
                               const muduo::string &message,
                               muduo::Timestamp)> CompleteMessageCallback;

    explicit LengthHeaderCodec(const CompleteMessageCallback &cb)
            : messageCallback_(cb) {
    }

    void onMessage(const muduo::net::TcpConnectionPtr &conn,
                   muduo::net::Buffer *buf,
                   muduo::Timestamp receiveTime) {
        while (buf->readableBytes() >= kHeaderLen) // kHeaderLen == 7
        {
            const char *data = buf->peek();
            const int32_t len = data[5] * 256 + data[6];
            if (len > 65536 || len < 0) {
                LOG_ERROR << "Invalid length " << len;
                conn->shutdown();
                break;
            } else if (buf->readableBytes() >= len + kHeaderLen) {
                muduo::string message(buf->peek(), len + kHeaderLen);
                messageCallback_(conn, message, receiveTime);
                buf->retrieve(kHeaderLen);
                buf->retrieve(len);
            } else {
                break;
            }
        }
    }

    void send(muduo::net::TcpConnection *conn,
              const muduo::StringPiece &message) {
        muduo::net::Buffer buf;
        buf.append(message.data(), message.size());
        conn->send(&buf);
    }

private:
    CompleteMessageCallback messageCallback_;
    const static size_t kHeaderLen = 7; // 固定头部字节有７位
};

#endif
