#include "Buffer.h"
#include "Endian.h"
#include <iostream>
int main()
{
    int32_t len = static_cast<int32_t>(1);
    int32_t be32 = muduo::net::sockets::hostToNetwork32(len);
    std::cout << be32 << std::endl;
}