#pragma once

#include "ik.h"
#include <string>

class CSerialization
{
public:

    CSerialization(const unsigned char* data,size_t dataSize);
    ~CSerialization();

    unsigned char readByte();
    int readInt();
    float readFloat();
    std::string readString();

private:
    const unsigned char* _buffer;
    size_t _bufferSize;
    size_t _readPos;
};
