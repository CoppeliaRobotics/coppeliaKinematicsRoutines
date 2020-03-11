#include "serialization.h"

CSerialization::CSerialization(const unsigned char* data,size_t dataSize)
{
    _buffer=data;
    _bufferSize=dataSize;
    _readPos=0;
}

CSerialization::~CSerialization()
{
}

unsigned char CSerialization::readByte()
{
    return(_buffer[_readPos++]);
}

int CSerialization::readInt()
{
    unsigned char tmp[sizeof(int)];
    for (size_t i=0;i<sizeof(int);i++)
        tmp[i]=_buffer[_readPos++];
    return((reinterpret_cast<int*>(&tmp))[0]);
}

float CSerialization::readFloat()
{
    unsigned char tmp[sizeof(float)];
    for (size_t i=0;i<sizeof(float);i++)
        tmp[i]=_buffer[_readPos++];
    return((reinterpret_cast<float*>(&tmp))[0]);
}

std::string CSerialization::readString()
{
    std::string retVal;
    while (_buffer[_readPos]!=0)
        retVal+=char(_buffer[_readPos++]);
    _readPos++;
    return(retVal);
}
