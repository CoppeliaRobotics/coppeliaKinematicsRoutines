#include "serialization.h"

CSerialization::CSerialization()
{
    _writing=true;
}

CSerialization::CSerialization(const unsigned char* data,size_t dataSize)
{
    _writing=false;
    _buffer=data;
    _bufferSize=dataSize;
    _readPos=0;
}

CSerialization::~CSerialization()
{
}

bool CSerialization::isWriting() const
{
    return(_writing);
}

void CSerialization::writeFloat(float a)
{
    _writeBuffer.push_back(((unsigned char*)&a)[0]);
    _writeBuffer.push_back(((unsigned char*)&a)[1]);
    _writeBuffer.push_back(((unsigned char*)&a)[2]);
    _writeBuffer.push_back(((unsigned char*)&a)[3]);
}

void CSerialization::writeInt(int a)
{
    _writeBuffer.push_back(((unsigned char*)&a)[0]);
    _writeBuffer.push_back(((unsigned char*)&a)[1]);
    _writeBuffer.push_back(((unsigned char*)&a)[2]);
    _writeBuffer.push_back(((unsigned char*)&a)[3]);
}

void CSerialization::writeUInt(unsigned int a)
{
    _writeBuffer.push_back(((unsigned char*)&a)[0]);
    _writeBuffer.push_back(((unsigned char*)&a)[1]);
    _writeBuffer.push_back(((unsigned char*)&a)[2]);
    _writeBuffer.push_back(((unsigned char*)&a)[3]);
}

void CSerialization::writeWord(unsigned short a)
{
    _writeBuffer.push_back(((unsigned char*)&a)[0]);
    _writeBuffer.push_back(((unsigned char*)&a)[1]);
}

void CSerialization::writeByte(unsigned char a)
{
    _writeBuffer.push_back(a);
}

void CSerialization::writeChar(char a)
{
    _writeBuffer.push_back((unsigned char)a);
}

void CSerialization::writeString(const char* string)
{
    for (int i=0;i<int(strlen(string));i++)
        _writeBuffer.push_back((unsigned char)string[i]);
    _writeBuffer.push_back(0);
}

unsigned char* CSerialization::getWriteBuffer(size_t& bufferLength)
{
    bufferLength=_writeBuffer.size();
    return(&_writeBuffer[0]);
}

size_t CSerialization::getReadPos() const
{
    return(_readPos);
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
