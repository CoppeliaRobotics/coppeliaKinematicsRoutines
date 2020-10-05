#pragma once

#include "ik.h"
#include <string>

class CSerialization
{
public:
    CSerialization(); // writing
    CSerialization(const unsigned char* data,size_t dataSize); // reading
    ~CSerialization();

    bool isWriting() const;

    void writeFloat(float a);
    void writeInt(int a);
    void writeUInt(unsigned int a);
    void writeWord(unsigned short a);
    void writeByte(unsigned char a);
    void writeChar(char a);
    void writeString(const char* string);
    unsigned char* getWriteBuffer(size_t& bufferLength);

    size_t getReadPos() const;
    unsigned char readByte();
    int readInt();
    float readFloat();
    std::string readString();

private:
    bool _writing;
    std::vector<unsigned char> _writeBuffer;

    const unsigned char* _buffer;
    size_t _bufferSize;
    size_t _readPos;
};
