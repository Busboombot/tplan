#pragma once

class Stream {

public:
    Stream(){}

};

typedef void (*PacketHandlerFunctionWithSender)(const void* sender, const uint8_t* buffer, size_t size);

class PacketSerial {

public:

    PacketSerial() {}

    void setStream(Stream *stream){};

    void setPacketHandler(PacketHandlerFunctionWithSender onPacketFunction){}

    void send(const uint8_t* buffer, size_t size) const{}

    void update(){}
};