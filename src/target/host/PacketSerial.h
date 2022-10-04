#pragma once
#include <deque>
#include <iostream>
#include <vector>
#include "types.h"
#include "ipacketserial.h"
#include "FastCRC.h"

extern FastCRC8 CRC8;

using namespace std;

typedef void (*PacketHandlerFunctionWithSender)(const void* sender, const uint8_t* buffer, size_t size);



class MockPacketSerial : public virtual IPacketSerial {

public:

    MockPacketSerial() {}

    size_t available() override {
        return incoming.size();
    }

    bool empty() override {
        return incoming.empty();
    }

    void pop() override {
        incoming.pop_front();
    }

    vector<uint8_t> &front() override {
        return incoming.front();
    }

    // Outgoing send.
    void send(const uint8_t *buffer, size_t size)  override {
        outgoing.emplace_back(buffer, buffer+size);
    }

    // Incoming send, push a message to simulate incomming messages.
    void push(const uint8_t *buffer, size_t size)  {
        incoming.emplace_back(buffer, buffer+size);

    }

    uint8_t crc(const uint8_t *buffer, size_t length) {
        auto *ph = (PacketHeader *) buffer;
        ph->crc = 0;
        ph->crc = CRC8.smbus(buffer, length);
        return ph->crc;
    }

    void push(PacketHeader ph, char *payload, size_t payload_size){

        uint8_t buffer[MESSAGE_BUF_SIZE];
        memcpy(buffer, &ph, sizeof(ph));
        memcpy(((char*)buffer)+(size_t)sizeof(ph), payload, payload_size);
        crc(buffer,payload_size+sizeof(ph));

        push(buffer, payload_size+sizeof(ph));
    }

    void update() override {

    }

public:

    bool iqEmpty(){ return incoming.empty(); }

    deque<vector<uint8_t>> outgoing;
    deque<vector<uint8_t>> incoming;

};
