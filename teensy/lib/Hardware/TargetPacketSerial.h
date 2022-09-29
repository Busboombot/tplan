#pragma once
#include "Arduino.h"
#include <deque>
#include <iostream>
#include <vector>
#include <cstring>
#include <cstdarg>
#include <cstdio>
#include "types.h"
#include "ipacketserial.h"
#include "PacketSerial.h"

using namespace std;

class TargetPacketSerial : public PacketSerial, public virtual IPacketSerial {

public:

    TargetPacketSerial() {
        setStream(&Serial);

        PacketSerial::setPacketHandler([](const void* sender, const uint8_t* buffer, size_t size) {
            ((TargetPacketSerial*)sender)->push(buffer,size);
        });

    }

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

    void push(PacketHeader ph, char *payload, size_t payload_size){
        uint8_t buffer[MESSAGE_BUF_SIZE];
        memcpy(buffer, &ph, sizeof(ph));
        memcpy(((char*)buffer)+(size_t)sizeof(ph), payload, payload_size);

        push(buffer, payload_size+sizeof(ph));
    }

    void update() override {
        PacketSerial::update();
        if (!outgoing.empty()){
            auto b = outgoing.front();
            PacketSerial::send(b.data(), b.size());
            outgoing.pop_front();
        }
    }

public:

    bool iqEmpty(){ return incoming.empty(); }

    deque<vector<uint8_t>> outgoing;
    deque<vector<uint8_t>> incoming;

};
