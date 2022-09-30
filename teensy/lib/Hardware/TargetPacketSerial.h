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
#include "messageprocessor.h"

using namespace std;

class TargetPacketSerial : public PacketSerial, public virtual IPacketSerial {

public:

    TargetPacketSerial(Stream* serial) {
        setStream(serial);
    }

    size_t available() override {
        return incoming.size();
    }

    bool empty() override {
        return incoming.empty();
    }

    void pop() override {
        log("TargetPacketSerial: pop");
        incoming.pop_front();
    }

    MessageBuffer &front() override {
        log("TargetPacketSerial: read front");
        return incoming.front();
    }

    // Outgoing send.
    void send(const uint8_t *buffer, size_t size)  override {
        outgoing.emplace_back(buffer, buffer+size);
    }


    void update() override {

        PacketSerial::update();

        if (!outgoing.empty()){
            auto b = outgoing.front();
            PacketSerial::send(b.data(), b.size());
            outgoing.pop_front();
        }
    }

    void handlePacket(const uint8_t *buffer, size_t size) override {
        incoming.emplace_back(buffer, buffer+size);
        logf("Incoming size %d", incoming.size());
    }


public:

    bool iqEmpty() const{ return incoming.empty(); }

    deque<MessageBuffer> outgoing;
    deque<MessageBuffer> incoming;



};
