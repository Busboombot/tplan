#pragma once

#include <functional>
#include <deque>
#include <vector>
#include <sstream>
#include <string>

#include "const.h"
#include "types.h"

#include "ipacketserial.h"

using namespace std;


class MessageProcessor {



public:

    MessageProcessor(IPacketSerial &ps);

    void update(tmicros t, CurrentState &current_state);

    void updateAll(tmicros t, CurrentState &current_state);

    void updateCurrentState(CurrentState &current_state);

    void setLastSegNum(int v);

    void sendAck(uint16_t seq);

    void sendAlive();

    void sendNack();

    void sendDone(uint16_t seq);

    void sendEmpty(uint16_t seq);

    // Send a text message
    void sendMessage(const char *message_);

    void sendMessage(const string &str);

    void sendMessage(stringstream &ss);

    void printf(const char *fmt, ...);

    void log(const char *message_);

    void log(const string &str);

    void log(stringstream &ss);

    void logf(const char *fmt, ...);

    // Remove a message from the queue
    void pop();

    Message &firstMessage();

    int availableMessages();

    bool empty();

    // Perform operation for each type of message recieved
    void processPacket(const MessageBuffer &messageBuffer);

    void processPacket(PacketHeader *ph, const uint8_t *payload, size_t payload_size);

    void processPacket(PacketHeader ph, char *payload, size_t payload_size);

private:

    uint8_t crc(const uint8_t *buffer, size_t length);

    void send(size_t length);

    void send(CommandCode code, uint16_t seq, size_t length);

    void send(const uint8_t *payload, CommandCode code, uint16_t seq, size_t length);


private:

    IPacketSerial &ps;

    uint8_t buffer[MESSAGE_BUF_SIZE]; // Outgoing message buffer

    uint16_t last_seq = 0;

    std::deque<Message> messages;

    CurrentState current_state;

    tmicros last_alive = 0;
    tmicros last_time = 0;


};

// Singleton message proces for logging on the teensy.
// THe logging functions will use this mp, if it is set
extern MessageProcessor *message_processor;

void log(const char *str);

void log(const string &str);

void log(stringstream &str);

void logf(const char *fmt, ...);


