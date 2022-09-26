
#include <stdarg.h>

#include "trj_types.h"
#include "FastCRC.h"
#include "PacketSerial.h"
#include "messageprocessor.h"

FastCRC8 CRC8;

char printf_buffer[2048];


MessageProcessor::MessageProcessor(IPacketSerial &ps) : ps(ps) {}

void MessageProcessor::update() {
    ps.update();

    if(!ps.empty()){ //
        processPacket(ps.front().data(), ps.front().size());
        ps.pop();
    }
}

void MessageProcessor::updateAll() {
    while(!ps.empty()){
        update();
    }
}

void MessageProcessor::setLastSegNum(int v) {
    lastSegNum = v;
}

// print to the print buffer
void MessageProcessor::printf(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vsprintf(printf_buffer, fmt, args);
    va_end(args);
    sendMessage(printf_buffer);
}

uint8_t MessageProcessor::crc(size_t length) {
    auto *ph = (PacketHeader *) buffer;
    ph->crc = 0;
    ph->crc = CRC8.smbus(buffer, length);
    return ph->crc;
}

void MessageProcessor::send(size_t length) {
    ps.send((const uint8_t *) buffer, length + sizeof(PacketHeader));
}

void MessageProcessor::send(CommandCode code, uint16_t seq, size_t length) {
    auto *ph = (PacketHeader *) &buffer;
    ph->code = code;
    ph->seq = seq;
    crc(length + sizeof(PacketHeader));
    send(length);
}

void MessageProcessor::send(const uint8_t *payload, CommandCode code, uint16_t seq, size_t length) {
    memcpy(buffer + sizeof(PacketHeader), payload, length > MAX_PAYLOAD_SIZE ? MAX_PAYLOAD_SIZE : length);
    send(code, seq, length);

}

void MessageProcessor::sendMessage(const char *message_) {
    send((const uint8_t *) message_, CommandCode::MESSAGE, lastSegNum, strlen(message_));
}

void MessageProcessor::sendEmpty(uint16_t seq, CurrentState &current_state) {
    send((const uint8_t *) &current_state, CommandCode::EMPTY, seq, sizeof(current_state));
}

void MessageProcessor::sendDone(uint16_t seq, CurrentState &current_state) {
    send((const uint8_t *) &current_state, CommandCode::DONE, seq, sizeof(current_state));
}

void MessageProcessor::sendNack() {
    send(CommandCode::NACK, lastSegNum, 0);
}

void MessageProcessor::sendAck(uint16_t seq, CurrentState &current_state) {
    send((const uint8_t *) &current_state, CommandCode::ACK, seq, sizeof(current_state));
}

// An ACK with no current state
void MessageProcessor::sendAck(uint16_t seq) {
    send(CommandCode::ACK, seq, 0);
}

// Mostly for testing
void MessageProcessor::processPacket(PacketHeader ph, char *payload, size_t payload_size) {
    processPacket(&ph, (const uint8_t *)payload, payload_size);
}
void MessageProcessor::processPacket(PacketHeader *ph, const uint8_t *payload, size_t payload_size) {

    if (ph->code == CommandCode::NOOP) {
        return;
    } else if (ph->code == CommandCode::ECHO) {
        send(payload, ph->code, ph->seq, payload_size);
        return; // Echos are their own ack.
    }

    messages.emplace_back(*ph, (char *) payload, payload_size);
    sendAck(ph->seq);
}



void MessageProcessor::processPacket(const uint8_t *buffer_, size_t size) {

    auto *ph = (PacketHeader *) buffer_;

    auto payload = (const uint8_t *) (((const uint8_t *) buffer_) + sizeof(PacketHeader));
    size_t payload_size = size - sizeof(PacketHeader);

    uint8_t that_crc = ph->crc;
    crc(size); // Calc crc on buffer, put back into ph. 
    if (that_crc != ph->crc) {
        sendNack();
        return;
    }

    processPacket(ph, payload, payload_size);
}



void MessageProcessor::pop() {
    messages.pop_front();
}

Message& MessageProcessor::firstMessage() {
    return  messages.front();
}

int MessageProcessor::availableMessages() {
    return messages.size();
}

bool MessageProcessor::empty() {
    return messages.empty();
}
