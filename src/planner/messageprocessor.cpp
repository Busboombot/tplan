#include <cstring>
#include <cstdarg>
#include <string>
#include <vector>
#include <cstdio>


#ifdef TRJ_ENV_HOST

#include <iostream>

#endif

#ifdef TRJ_ENV_ARDUINO
#include "Arduino.h"
#endif

#include "types.h"
#include "messageprocessor.h"
#include "util.h"
#include "FastCRC.h"

using namespace std;

char printf_buffer[5000];
FastCRC8 CRC8;

// Singleton message proces for logging on the teensy.
// This should be assigned in the module where the MessageProcessor,
// usually in main()
MessageProcessor *message_processor = nullptr;


void log(const char *str) {
    if (message_processor != nullptr) {
        message_processor->log(str);
    }
}

void log(const string &str) {
    if (message_processor != nullptr) {
        message_processor->log(str);
    }
}

void log(stringstream &str) {
    if (message_processor != nullptr) {
        message_processor->log(str);
    }
}

void logf(const char *fmt, ...) {

    if (message_processor != nullptr) {

        va_list args;
        va_start(args, fmt);
        vsprintf(printf_buffer, fmt, args);
        va_end(args);

        message_processor->log(printf_buffer);
    }

}

MessageProcessor::MessageProcessor(IPacketSerial &ps) : ps(ps) {}

void MessageProcessor::updateCurrentState(CurrentState &current_state_) {
    current_state = current_state_;
}

void MessageProcessor::update(tmillis t, CurrentState &current_state) {

    updateCurrentState(current_state);

    ps.update();

    if (!ps.empty()) {
        processPacket(ps.front());
        ps.pop();
    }
}

void MessageProcessor::updateAll(tmillis t, CurrentState &current_state) {
    while (!ps.empty()) {
        update(t, current_state);
    }
}

void MessageProcessor::setLastSegNum(int v) {
    lastSegNum = v;
}


uint8_t MessageProcessor::crc(const uint8_t *buffer, size_t length) {
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
    crc(buffer, length + sizeof(PacketHeader));
    send(length);
}

void MessageProcessor::send(const uint8_t *payload, CommandCode code, uint16_t seq, size_t length) {
    memcpy(buffer + sizeof(PacketHeader), payload, length > MAX_PAYLOAD_SIZE ? MAX_PAYLOAD_SIZE : length);
    send(code, seq, length);

}

void MessageProcessor::sendEmpty(uint16_t seq) {
    send((const uint8_t *) &current_state, CommandCode::EMPTY, seq, sizeof(current_state));
}

void MessageProcessor::sendDone(uint16_t seq) {
    send((const uint8_t *) &current_state, CommandCode::DONE, seq, sizeof(current_state));
}

void MessageProcessor::sendNack() {
    send(CommandCode::NACK, lastSegNum, 0);
}

void MessageProcessor::sendAck(uint16_t seq) {
    logf("ACK %d", seq);
    send((const uint8_t *) &current_state, CommandCode::ACK, seq, sizeof(current_state));
}

// Mostly for testing
void MessageProcessor::processPacket(PacketHeader ph, char *payload, size_t payload_size) {
    processPacket(&ph, (const uint8_t *) payload, payload_size);
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


void MessageProcessor::processPacket(const MessageBuffer &mb) {

    auto *ph = (PacketHeader *) mb.data();
    auto payload = (const uint8_t *) ( mb.data() + sizeof(PacketHeader));
    size_t payload_size = mb.size() - sizeof(PacketHeader);

    uint8_t that_crc = ph->crc;
    crc(mb.data(), mb.size()); // Calc crc on buffer, put back into ph. Note that crc() changes data in place, which is bad.
    if (that_crc != ph->crc) {
        sendNack();
        return;
    }

    processPacket(ph, payload, payload_size);
}

void MessageProcessor::sendMessage(const string &str) {

#ifdef TRJ_ENV_HOST
    cout << "|| " << str;
#else
    send((const uint8_t *) str.data(), CommandCode::MESSAGE, lastSegNum, str.size());
#endif


}

void MessageProcessor::sendMessage(const char *message_) {
    sendMessage(string(message_));
}


/**
 * Send a string stream as a message, breaking it into lines
 * @param ss
 */
void MessageProcessor::sendMessage(stringstream &ss) {

    string line;
    while (getline(ss, line, '\n')) {
        sendMessage(line + '\n');
    }
}

// print to the print buffer, or to cout, or to the debug
// serial if it is attached.
void MessageProcessor::printf(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vsprintf(printf_buffer, fmt, args);
    va_end(args);

    sendMessage(printf_buffer);
}


void MessageProcessor::log(const string &str) {

#ifdef TRJ_DEBUG
#ifdef TRJ_DEBUG_SERIAL
    TRJ_DEBUG_SERIAL.print((str+"\r\n").c_str());
#elif defined(TRJ_ENV_HOST)
    send((const uint8_t *) str.data(), CommandCode::MESSAGE, lastSegNum, str.size());
#endif
#endif


}

void MessageProcessor::log(const char *message_) {
    log(string(message_));
}


/**
 * Send a string stream as a message, breaking it into lines
 * @param ss
 */
void MessageProcessor::log(stringstream &ss) {

    if(ss.str().find('\n') == std::string::npos){
        log(ss.str());
    } else {

        string line;
        while (getline(ss, line, '\n')) {
            log(line);
        }
    }
}

// print to the print buffer, or to cout, or to the debug
// serial if it is attached.
void MessageProcessor::logf(const char *fmt, ...) {

    va_list args;
    va_start(args, fmt);
    vsprintf(printf_buffer, fmt, args);
    va_end(args);

    log(printf_buffer);
}



void MessageProcessor::pop() {
    messages.pop_front();
}

Message &MessageProcessor::firstMessage() {
    return messages.front();
}

int MessageProcessor::availableMessages() {
    return messages.size();
}


bool MessageProcessor::empty() {
    return messages.empty();
}
