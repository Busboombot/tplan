#pragma once

#include <cstdint>
#include <limits.h>
#include <deque>
#include <vector>
#include "trj_const.h"

#include <iostream>

using namespace std;

enum class MoveType {
    none,
    relative,
    absolute,
    jog,
    home
};

using MoveArray = std::vector<int32_t>;


typedef enum {
    CCW = -1,  ///< Clockwise
    STOP = 0,  ///< Clockwise
    CW = 1   ///< Counter-Clockwise

} Direction;


/* Enumeration that describe the contents of a command message */
enum class CommandCode : uint8_t {

    ACK = 1,
    NACK = 2,
    DONE = 3,  // A Movement command is finished
    EMPTY = 4, // Queue is empty, nothing to do.

    RMOVE = 11,  // A relative movement segment, with just the relative distance.
    AMOVE = 12,  // An absolute movement
    JMOVE = 13,   // A Jog movement.
    HMOVE = 14,   // A Home movement. Move to the next limit


    RUN = 21,
    STOP = 22,
    RESET = 23,  //
    ZERO = 24,  // Zero positions
    CONFIG = 25, // Reset the configuration
    AXES = 26, // Configure an axis


    MESSAGE = 91,  // Payload is a message; the next packet is text
    ERROR = 92,  // Some error
    ECHO = 93,  // Echo the incomming header
    DEBUG = 94,  //
    INFO = 95, // Return info messages

    NOOP = 99,  // Does nothing, but get ACKED

    POSITIONS = 101,  // Position report. ( Unused)

};

/* Header on every command packet */
struct PacketHeader {
    uint16_t seq;       // Packet sequence number
    CommandCode code;   // Command code
    uint8_t crc = 0;    // Payload CRC8

    PacketHeader() : seq(0), code(CommandCode::NOOP), crc(0) {}

    PacketHeader(uint16_t seq, CommandCode code) : seq(seq), code(code), crc(0) {}

    explicit PacketHeader(char *buffer) : seq(*(uint16_t *) buffer), code(*(CommandCode *) (buffer + sizeof(uint16_t))),
                                          crc(0) {
    }

};  // 4

// Payload of the move command
struct Moves {
    uint32_t segment_time = 0; // total segment time, in microseconds // 4
    int32_t x[N_AXES];
}; // 8

/**
 * @brief Track the current conditions for the queue and positions, as of the time
 * of ACKs and DONE messages. 
 * 
 */

struct CurrentState {
    int32_t queue_length = 0;
    uint32_t queue_time = 0;
    int32_t positions[N_AXES] = {0};
    int32_t planner_positions[N_AXES] = {0};
};

// Configuration record for one axis
// 8 Bytes
struct AxisConfig {

    uint8_t axis;           // Axis number

    uint8_t step_pin;       // Step output, or quadture b
    uint8_t direction_pin;  // Direction output, or quadrature b
    uint8_t enable_pin;

    uint8_t step_high_value; // Whether step is HIGH or LOW when enabled. 
    uint8_t direction_high_value;
    uint8_t enable_high_value;

    uint8_t step_output_mode; // OUTPUT or OUTPUT_OPEN_DRAIN
    uint8_t direction_output_mode;
    uint8_t enable_output_mode;

    uint8_t pad1;
    uint8_t pad2;

    uint32_t v_max;
    uint32_t a_max;
};

// Main Configuration class
struct Config {

    uint8_t n_axes = 0;         // Number of axes
    uint8_t interrupt_delay = INTERRUPT_DELAY;    // How often interrupt is called, in microseconds
    uint8_t segment_complete_pin = 0; // Pin on which to signal that a segment is complete
    uint8_t limit_pin = 0; // Pin to recieve signals that the encoder foind a limit
    bool debug_print = true;
    bool debug_tick = true;
};


typedef struct Message {
    PacketHeader header;
    vector<char> buffer;

    Message() {}

    // Construct a message with a seperate payload and header.
    Message(PacketHeader ph, char *buffer_, size_t size) : header(ph), buffer(buffer_, buffer_ + size) {
        buffer.push_back('\0');
    };

    // Header is the first part of the payload
    Message(char *buffer_, size_t size) :
            header(buffer_),
            buffer((char *) (buffer_ + sizeof(PacketHeader)), (char *) (buffer_ + size)) {}

    Message(vector<uint8_t> &v) : Message( (char*)v.data(), v.size()){}


    Config *asConfig() { return (Config *) &buffer[0]; }

    AxisConfig *asAxisConfig() { return (AxisConfig *) &buffer[0]; }

    Moves *asMoves() { return (Moves *) &buffer[0]; }

    string asString() { return string((char *) &buffer[0]); }

    friend ostream &operator<<(ostream &output, const Message &s);

} Message;

