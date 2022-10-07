#pragma once

#include <cstdint>
#include <stdint.h>
#include <climits>
#include <deque>
#include <vector>
#include <array>
#include <iostream>
#include <bitset>
#include "const.h"
using namespace std;

enum class MoveType {
    none,
    relative,
    absolute,
    jog,
    home,
    velocity
};

using AxisPos = int32_t;
using MoveVector = std::vector<AxisPos>;
using MoveArray = array<AxisPos,N_AXES>;


MoveVector& operator-=(MoveVector &a, const MoveArray &b);
MoveVector& operator+=(MoveVector &a, const MoveArray &b);
MoveArray& operator+=(MoveArray &a, const MoveVector &b);
MoveArray& operator-=(MoveArray &a, const MoveVector &b);
MoveArray& operator+=(MoveArray &a, const MoveArray &b);
MoveArray& operator-=(MoveArray &a, const MoveArray &b);


ostream &operator<<( ostream &output, const MoveVector &m );
ostream &operator<<( ostream &output, const MoveArray &m );

using tmillis = uint32_t;
using tmicros = uint32_t;
using Pin = uint8_t;
using PinVal = uint8_t;
using AxisArray = std::array<PinVal , N_AXES>;

using MessageBuffer = vector<uint8_t>;


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
    VMOVE = 15,   // Velocity move.

    RUN = 21,
    STOP = 22,
    RESET = 23,  //
    ZERO = 24,  // Zero positions
    SET = 25, // Set planner positions

    CONFIG = 31, // Reset the configuration
    AXES = 32, // Configure an axis

    MESSAGE = 91,  // Payload is a message; the next packet is text
    ERROR = 92,  // Some error
    ECHO = 93,  // Echo the incomming header
    DEBUG = 94,  //
    INFO = 95, // Return info messages
    QUEUE = 96, // Print out the queue
    SYNC = 97, // Print out the queue
    ALIVE = 98,  // Step controller tells client it is still alive.
    NOOP = 99,  // Does nothing, but get ACKED

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
    int32_t x[N_AXES] = {0};

    friend ostream &operator<<( ostream &output, const Moves &m );
}; // 8


/**
 * @brief Track the current conditions for the queue and positions, as of the time
 * of ACKs and DONE messages.
 *
 */

enum class CSFLags : size_t {
    RUNNING = 0,
    EMPTY = 1
};


struct CurrentState {
    int32_t queue_length = 0;
    uint32_t queue_time = 0;
    int32_t positions[N_AXES] = {0};
    int32_t planner_positions[N_AXES] = {0};
    bitset<32> flags;

    explicit CurrentState():  queue_length(0), queue_time(0){
        for(int i=0; i< N_AXES; i++){
            positions[i] = planner_positions[i] =  0;
        }
    }

    CurrentState(int32_t queueLength, uint32_t queueTime, vector<int32_t> positions_,  vector<int32_t> ppositions_) :
            queue_length(queueLength), queue_time(queueTime)
    {

        for(size_t i=0; i< N_AXES; i++){
            positions[i] = (i<=positions_.size()) ? positions_[i] : 0;
            planner_positions[i] = (i<=ppositions_.size()) ? ppositions_[i] : 0;
        }
    }

    friend ostream &operator<<( ostream &output, const CurrentState &c );
};





// Configuration record for one axis
// 8 Bytes
typedef struct AxisConfig {

    uint8_t axis=0;           // Axis number

    uint8_t step_pin=0;       // Step output, or quadture b
    uint8_t direction_pin=0;  // Direction output, or quadrature b
    uint8_t enable_pin=0;

    uint8_t step_high_value=0; // Whether step is HIGH or LOW when enabled.
    uint8_t direction_high_value=0;
    uint8_t enable_high_value=0;

    uint8_t step_output_mode=0; // OUTPUT or OUTPUT_OPEN_DRAIN
    uint8_t direction_output_mode=0;
    uint8_t enable_output_mode=0;

    uint8_t pad1=0xBE;
    uint8_t pad2=0xEF;

    uint32_t v_max=0;
    uint32_t a_max=0;

    friend ostream &operator<<( ostream &output, const AxisConfig &ac );

} AxisConfig;

// Main Configuration class
typedef struct Config {

    uint8_t n_axes = 0;         // Number of axes
    uint8_t interrupt_delay = INTERRUPT_DELAY;    // How often interrupt is called, in microseconds
    uint8_t segment_complete_pin = 0; // Pin on which to signal that a segment is complete
    uint8_t limit_pin = 0; //
    uint8_t yellow_led_pin=0;
    uint8_t blue_led_pin=0;
    uint8_t running_led_pin=0;
    uint8_t empty_led_pin=0;
    uint8_t builtin_led_pin=0;
    bool debug_print = true;
    bool debug_tick = true;


    friend ostream &operator<<( ostream &output, const Config &c );
} Config;


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

    Config *asConfig() const { return (Config *) buffer.data(); }

    AxisConfig * asAxisConfig() const { return (AxisConfig *)buffer.data(); }

    Moves *asMoves() const { return (Moves *)buffer.data(); }

    CurrentState *asCurrentState() const { return (CurrentState *)buffer.data(); }

    string asString() const { return string( buffer.data(),buffer.size() ); }

    friend ostream &operator<<(ostream &output, const Message &s);

} Message;



/// A Move vector, which describes distances for all axes
/**
 * The Move Vector describes the distances to move for all
 * axes, plus the maximum velocity for the whole vector.
 * Note: The max velocity parameter is not currently used.
 *
 * THis is not a move message. THe move message is Moves
*/

struct Move {

    uint32_t seq = 0;

    MoveType move_type = MoveType::relative;

    // Total Vector Time, in microseconds.
    uint32_t t = 0;

    // Distances
    MoveVector x;

    Move(int n_joints):seq(0), move_type(MoveType::relative), t(0), x(){
        x.resize(n_joints);
    }

    //Move(int n_joints, uint32_t seq, uint32_t t, int v): seq(seq), t(t), x(){
    //    x.resize(n_joints);
    //}

    Move(uint32_t seq, uint32_t t, MoveType move_type, MoveVector x): seq(seq), move_type(move_type), t(t), x(x){}

    Move(uint32_t seq, uint32_t t, MoveType move_type, std::initializer_list<int> il):
            seq(seq), move_type(move_type), t(t), x(MoveVector(il.begin(), il.end())){}

    Move(uint32_t t, std::initializer_list<int> il): Move(0, t, MoveType::relative, il){}


private:
    friend std::ostream &operator<<( ostream &output, const Move &p );
};


