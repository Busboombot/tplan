#include <iostream>
#include <string>
#include <limits>
#include <map>
#include <chrono>
#include <thread>

#include <catch2/catch_test_macros.hpp>

#include "types.h"
#include "messageprocessor.h"
#include "PacketSerial.h"
#include "HostHardware.h"
#include "joint.h"

#define CATCH_CONFIG_MAIN

using namespace std;
using namespace std::chrono;

Config defaultConfig(uint8_t n_axes){

    uint8_t base_pin = (((N_AXES+1)*3)+0);

    return Config {
            n_axes, // uint8_t n_axes = 0;         // Number of axes
            4,      // uint8_t interrupt_delay = INTERRUPT_DELAY;    // How often interrupt is called, in microseconds
            uint8_t(base_pin+0),     // uint8_t segment_complete_pin = 0; // Pin on which to signal that a segment is complete
            uint8_t(base_pin+1),     // uint8_t limit_pin = 0; // Pin to recieve signals that the encoder foind a limit
            uint8_t(base_pin+2), // uint8_t yellow_led_pin=0;
            uint8_t(base_pin+3), // uint8_t blue_led_pin=0;
            uint8_t(base_pin+4), // uint8_t running_led_pin=0;
            uint8_t(base_pin+5), // uint8_t empty_led_pin=0;
            uint8_t(base_pin+6), // uint8_t builtin_led_pin=0;
            false,  // bool debug_print = true;
            false,  // bool debug_tick = true;
    };
}

AxisConfig defaultAxisConfig(uint8_t axis){

    return {

            axis,           // uint8_t axis=0;           // Axis number

            static_cast<uint8_t>(((axis+1)*3)+0),     // uint8_t step_pin=0;       // Step output,
            static_cast<uint8_t>(((axis+1)*3)+1),     // uint8_t direction_pin=0;  // Direction output
            static_cast<uint8_t>(((axis+1)*3)+2),     // uint8_t enable_pin=0;

            HIGH,           // step_high_value=0; // Whether step is HIGH or LOW when enabled.
            HIGH,           // uint8_t direction_high_value=0;
            HIGH,           // uint8_t enable_high_value=0;

            0,              // uint8_t step_output_mode=0; // OUTPUT or OUTPUT_OPEN_DRAIN
            0,              // uint8_t direction_output_mode=0;
            0,              // uint8_t enable_output_mode=0;

            0xDE,
            0XAD,

            5000,          // uint32_t v_max=0;
            100000         //  a_max=0;
    };
}

int seq_id = 0;

void pushConfig(MockPacketSerial &mps, int axes, int period=5){
    Config config = defaultConfig(axes);
    config.interrupt_delay = period;
    mps.push(PacketHeader(seq_id++, CommandCode::CONFIG), (char *) &config, sizeof(Config));
}

void pushAxisConfig(MockPacketSerial &mps, int axis){
    AxisConfig ac = defaultAxisConfig(axis);
    mps.push(PacketHeader(seq_id++, CommandCode::AXES), (char *) &ac, sizeof(AxisConfig));
}

void pushMove(MockPacketSerial &mps, CommandCode cmd, Moves m){
    mps.push(PacketHeader(seq_id++, cmd), (char *) &m, sizeof(Moves));
}

void pushMessage(MockPacketSerial &mps, CommandCode cmd){
    mps.push(PacketHeader(seq_id++, cmd), (char *) 0, 0);
}


std::vector<int> extractIntegerWords(const string& str)
{
    stringstream ss(str);
    std::vector<int> result;

    string temp;
    int found;
    while (!ss.eof()) {

        ss >> temp;

        if (stringstream(temp) >> found)
            result.push_back(found);

        temp = "";
    }

    return result;
}

vector<Move> get2Moves(){
    return std::vector<Move>{
            Move( 0, {10000,100}),
            Move( 0, {10000,100})
    };
}

vector<Joint> get2Joints(){
    return std::vector<Joint>{
            Joint(0, 5e3, 50e3),
            Joint(1, 5e3, 50e3)
    };
}

vector<Joint> get4Joints(){
    return std::vector<Joint>{
            Joint(0, 5e3, 50e3),
            Joint(1, 5e3, 50e3),
            Joint(2, 5e3, 50e3),
            Joint(3, 5e3, 50e3)
    };
}


