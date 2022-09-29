#include <iostream>
#include <sstream>
#include <string>
#include <catch2/catch_test_macros.hpp>

#include "types.h"

#define CATCH_CONFIG_MAIN

using namespace std;

// extern Config defaultConfig(uint8_t n_axes);
// AxisConfig defaultAxisConfig(uint8_t axis)

Config defaultConfig(uint8_t n_axes){
    return Config {
            n_axes, // uint8_t n_axes = 0;         // Number of axes
            4,      // uint8_t interrupt_delay = INTERRUPT_DELAY;    // How often interrupt is called, in microseconds
            20,     // uint8_t segment_complete_pin = 0; // Pin on which to signal that a segment is complete
            21,     // uint8_t limit_pin = 0; // Pin to recieve signals that the encoder foind a limit
            22, // uint8_t yellow_led_pin=0;
            23, // uint8_t blue_led_pin=0;
            24, // uint8_t running_led_pin=0;
            25, // uint8_t empty_led_pin=0;
            26, // uint8_t builtin_led_pin=0;
            false,  // bool debug_print = true;
            false,  // bool debug_tick = true;
    };
}

AxisConfig defaultAxisConfig(uint8_t axis){

    return {

        axis,           // uint8_t axis=0;           // Axis number

        static_cast<uint8_t>((axis*3)+0),     // uint8_t step_pin=0;       // Step output,
        static_cast<uint8_t>((axis*3)+1),     // uint8_t direction_pin=0;  // Direction output
        static_cast<uint8_t>((axis*3)+2),     // uint8_t enable_pin=0;

        HIGH,           // step_high_value=0; // Whether step is HIGH or LOW when enabled.
        HIGH,           // uint8_t direction_high_value=0;
        HIGH,           // uint8_t enable_high_value=0;

        0,              // uint8_t step_output_mode=0; // OUTPUT or OUTPUT_OPEN_DRAIN
        0,              // uint8_t direction_output_mode=0;
        0,              // uint8_t enable_output_mode=0;

        0xDE,
        0XAD,

        10000,          // uint32_t v_max=0;
        100000         //  a_max=0;
    };
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


