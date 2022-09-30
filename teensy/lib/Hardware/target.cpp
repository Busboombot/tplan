//
// Created by Eric Busboom on 9/29/22.
//

#include "Arduino.h"
#include "target.h"
#include <string>
#include "types.h"


Config defaultConfig(uint8_t n_axes){

    uint8_t base_pin = (((N_AXES+1)*3)+0);

    return Config {
            1, // uint8_t n_axes = 0;         // Number of axes
            5,      // uint8_t interrupt_delay = INTERRUPT_DELAY;
            uint8_t(base_pin+0),     // uint8_t segment_complete_pin = 0;
            uint8_t(base_pin+1),     // uint8_t limit_pin = 0;
            26, // uint8_t yellow_led_pin=0;
            31, // uint8_t blue_led_pin=0;
            28, // uint8_t running_led_pin=0;
            30, // uint8_t empty_led_pin=0;
            13, // uint8_t builtin_led_pin=0;
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

            OUTPUT,         // uint8_t step_output_mode=0; // OUTPUT or OUTPUT_OPEN_DRAIN
            OUTPUT,         // uint8_t direction_output_mode=0;
            OUTPUT,         // uint8_t enable_output_mode=0;

            0xDE, 0XAD, // Padding

            5000,          // uint32_t v_max=0;
            100000         //  a_max=0;
    };
}