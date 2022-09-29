#include "Hardware.h"
#include <iostream>
#include <algorithm>

using namespace std;

Hardware::Hardware() {
    default_stepper = new Stepper(this);

    Config c = Config();
    Hardware::setConfig(c);

}


void Hardware::setConfig(const Config &c) {
    config = c;
    axes.resize(config.n_axes);

}

void Hardware::setAxisConfig(const AxisConfig &ac) {
    if (ac.axis < axes.size()) {
        axes[ac.axis] = ac;
    }
}

Stepper Hardware::getStepper(int axis) {

    if ((size_t)axis < axes.size()) {
        AxisConfig &ac = axes[axis];
        return {this, (int8_t) ac.axis, ac.step_pin, ac.direction_pin, ac.enable_pin};
    } else {
        return Stepper(this);
    }
}


void Hardware::update() {
    writePin(config.segment_complete_pin, LOW);
}


tmillis Hardware::millisSince(uint8_t tag) {

    if(millis_0.find(tag)!=millis_0.end()) {
        return millis()-millis_0.at(tag);
    } else {
        setMillisZero(tag);
        return millisSince(tag);
    }

}

tmicros Hardware::microsSince(uint8_t tag) {

    if(micros_0.find(tag)!=micros_0.end()){
        return micros()-micros_0.at(tag);
    } else {
        setMicrosZero(tag);
        return microsSince(tag);
    }
}

void Hardware::setMillisZero(uint8_t tag) {
    millis_0[tag] = millis();
}

void Hardware::setMicrosZero(uint8_t tag) {
    micros_0[tag] = micros();
}


void Hardware::cycleLeds(){

    std::vector<int> pins{config.builtin_led_pin,config.empty_led_pin,
                          config.running_led_pin, config.yellow_led_pin,config.blue_led_pin };

    bool tog = true;
    // Cycle thorugh the LEDs
    for (int i = 0; i < 4; i++){
        for(int p: pins){
            writePin(p, tog);
            delayMillis(75);
        }
        tog = !tog;
    }
}

#define PATTERN_SIZE 20
#define BASE_DELAY 2000/PATTERN_SIZE // Pattern runs over 2,000 ms

int blink_patterns[4][PATTERN_SIZE] = {
        {1,0,1,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0},  // !empty & !running: 4 fast blinks
        {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,0},  // !empty & running: continuous fast blink
        {1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0},  // empty  & !running: long, slow blink
        {1,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0},  // empty  & running: 1 per second
};

/**
 * @brief Blink the onboard LED and set the Empty and Running LEDs.
 *
 * @param running
 * @param empty
 */
void Hardware::blink(bool running, bool empty){

    static int pattern_index = 0;

    static unsigned long last = millis();

    int pattern = ((int)empty)<<1 | ((int)running);

    setEmptyLed(empty);
    setRunningLed(running);

    if( (millis() - last) > BASE_DELAY  ){

        pattern_index = (pattern_index+1)%PATTERN_SIZE;
        setBuiltinLed(blink_patterns[pattern][pattern_index]);
        last = millis();
    }
}

