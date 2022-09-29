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

    if (axis < axes.size()) {
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
    auto v = millis_0[tag];

    try {
        return millis()-millis_0.at(tag);
    } catch (out_of_range& e) {
        setMillisZero(tag);
        return millisSince(tag);
    }
}

tmicros Hardware::microsSince(uint8_t tag) {
    try {
        return micros()-micros_0.at(tag);
    } catch (out_of_range& e) {
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

