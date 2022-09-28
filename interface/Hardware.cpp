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

vector<Stepper> Hardware::getSteppers() {
    return steppers;
}

