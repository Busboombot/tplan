//
// Created by Eric Busboom on 9/26/22.
//

#include "Arduino.h"
#include "TargetHardware.h"



TargetHardware::TargetHardware() : Hardware() {

}

void TargetHardware::update() {

}

void TargetHardware::writePin(PinVal pin, PinVal value) {
    digitalWriteFast(pin, value);
}

int TargetHardware::readPin(PinVal pin) {
    return digitalReadFast(pin);
}

void TargetHardware::signalRunning(bool v) {

}

void TargetHardware::signalEmpty(bool v) {

}

void TargetHardware::signalError(bool v) {

}


tmillis TargetHardware::millis() {
    return ::millis();
}

tmicros TargetHardware::micros() {
    return ::micros();
}


void TargetHardware::delayMillis(uint32_t v) {
    return delay(v);
}

void TargetHardware::delayMicros(uint32_t v) {
    return delayMicroseconds(v);
}

bool TargetHardware::limitChanged() {
    //pinMode(config.limit_pin, INPUT);
    //attachInterrupt(config.limit_pin, limitChangedISR, RISING);
    return false;
}

void TargetHardware::setPinMode(Pin pin, PinVal val) {
    pinMode(pin, val);
}

void TargetHardware::setConfig(const Config &c) {
    Hardware::setConfig(c);

    setPinMode(c.segment_complete_pin, OUTPUT);
    setPinMode(c.limit_pin, OUTPUT);

    setPinMode(c.yellow_led_pin, OUTPUT);
    setPinMode(c.blue_led_pin, OUTPUT);
    setPinMode(c.running_led_pin, OUTPUT);
    setPinMode(c.empty_led_pin, OUTPUT);
    setPinMode(c.builtin_led_pin, OUTPUT);

}

void TargetHardware::setAxisConfig(const AxisConfig &ac) {
    Hardware::setAxisConfig(ac);

    setPinMode(ac.step_pin, ac.step_output_mode);
    setPinMode(ac.direction_pin, ac.direction_output_mode);
    setPinMode(ac.enable_pin, ac.enable_output_mode);

}
