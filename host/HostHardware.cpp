#include "HostHardware.h"
#include "iostream"
#include "trj_util.h"
#include <ratio>

using namespace std::chrono;

void HostHardware::update() {

}

void HostHardware::printPins(){

    u_int8_t crc = crc8(pins);
    if (crc != pin_change_crc) {
        cout << overwrite;
        for (int i = 0; i < pins.size(); i++) {
            auto v = pins[i];

            if (v) {
                cout << blue_bg << i << creset;
            } else {
                cout << i;
            }
        }
        cout << endl;
        pin_change_crc = crc;
    }

}

void HostHardware::writePin(Pin pin, PinVal value) {
    if (pin < 255 && pin > pins.size()) {
        pins.resize(pin, 0);

    }

    pins[pin] = value;

    if (print_pin_change) {
        printPins();
    }

}

int HostHardware::readPin(Pin pin) {
    return 0;
}

void HostHardware::signalRunning(bool v) {

}

void HostHardware::signalEmpty(bool v) {

}

void HostHardware::signalError(bool v) {

}

void HostHardware::signalSegmentComplete() {

}

void HostHardware::setEmptyLed(PinVal value) {

}

void HostHardware::setRunningLed(PinVal value) {

}

void HostHardware::setBuiltinLed(PinVal value) {

}

void HostHardware::setYLed(PinVal value) {

}

void HostHardware::setBLed(PinVal value) {

}

tmillis HostHardware::millis() {
    return 0;
}

tmicros HostHardware::micros() {
    return 0;
}

tmillis HostHardware::millisSince(uint8_t tag) {
    return 0;
}

tmicros HostHardware::microsSince(uint8_t tag) {
    return 0;
}

void HostHardware::setMillisZero(uint8_t tag) {

}

void HostHardware::setMicrosZero(uint8_t tag) {

}

void HostHardware::delayMillis(uint32_t v) {

}

void HostHardware::delayMicros(uint32_t v) {

}

bool HostHardware::limitChanged() {
    return false;
}
