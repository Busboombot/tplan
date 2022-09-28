#pragma once
#include <chrono>
#include "Hardware.h"
#include <vector>

using namespace std::chrono;

class HostHardware : public virtual Hardware {

public:

    explicit HostHardware() : Hardware() {
    }

    void update() override;

    void writePin(Pin pin, PinVal value) override;

    int readPin(Pin pin) override;

    void signalRunning(bool v) override;

    void signalEmpty(bool v) override;

    void signalError(bool v) override;

    tmillis millis() override;

    tmicros micros() override;

    tmillis millisSince(uint8_t tag) override;

    tmicros microsSince(uint8_t tag) override;

    void setMillisZero(uint8_t tag) override;

    void setMicrosZero(uint8_t tag) override;

    void delayMillis(uint32_t v) override;

    void delayMicros(uint32_t v) override;

    bool limitChanged() override;

public: // # local testing functions

    void printPins();

    void setPrintPins(bool v){print_pin_change = v;}

private:

    bool print_pin_change = false;
    u_int8_t pin_change_crc = 0;
    steady_clock::time_point last_pin_change;
    vector <PinVal> pins={0};

};