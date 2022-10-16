#pragma once
#include <chrono>
#include <vector>
#include <map>
#include "Hardware.h"
using namespace std::chrono;

class HostHardware : public virtual Hardware {

public:

    explicit HostHardware();

    void update() override;

    void writePin(Pin pin, PinVal value) override;

    int readPin(Pin pin) override;

    void signalRunning(bool v) override;

    void signalEmpty(bool v) override;

    void signalError(bool v) override;

    tmillis millis() override;

    tmicros micros() override;

    void delayMillis(uint32_t v) override;

    void delayMicros(uint32_t v) override;

    bool limitChanged() override;

public: // # local testing functions

    void printPins();

    // Set to true to print out pin changes.
    void setPrintPins(bool v){print_pin_change = v;}

    void dumpPinCounts();

    // Increment internal time
    void stepTime(tmicros dt);

    void useSystemTime(bool v=true){use_system_time = v;}

private:

    bool use_system_time = false;
    tmicros hw_time=0;

    // Timepoint when this object was constructed
    steady_clock::time_point t0;

    bool print_pin_change = false;
    u_int8_t pin_change_crc = 0;
    steady_clock::time_point last_pin_change;
public: // Public b/c this object is for testing.
    vector<PinVal> pins={0};
    map<PinVal, int> highCount;
    map<PinVal, int> lowCount;
    map<PinVal, int> missCount;
    map<PinVal, int> transCount;

    map<PinVal, tmicros> writeTime;

};