#pragma once
#include "Hardware.h"

class TargetHardware: public virtual Hardware {

public:

    TargetHardware();

    void update() override;

    void writePin(PinVal pin, PinVal value) override;

    int readPin(PinVal pin) override;

    void setPinMode(Pin pin, PinVal val) override;

    void signalRunning(bool v) override;

    void signalEmpty(bool v) override;

    void signalError(bool v) override;

    tmillis millis() override;

    tmicros micros() override;

    void delayMillis(uint32_t v) override;

    void delayMicros(uint32_t v) override;

    bool limitChanged() override;

    void setConfig(const Config &c) override;

    void setAxisConfig(const AxisConfig &ac) override;

};

