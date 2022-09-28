#pragma once
#include "Hardware.h"

class TargetHardware: public virtual Hardware {

public:
    void update() override;

    void writePin(PinVal pin, PinVal value) override;

    int readPin(PinVal pin) override;

    void signalRunning(bool v) override;

    void signalEmpty(bool v) override;

    void signalError(bool v) override;

    void signalSegmentComplete() override;

    void setEmptyLed(PinVal value) override;

    void setRunningLed(PinVal value) override;

    void setBuiltinLed(PinVal value) override;

    void setYLed(PinVal value) override;

    void setBLed(PinVal value) override;

    tmillis millis() override;

    tmicros micros() override;

    tmillis millisSince(uint8_t tag) override;

    tmicros microsSince(uint8_t tag) override;

    void setMillisZero(uint8_t tag) override;

    void setMicrosZero(uint8_t tag) override;

    void delayMillis(uint32_t v) override;

    void delayMicros(uint32_t v) override;

    bool limitChanged() override;

    void setConfig(const Config &c) override;

    void setAxisConfig(const AxisConfig &ac) override;

    Stepper getStepper(int axis) override;

    vector<Stepper> getSteppers() override;

};

