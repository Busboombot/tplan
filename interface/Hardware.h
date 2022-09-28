#pragma once

#include "trj_types.h"
#include "trj_const.h"
#include <array>

class Stepper;

class StepperState;

class Hardware {

private:
    int n_axes = 0;
    Config config;
    vector<AxisConfig> axes;
    vector<Stepper> steppers;
    Stepper *default_stepper;

public:

    Hardware();

    virtual ~Hardware() = default;

    virtual void update();

    virtual void setPin(Pin pin) { writePin(pin, HIGH); }

    virtual void clearPin(Pin pin) { writePin(pin, LOW); }

    virtual void writePin(Pin pin, PinVal value) = 0;

    virtual int readPin(Pin pin) = 0;

    virtual void signalRunning(bool v) = 0;

    virtual void signalEmpty(bool v) = 0;

    virtual void signalError(bool v) = 0;

    virtual void signalSegmentComplete() { writePin(config.segment_complete_pin, HIGH); }

    void setEmptyLed(PinVal value) { writePin(config.empty_led_pin, value); }

    void setRunningLed(PinVal value) { writePin(config.running_led_pin, value); }

    void setBuiltinLed(PinVal value) { writePin(config.builtin_led_pin, value); }

    void setYLed(PinVal value) { writePin(config.yellow_led_pin, value); }

    void setBLed(PinVal value) { writePin(config.blue_led_pin, value); }

    double getDTime() { return (((double) config.interrupt_delay) / TIMEBASE); }

    virtual tmillis millis() = 0;

    virtual tmicros micros() = 0;

    virtual tmillis millisSince(uint8_t tag) = 0;

    virtual tmicros microsSince(uint8_t tag) = 0;

    virtual void setMillisZero(uint8_t tag) = 0;

    virtual void setMicrosZero(uint8_t tag) = 0;

    virtual void delayMillis(uint32_t v) = 0;

    virtual void delayMicros(uint32_t v) = 0;

    virtual bool limitChanged() = 0;

    virtual void setConfig(const Config &c);

    virtual void setAxisConfig(const AxisConfig &ac);

    virtual Stepper getStepper(int axis);

    virtual vector<Stepper> getSteppers();
};

/**
 * @brief Hardware interface to steppers
 *
 * Stepper objects are created from Hardware objects as interfaces to the pins
 * for a single axis.
 *
 */
class Stepper {

protected:

    Hardware *hw;
    int8_t axis;

    Pin step_pin = -1;
    Pin direction_pin = -1;
    Pin enable_pin = -1;

    PinVal enable_val = HIGH;
    PinVal cw_dir_val = HIGH; // Pin value for CW direction“

public:

    explicit Stepper(Hardware *hardware) :
            hw(hardware), axis(-1), step_pin(-1), direction_pin(-1), enable_pin(-1) {}

    Stepper(Hardware *hardware, int8_t axis) : hw(hardware), axis(axis) {};

    Stepper(Hardware *hw, int8_t axis, Pin stepPin, Pin directionPin, Pin enablePin) :
            hw(hw), axis(axis), step_pin(stepPin), direction_pin(directionPin), enable_pin(enablePin) {}

    Stepper(Stepper const &stepper_) :
            hw(stepper_.hw), axis(stepper_.axis), step_pin(stepper_.step_pin), direction_pin(stepper_.direction_pin),
            enable_pin(stepper_.enable_pin), enable_val(stepper_.enable_val), cw_dir_val(stepper_.cw_dir_val) {

    }

    virtual ~Stepper() = default;

    /**
     * @brief Set the step pin for this axis to HIGH
     */
    void setStep() { hw->writePin(step_pin, HIGH); }

    // Set the step pin for this axis to LOW
    void clearStep() { hw->writePin(step_pin, LOW); }

    void enable() { hw->writePin(enable_pin, enable_val); }

    void enable(Direction dir) {
        setDirection(dir);
        enable();
    }

    void disable() {
        setDirection(STOP);
        hw->writePin(enable_pin, !enable_val);
    }

    void setDirection(Direction dir) {
        hw->writePin(direction_pin, (dir == Direction::CW) ? cw_dir_val : !cw_dir_val);
    }

    void setDirection(int dir) { setDirection(static_cast<Direction>(dir)); };

    friend ostream &operator<<(ostream &output, const Stepper &s);

    friend ostream &operator<<(ostream &output, const StepperState &ss);

private:

};