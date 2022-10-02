#pragma once

#include "types.h"
#include "const.h"
#include <array>
#include <map>

class Stepper;
class StepperState;


using  millimap = std::map<uint8_t,tmillis>;
using  micromap = std::map<uint8_t,tmicros>;

class Hardware {


private:
    int n_axes = 0;
    Config config;
    vector<AxisConfig> axes;

    Stepper *default_stepper;

    millimap millis_0; // Base time for millis since
    micromap micros_0; // Base time for micros since

    size_t blink_index = 0;

public:

    Hardware();

    virtual ~Hardware() = default;

    virtual void update();

    virtual void setPin(Pin pin) { writePin(pin, HIGH); }

    virtual void clearPin(Pin pin) { writePin(pin, LOW); }

    virtual void writePin(Pin pin, PinVal value) = 0;

    virtual void setPinMode(Pin, PinVal){}

    virtual int readPin(Pin pin) = 0;

    virtual void signalRunning(bool v) = 0;

    virtual void signalEmpty(bool v) = 0;

    virtual void signalError(bool v) = 0;

    virtual void signalSegmentComplete() { writePin(config.segment_complete_pin, HIGH); }


    double getDTime() { return (((double) config.interrupt_delay) / TIMEBASE); }

    virtual tmillis millis() = 0;

    virtual tmicros micros() = 0;

    tmillis millisSince(uint8_t tag);

    tmicros microsSince(uint8_t tag);

    void setMillisZero(uint8_t tag);

    void setMicrosZero(uint8_t tag);

    bool everyMs(uint8_t tag, tmillis frequency);

    virtual void delayMillis(uint32_t v) = 0;

    virtual void delayMicros(uint32_t v) = 0;

    virtual bool limitChanged() = 0;

    virtual void setConfig(const Config &c);

    virtual void setAxisConfig(const AxisConfig &ac);

    void enableSteppers();

    void disableSteppers();

    virtual Stepper getStepper(int axis);

    // LEDS

    void cycleLeds();

    void blink(bool running, bool empty);

    void setEmptyLed(PinVal value) { writePin(config.empty_led_pin, value); }

    void setRunningLed(PinVal value) { writePin(config.running_led_pin, value); }

    void setBuiltinLed(PinVal value) { writePin(config.builtin_led_pin, value); }

    void setYLed(PinVal value) { writePin(config.yellow_led_pin, value); }

    void setBLed(PinVal value) { writePin(config.blue_led_pin, value); }


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

protected:

    Pin step_pin = -1;
    Pin direction_pin = -1;
    Pin enable_pin = -1;

    PinVal enable_val = HIGH;
    PinVal cw_dir_val = HIGH; // Pin value for CW direction“

    PinVal step_state;

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
    void setStep() { hw->writePin(step_pin, HIGH); step_state = 1; }

    // Set the step pin for this axis to LOW
    void clearStep() {
        // We clear the step pin regularly, so it may be more efficient to avoid setting it
        // when it is already unset.
        //if (step_state != 0) {
        hw->writePin(step_pin, LOW);
        step_state = 0;
        //}
    }

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

    int8_t getAxis() const { return axis; }

    PinVal getStepState(){ return step_state; }

    friend ostream &operator<<(ostream &output, const Stepper &s);

    friend ostream &operator<<(ostream &output, const StepperState &ss);

private:

};