#pragma once

#include <limits>
#include <array>

#include "messageprocessor.h"
#include "planner.h"
#include "Hardware.h"
#include "config.h"
#include "stepper.h"

class StepperState;
class Stepper;

class Loop {

public:

    Loop(MessageProcessor &message_processor, Hardware &hardware, Planner &planner ) :
        mp(message_processor), hw(hardware), pl(planner), ss(SegmentStepper(planner, hardware))  {

        last_step_time = hw.millis();
    }

    void setup();

    void loopOnce();

    void processMessage(Message& message);

    void processMove(Message& message);


    void reset();
    void zero();
    void enable();
    void disable();

    int getLastSegNum(){ return last_seg_num; }

    void printInfo();

public:
    Config &getConfig(){ return config; };
    array<AxisConfig,N_AXES>& getAxesConfig(){ return axes_config;};
    CurrentState &getCurrentState(){ return current_state; }
    bool isPlannerEmpty(){ return pl.isEmpty();}
    bool isMessageEmpty(){ return mp.empty();}

    friend ostream &operator<<(ostream &output, const Loop &p);

    const SegmentStepper &getSegStepper() const { return ss; }

    void setConfig(const Config& config);
    void setAxisConfig(const AxisConfig& ac);

private:

    void setJoints();

    MessageProcessor &mp;
    Hardware  &hw;
    Planner &pl;

    SegmentStepper ss;


    Config config;
    array<AxisConfig,N_AXES> axes_config;
    CurrentState current_state;

    bool is_stopped = false;
    int last_seg_num = 0;
    bool running = false;
    bool empty = true;

    tmicros last_step_time;
};