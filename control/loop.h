#pragma once

#include <limits>
#include <array>

#include "messageprocessor.h"
#include "planner.h"
#include "Hardware.h"
#include "trj_config.h"
#include "stepper.h"

class StepperState;
class Stepper;

class Loop {

public:

    Loop(MessageProcessor &message_processor, Hardware &hardware, Planner &planner ) :
        mp(message_processor), hw(hardware), pl(planner), ss(SegmentStepper(planner, hardware))  {
    }

    void setup();

    void loopOnce();

    void processMessage(Message& message);

    void setConfig(Message& message);

    void setAxisConfig(Message& message);

    void processMove(Message& message);


    void stop();

    void start();

    inline Config& getConfig(){ return config;}

    void reset();
    void zero();
    void enable();
    void disable();

    
    void signalSegmentComplete(); // Toggle pin to tell encoders that a segment is done
    void clearSegmentComplete();

    // Read encoder signal that a limit has been hit. 
    void limitChanged(); 


    int getLastSegNum(){
        return last_seg_num;
    }

    bool isEmpty(){ return false; }

    void printInfo();


private:

    MessageProcessor &mp;
    Planner &pl;
    Hardware  &hw;
    SegmentStepper ss;

    Config config;
    array<AxisConfig,N_AXES> axes_config;
    CurrentState current_state;

    unsigned int n_axes=N_AXES; // Number of axes in use.
    tmillis interrupt_delay=4;

    bool debug_print = false;
    bool debug_tick = false;


    bool is_stopped = false;
    int last_seg_num = 0;
    bool running = false;

};