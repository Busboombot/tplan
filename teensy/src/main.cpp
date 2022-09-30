#include <Arduino.h>
#include "types.h"
#include "target.h"
#include "loop.h"
#include "messageprocessor.h"
#include "TargetHardware.h"
#include "TargetPacketSerial.h"
#include <deque>
#include <sstream>

//extern MessageProcessor *message_processor;

int main (){

    TargetPacketSerial tps(&Serial);
    TargetHardware hw;
    Planner pl;
    MessageProcessor mp(tps);
    CurrentState current_state;
    Loop loop(mp, hw, pl);

    message_processor = &mp;

    loop.setConfig(defaultConfig(2));
    loop.setAxisConfig(defaultAxisConfig(0));
    loop.setAxisConfig(defaultAxisConfig(1));

#ifdef TRJ_DEBUG_SERIAL
    TRJ_DEBUG_SERIAL.begin(115200);
#endif

    hw.setMillisZero(101);
    while (true){
        loop.loopOnce();

        if(hw.millisSince(101)>1500){
            hw.setMillisZero(101);
        }

    }


}


// These are supposed to be defined somewhere, but aren't
unsigned __exidx_start;
unsigned __exidx_end;
