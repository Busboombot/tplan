#include <Arduino.h>
#include "types.h"
#include "loop.h"
#include "messageprocessor.h"
#include "TargetHardware.h"

// These are supposed to be defined somewhere, but aren't
unsigned __exidx_start;
unsigned __exidx_end;



int main (void){
    CurrentState current_state;
    TargetHardware hw;
    Planner pl;
    MessageProcessor mp(static_cast<IPacketSerial &>(mps));

    Loop loop(mp, hw, pl);
}