#include <iostream>

#include <catch2/catch_test_macros.hpp>

#include "types.h"
#include "messageprocessor.h"
#include "PacketSerial.h"
#include "HostHardware.h"

#include "loop.h"
#include "test.h"


TEST_CASE("Test Loop Blink", "[loop][devel]") {

    CurrentState current_state;
    MockPacketSerial mps;
    MessageProcessor mp(static_cast<IPacketSerial &>(mps));
    HostHardware hw;
    Planner pl;

    Loop loop(mp, hw, pl);

    hw.useSystemTime(false);
    hw.setPrintPins(true);
    hw.setConfig(defaultConfig(0));

    for(int i = 0; i < 4; i++){
        int running = i&1;
        int empty = i&2;

        for(int j = 0; j<100; j++){
            hw.stepTime(90*1000);
            hw.blink(running, empty);
        }
        cout << "-------"<<endl;
    }
}

TEST_CASE("Test Loop Blink 2", "[loop][devel]") {

    CurrentState current_state;
    MockPacketSerial mps;
    MessageProcessor mp(static_cast<IPacketSerial &>(mps));
    HostHardware hw;
    Planner pl;

    Loop loop(mp, hw, pl);

    hw.useSystemTime(true);
    hw.setPrintPins(true);
    hw.setConfig(defaultConfig(0));

    hw.setMillisZero(101);

    while(hw.millisSince(101) < 10000){
        loop.loopOnce();
    };
}