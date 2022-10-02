#include <iostream>
#include <string>
#include <limits>
#include <map>
#include <chrono>
#include <thread>

#include <catch2/catch_test_macros.hpp>

#include "types.h"
#include "messageprocessor.h"
#include "PacketSerial.h"
#include "HostHardware.h"

#include "loop.h"

#include "test.h"

TEST_CASE("Basic Loop Test", "[loop]"){

    CurrentState current_state;
    MockPacketSerial mps;
    MessageProcessor mp(static_cast<IPacketSerial &>(mps));
    HostHardware hw;
    Planner pl;

    Loop loop(mp, hw, pl);

    pushConfig(mps, 3, 5);
    pushAxisConfig(mps, 0);
    pushAxisConfig(mps, 1);
    pushAxisConfig(mps, 2);

    hw.useSystemTime(true);
    while(!mps.iqEmpty()) loop.loopOnce();

    loop.printInfo();

    REQUIRE(loop.getConfig().n_axes == 3);
    REQUIRE(loop.getConfig().segment_complete_pin == 21);
    REQUIRE(loop.getConfig().limit_pin == 22);
    REQUIRE(loop.getAxesConfig()[0].step_pin == 3);
    REQUIRE(loop.getAxesConfig()[1].step_pin == 6);
    REQUIRE(loop.getAxesConfig()[2].step_pin == 9);
    REQUIRE(loop.getAxesConfig()[3].step_pin == 0); // This axis is unconfigured

    pushMove(mps, CommandCode::RMOVE, {0, {10000,10,1000}});
    pushMove(mps, CommandCode::RMOVE, {0, {-10000,20,-1000}});
    pushMessage(mps, CommandCode::RUN);

    cout << loop <<  pl << endl;
    while(!mps.iqEmpty()) loop.loopOnce(); // Load the message processor. w/o this, pl will be empty for next loop

    cout << loop << pl << endl;
    hw.useSystemTime(false);
    //hw.setPrintPins(true)
    const auto t0 = sclock::now();

    int n = 0;
    while(!pl.isEmpty()){
        hw.stepTime(1); // Step 1 micro second per loop
        n++;
        loop.loopOnce();
    }
    const sec dt = sclock::now() - t0;

    std::cout << "Loop time: " << n/dt.count() << " loop/sec" << endl;

    cout << loop  << pl << loop.getSegStepper() << endl;

    hw.dumpPinCounts();
}

TEST_CASE("Test Loop Blink", "[loop]") {

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

TEST_CASE("Test Loop Blink 2", "[loop]") {

    return; // Mostly slo and uninteresting.
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