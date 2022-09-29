#include <iostream>
#include <string>
#include <limits>
#include <map>
#include <chrono>
#include <thread>

#include <catch2/catch_test_macros.hpp>

#include "trj_types.h"
#include "messageprocessor.h"
#include "PacketSerial.h"
#include "HostHardware.h"

#include "loop.h"

extern Config defaultConfig(uint8_t n_axes);
extern AxisConfig defaultAxisConfig(uint8_t axis);
extern void pushConfig(MockPacketSerial &mps, int axes, int period);
extern void pushAxisConfig(MockPacketSerial &mps, int axis);
extern void pushMove(MockPacketSerial &mps, CommandCode cmd, Moves m);
extern void pushMessage(MockPacketSerial &mps, CommandCode cmd);

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

    while(!mps.iqEmpty()) loop.loopOnce();

    loop.printInfo();

    REQUIRE(loop.getConfig().n_axes == 3);
    REQUIRE(loop.getConfig().segment_complete_pin == 21);
    REQUIRE(loop.getConfig().limit_pin == 22);
    REQUIRE(loop.getAxesConfig()[0].step_pin == 3);
    REQUIRE(loop.getAxesConfig()[1].step_pin == 6);
    REQUIRE(loop.getAxesConfig()[2].step_pin == 9);
    REQUIRE(loop.getAxesConfig()[3].step_pin == 0); // This axis is unconfigured

    pushMessage(mps, CommandCode::RUN);
    pushMove(mps, CommandCode::RMOVE, {0, {1000,10,10}});
    pushMove(mps, CommandCode::RMOVE, {0, {-1000,20,-10}});
    cout << loop <<  pl << endl;
    while(!mps.iqEmpty()) loop.loopOnce(); // Load the message processor

    cout << loop << pl << endl;

    //hw.setPrintPins(true)
    while(!pl.empty()){
        loop.loopOnce();
    }

    cout << loop  << pl << loop.getSegStepper() << endl;

    hw.dumpPinCounts();
}