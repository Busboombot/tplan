#include <iostream>
#include <string>
#include <chrono>
#include <catch2/catch_test_macros.hpp>

#include "types.h"
#include "messageprocessor.h"
#include "PacketSerial.h"
#include "HostHardware.h"

#include "loop.h"

#include "test.h"

TEST_CASE("Basic Loop Test", "[loop]") {

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
    while (!mps.iqEmpty()) loop.loopOnce();

    loop.printInfo();

    REQUIRE(loop.getConfig().n_axes == 3);
    REQUIRE(loop.getConfig().segment_complete_pin == 21);
    REQUIRE(loop.getConfig().limit_pin == 22);
    REQUIRE(loop.getAxesConfig()[0].step_pin == 3);
    REQUIRE(loop.getAxesConfig()[1].step_pin == 6);
    REQUIRE(loop.getAxesConfig()[2].step_pin == 9);
    REQUIRE(loop.getAxesConfig()[3].step_pin == 0); // This axis is unconfigured

    pushMove(mps, CommandCode::RMOVE, {0, {10000, 10, 1000}});
    pushMove(mps, CommandCode::RMOVE, {0, {-10000, 20, -1000}});
    pushMessage(mps, CommandCode::RUN);

    cout << loop << pl << endl;
    while (!mps.iqEmpty()) loop.loopOnce(); // Load the message processor. w/o this, pl will be empty for next loop

    cout << loop << pl << endl;
    hw.useSystemTime(false);
    //hw.setPrintPins(true)
    const auto t0 = sclock::now();

    int n = 0;
    while (!pl.isEmpty()) {
        hw.stepTime(1); // Step 1 micro second per loop
        n++;
        loop.loopOnce();
    }
    const sec dt = sclock::now() - t0;

    std::cout << "Loop time: " << n / dt.count() << " loop/sec" << endl;

    cout << loop << pl << loop.getSegStepper() << endl;

    hw.dumpPinCounts();

    REQUIRE(hw.highCount[3] == 20000);
    REQUIRE(hw.lowCount[3] == 20000);

    REQUIRE(hw.highCount[9] == 2000);
    REQUIRE(hw.lowCount[9] == 2000);

}

/**
 * @brief Same test as above, but interacts directly with the loop, skipping
 * the messages passed through the message processor.
 */
TEST_CASE("Short Loop Test", "[loop]") {

    CurrentState current_state;
    MockPacketSerial mps;
    MessageProcessor mp(static_cast<IPacketSerial &>(mps));
    HostHardware hw;
    Planner pl;

    Loop loop(mp, hw, pl);

    loop.setConfig(defaultConfig(3));
    loop.setAxisConfig(defaultAxisConfig(0));
    loop.setAxisConfig(defaultAxisConfig(1));
    loop.setAxisConfig(defaultAxisConfig(2));

    loop.processMove(Move(0, 0, MoveType::relative, {10000, 10, 1000} ));
    loop.processMove(Move(0, 0, MoveType::relative, {-10000, 10, -1000} ));

    loop.run();


    int n = 0;
    while (!pl.isEmpty()) {
        hw.stepTime(1); // Step 1 micro second per loop
        n++;
        loop.loopOnce();
    }

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
    hw.setPrintPins(false);
    hw.setConfig(defaultConfig(0));

    for (int i = 0; i < 4; i++) {
        int running = i & 1;
        int empty = i & 2;

        for (int j = 0; j < 1000; j++) {
            hw.stepTime(90 * 1000);
            hw.blink(running, empty);
        }
    }

    hw.dumpPinCounts();

    REQUIRE(hw.lowCount[13] == 399);

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

    while (hw.millisSince(101) < 10000) {
        loop.loopOnce();
    };
}

TEST_CASE("VMOVE Loop Test", "[loop]") {

    CurrentState current_state;
    MockPacketSerial mps;
    MessageProcessor mp(static_cast<IPacketSerial &>(mps));
    HostHardware hw;
    Planner pl;

    Loop loop(mp, hw, pl);

    loop.setConfig(defaultConfig(2));
    loop.setAxisConfig(defaultAxisConfig(0));
    loop.setAxisConfig(defaultAxisConfig(1));

    loop.processMove(Move(0, 300'000, MoveType::jog, {500,1000} ));
    loop.processMove(Move(0, 300'000, MoveType::jog, {500,1000} ));
    loop.processMove(Move(0, 300'000, MoveType::jog, {500,1000} ));
    loop.processMove(Move(0, 300'000, MoveType::jog, {500,1000} ));

    loop.printQueue();

    loop.run();


    int n = 0;
    while (!pl.isEmpty()) {
        hw.stepTime(1); // Step 1 micro second per loop
        n++;
        loop.loopOnce();
    }

    hw.dumpPinCounts();

}

TEST_CASE("Zero Move Jog Test", "[loop]") {

    CurrentState current_state;
    MockPacketSerial mps;
    MessageProcessor mp(static_cast<IPacketSerial &>(mps));
    HostHardware hw;
    Planner pl;

    Loop loop(mp, hw, pl);

    loop.setConfig(defaultConfig(2));
    loop.setAxisConfig(defaultAxisConfig(0));
    loop.setAxisConfig(defaultAxisConfig(1));

    loop.run();

    for (int i = 0; i < 100; i++){
        loop.processMove(Move(0, 300'000, MoveType::jog, {0,0} ));
        for( int j = 0; j < 200'000; j++){
            hw.stepTime(2);
            loop.loopOnce();
        }

    }

    while (!pl.isEmpty()) {
        hw.stepTime(1); // Step 1 micro second per loop
        loop.loopOnce();
    }

    hw.dumpPinCounts();

}

