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


/* Verify that the hardware object's timing features
 * work */
TEST_CASE("Int Timing Test", "[scratch]") {

    using T = tmillis;
    T mx = numeric_limits<T>::max();
    T last = mx - 50;
    T now = mx + 50;

    // Just verifying wrap-around for differences
    cout << last << " " << now << " " << float(mx / 1000 / 60 / 60 / 24) / 365. << " " << numeric_limits<T>::digits
         << endl;
    cout << "DIFF " << now - last << endl;

    REQUIRE(now - last == 100);

    HostHardware hh;

    std::this_thread::sleep_for(milliseconds(10));
    hh.setMillisZero(1);
    hh.setMicrosZero(1);
    std::this_thread::sleep_for(milliseconds(10));
    hh.setMillisZero(2);
    hh.setMicrosZero(2);

    auto t0 = steady_clock::now();
    for (int i = 0; i < 100; i++) {
        std::this_thread::sleep_until(t0 + (i * milliseconds(1)));
        auto t = (tmicros) std::chrono::duration_cast<std::chrono::microseconds>(steady_clock::now() - t0).count();
        auto us = hh.micros();
        auto ms = hh.millis();
        auto u_since_1 = hh.microsSince(1);
        auto m_since_1 = hh.millisSince(1);
        auto u_since_2 = hh.microsSince(2);
        auto m_since_2 = hh.millisSince(2);

        cout << t << " " << us << " " << u_since_1 << " " << u_since_2 << " " <<
             ms << " " << m_since_1 << " " << m_since_2 << endl;

    }
}

TEST_CASE("Hardware Time Test", "[scratch]") {

    CurrentState current_state;
    MockPacketSerial mps;
    MessageProcessor mp(static_cast<IPacketSerial &>(mps));
    HostHardware hw;
    Planner pl;

    Loop loop(mp, hw, pl);


    for(int i = 0; i < 1e6; i++){
        hw.stepTime(1);
        loop.loopOnce();
    }
}
