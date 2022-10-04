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
#include "util.h"
#include "loop.h"


/* Verify that the hardware object's timing features
 * work */
TEST_CASE("Int Timing Test", "[devel]") {

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

TEST_CASE("Hardware Time Test", "[devel]") {

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

TEST_CASE("CRC Test", "[devel]") {

    vector<u_int8_t> data = {1,2,3,4,5,6};

    cout << (int)crc8<u_int8_t>(data) << endl;
    cout << (int)crc8<int>({'a','b','c','d','e','f'}) << endl;
}

#include <type_traits>

template <typename TA, typename TB>
TA& operator-=(TA &a, const TB &b){

    static_assert(std::is_same<TA, MoveArray>::value || std::is_same<TA, MoveVector>::value,
                  "Templated operator -= is only for MoveArray or MoveVector");
    static_assert(std::is_same<TB, MoveArray>::value || std::is_same<TB, MoveVector>::value,
                  "Templated operator -= is only for MoveArray or MoveVector");
    std::transform(a.begin(), a.end(), b.begin(), a.begin(), std::plus<>());
    return a;
}

template <typename TA, typename TB>
TA& operator+=(TA &a, const TB &b){
    std::transform(a.begin(), a.end(), b.begin(), a.begin(), std::plus<>());
    return a;
}

TEST_CASE("Vector Add", "[devel]") {
    MoveArray ma{5,5,5,5,5,5};
    MoveVector mv{7,7,7,7,7,7};

    auto &a = ma;
    auto &b = mv;

    mv -= ma;

    cout << a << endl;

}

TEST_CASE("Unicode size", "[devel]") {
    // Can unicode literals be used for defining USB names? THe names are UTF16,
    // and are usually defined with arrays and defines

#define MANUFACTURER_NAME    {'B','u','s','b','o','t'}
#define MANUFACTURER_NAME_LEN    6
    auto sd = 2 + MANUFACTURER_NAME_LEN * 2;

    auto s = u"Busbot";
    wchar_t *z;
    cout << s << " size="<< char_traits<char16_t>::length(s)<< " " << sd << endl;

}