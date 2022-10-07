#include <iostream>
#include <vector>

#include "../test.h"
#include "joint.h"
#include "planner.h"


#include <catch2/catch_test_macros.hpp>


TEST_CASE("Basic Planner Test", "[planner]") {

    vector<Joint> joints = get2Joints();

    Planner p(joints);

    p.move(1, {1000, 1000});
    p.move(2, {1000, 1000});
    p.move(3, {1000, 1000});


    cout << " ============ " << endl;
    cout << p << endl;
}

TEST_CASE("Large Small Planner Test", "[planner]") {

    vector<Joint> joints = get2Joints();

    Planner p(joints);

    p.move(10, {1000, 1});
    p.move(11, {1, 1000});

    cout << " ============ " << endl;
    cout << p << endl;

}

TEST_CASE("VMOVE PlannerTest", "[planner]") {

    vector<Joint> joints = get2Joints();

    Planner p(joints);

    for(int i = 0; i< 10; i+=2) {
        p.vmove(i, 200'000, {4000, 400});
        p.vmove(i + 1, 200'000, {400, 4000});
        //p.move(i,  {1000,1000});
        //p.move(i+1,{1000,1000});
    }

    cout << " ============ " << endl;
    cout << p << endl;
    for(const Segment &s:p.getSegments()) {
        cout << s << endl;
    }

    CurrentState current_state;
    MockPacketSerial mps;
    MessageProcessor mp(static_cast<IPacketSerial &>(mps));
    HostHardware hw;

    hw.setConfig(defaultConfig(2));
    hw.setAxisConfig(defaultAxisConfig(0));
    hw.setAxisConfig(defaultAxisConfig(1));

    Loop loop(mp, hw, p);
    loop.run();

    hw.useSystemTime(false);
    //hw.setPrintPins(true);

    int n = 0;
    while (!p.isEmpty()) {
        hw.stepTime(5); // Step 5 micro second per loop
        n++;
        loop.loopOnce();
    }

    hw.dumpPinCounts();

}





