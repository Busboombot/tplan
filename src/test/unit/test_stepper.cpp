#include <iostream>
#include <memory>
#include <vector>
#include <numeric>
#include <fstream>

#include "../test.h"

#include "catch2/catch_all.hpp"

#include "segment.h"
#include "joint.h"
#include "planner.h"
#include "stepper.h"
#include "HostHardware.h"



#include "boost/filesystem.hpp"   // includes all needed Boost.Filesystem declarations
#include <iostream>               // for std::cout

using namespace boost::filesystem;

extern vector<Move> get2Moves();
extern vector<Joint> get2Joints();
extern vector<Joint> get4Joints();
extern std::vector<int> extractIntegerWords(const string& str);
using Ints = vector<int>;


class CoutStepper  {

    Direction direction;

public:

    CoutStepper(int axis)  { }

    ~CoutStepper()  {}

    void writeStep()  {
        count += direction;
        lastStep = 1;
    }

    void clearStep()  {
        lastStep = 0;
    }

    void setDirection(Direction direction_)  {
        direction = direction_;
    }

public:
    int lastStep = 0;
    int count = 0;
};

#define csdc(p) ( std::dynamic_pointer_cast<CoutStepper>(p))

TEST_CASE("Basic Stepper Test", "[stepper]") {

    double dtime = 5./1e6; // 5 us

    vector<Joint> joints = get2Joints();
    Planner p(joints);
    HostHardware hh;
    SegmentStepper ss(p, hh);
    array<int,2> acc{};

    hh.setConfig(defaultConfig(2));
    hh.setAxisConfig(defaultAxisConfig(0));
    hh.setAxisConfig(defaultAxisConfig(1));

    ss.reloadJoints();

    for(StepperState const &ss: ss.getStepperStates()){
        cout << ss <<endl;
    }

    cout << endl << endl;

    p.move(40,{-1000, 5000});
    p.move(41, {-500, 10000});
    p.move(42, {1000, -15000});

    cout << " ============ " << endl;
    cout << p << endl;

    do {
        ss.next(dtime);

    } while (!p.empty());

    // Check that it doesn't crash after segments are exhausted.
    ss.next(dtime);
    ss.next(dtime);
    ss.next(dtime);

    hh.dumpPinCounts();

    REQUIRE(hh.highCount[3] == 2501);
    REQUIRE(hh.lowCount[3] == 2500);
    REQUIRE(hh.highCount[6] == 30000);
    REQUIRE(hh.lowCount[6] == 29999);

    cout << endl;

}

