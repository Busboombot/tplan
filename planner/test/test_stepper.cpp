#include <iostream>
#include <memory>
#include <vector>
#include <numeric>
#include <fstream>

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

extern Config defaultConfig(uint8_t n_axes);
extern AxisConfig defaultAxisConfig(uint8_t axis);

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

    cout << endl;

}

/* Read moves from a file */
TEST_CASE("Stepper File Test", "[stepper]") {

    return; // This test is really expensive.

    double dtime = 5. / 1e6; // 5 us

    vector<Joint> joints{
            Joint(0, 5e3, 50e3),
            Joint(1, 5e3, 50e3),
            Joint(2, 5e3, 50e3)
    };
    Planner p(joints);
    HostHardware hh;
    hh.setPrintPins(true);


    fstream inputFile;
    array<int, 3> counts = {0};
    path inputFilePath = current_path().parent_path().parent_path().parent_path() / "test_data" / "stepper_file_test.txt";
    inputFile.open( inputFilePath.string(), ios::in);

    if (inputFile.is_open()) {
        cout << "Loading  "<< inputFilePath << endl;
        int line_no = 0;
        for (std::string line; std::getline(inputFile, line);) {
            if (line[0] == ' ' || line[0] == '#') continue;
            Ints ints = extractIntegerWords(line); // Get all integers on a line
            p.move(line_no++, {ints[0], ints[1], ints[2]});

            for(int i=0; i < 3; i++) counts[i]+=ints[i];
        }
        inputFile.close(); //close the file object

        cout << "Loaded " << p.getQueueSize() << " moves,  Counts: "; for(int i=0; i < 3; i++) cout <<counts[i]<<" "; cout << endl;


    } else {
        cout << "Err: not opened:  "<< inputFilePath << endl;
        REQUIRE(false);
    }

    //
    // Run the steppers
    //

    SegmentStepper ss(p, hh);

    hh.setConfig(defaultConfig(3));
    hh.setAxisConfig(defaultAxisConfig(0));
    hh.setAxisConfig(defaultAxisConfig(1));
    hh.setAxisConfig(defaultAxisConfig(2));
    ss.reloadJoints();

    cout << endl <<  endl ;;

    auto start = chrono::steady_clock::now();

    int n_iter = 0;
    do {
        ss.next(dtime);
        if (++n_iter % 5'000'000 == 0){
            auto end  = chrono::steady_clock::now();
            auto diff = chrono::duration_cast<chrono::microseconds>(end - start);
            //cout << "Periods: "<< ss.getTotalPeriods()<<" ("<<double(diff.count()/double(ss.getTotalPeriods()))<<")" <<
            //"us/p Time: "<<ss.getTime() << " sec "<< endl;
        }
    } while (!p.empty());


    // Check that it doesn't crash after segments are exhausted.
    ss.next(dtime);
    ss.next(dtime);
    ss.next(dtime);

    /*
    cout << "Final: 1:"<< csdc(steppers[0])->count<<
            " 2: "<< csdc(steppers[1])->count <<
            " 3: "<< csdc(steppers[2])->count << endl;

    cout << "Total Periods: "<< ss.getTotalPeriods()<<" Time: "<<ss.getTime() << " sec "<< endl;

    cout << endl;
     */


}

