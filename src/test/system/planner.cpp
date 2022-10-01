/*
 * Read an input file that describe joints and moves, then print out the planned
 * moves, or the steps for those moves.
 */

#include <string>
#include <iostream>
#include <vector>
#include <sstream>
#include <exception>
#include <chrono>
#include <memory>

#include <boost/program_options.hpp>

#include "joint.h"
#include "planner.h"
#include "stepper.h"
#include "HostHardware.h"

using namespace std;
namespace po = boost::program_options;

#define csdc(p) ( std::dynamic_pointer_cast<CoutStepper>(p))


std::vector<int> extractIntegerWords(string str)
{
    stringstream ss(str);
    std::vector<int> result;

    string temp;
    int found;
    while (!ss.eof()) {

        ss >> temp;

        if (stringstream(temp) >> found)
            result.push_back(found);

        temp = "";
    }

    return result;
}

using Ints = vector<int>;

void loadData(vector<Joint> &joints, vector<Ints> &moves){

    int n_joints;
    int line_n = 0;


    for (std::string line; std::getline(std::cin, line);) {
        Ints ints = extractIntegerWords(line);
        if (line_n == 0) {
            n_joints = ints[0];
        } else if (line_n <= n_joints) {
            joints.emplace_back(line_n-1, ints[0],ints[1]);
        } else {
            moves.push_back(ints);
        }
        line_n += 1;
    }

}

Planner *makePlanner(vector<Joint> &joints, vector<Ints> &moves){

    Planner *planner = new Planner(joints);
    for(Ints &m : moves) {
        planner->move(0,m);
    }
    return planner;
}

void runSteppers(Planner &p, ostream &os, bool drop){

    double dtime = 5./1e6; // 5 us

    HostHardware hh;
    SegmentStepper ss(p, hh);

    auto steps = vector<int>(p.getJoints().size());

    double time = 0;
    string step_string(p.getJoints().size()*2, ' ');
    int sm;

    do {
        ss.next(dtime);

        sm = 0;
        auto si = step_string.begin();
        for(StepperState &stst : ss.getStepperStates()){
            Stepper &stp = stst.getStepper();
            si++;
            *si = stp.getStepState() ? '1' : '0';
            si++;
            sm += stp.getStepState();
        }

        if(!drop || sm > 0 ) {
            os << time << " " << step_string << endl;
        }

        time += dtime;
    } while (!p.empty());

}

int main(int ac, char **av) {

    vector<Joint> joints;
    vector<Ints> moves;

    po::options_description desc("Allowed options");
    desc.add_options()
            ("help,h", "produce help message")
            ("planner,p",  "Load moves into the planner and print it. ")
            ("stepper,s",  "Load moves into the planner run steppers ")
            ("drop,d",  "When stepping, don't print lines that are all zero ")
            ("json,j",  "Output JSON ")
            ;

    po::variables_map vm;
    po::store(po::parse_command_line(ac, av, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
        cout << desc << endl;
        return 1;
    }

    loadData(joints, moves);

    auto start = chrono::steady_clock::now();
    Planner *planner = makePlanner(joints, moves);
    auto end = chrono::steady_clock::now();
    auto diff = chrono::duration_cast<chrono::microseconds>(end - start);

    if (!vm.count("json") and !vm.count("stepper")){
        cout << "Processed moves in " << diff.count() << "μs" << endl;
    }

    if(vm.count("planner")){
        if (vm.count("json")){
            json j = planner->dump();
            j["_time"] = diff.count();
            cout << j << endl;
        } else {
            cout << *planner << endl;
            for(const Segment &s: planner->getSegments()){
                cout << s << endl;
            }
        }
    } else if(vm.count("stepper")){
        if (vm.count("json")){

        } else {
            runSteppers(*planner, cout, vm.count("drop")>0);
        }
    }


    return 0;
}

