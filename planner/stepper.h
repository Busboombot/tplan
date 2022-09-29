#pragma once
#include <utility>
#include <vector>
#include <array>
#include "util.h"
#include "Hardware.h"
#include "planner_types.h"

using namespace std;


class Planner;


struct StepperPhase{
    int x;
    double vi;
    double vf;
};

class StepperState {
private:

    int steps_left = 0;
    int steps_stepped = 0;
    int direction = 0;

    double dtime = 0; // a typical time delay between calling next()
    double t= 0;
    double t_f = 0;
    double phase_t = 0;
    double delay = 0;
    double delay_counter= 0;

    int clear_counter = 0;

    double a;

    bool done = false;

    int phase_n;
    int phases_left = 0;
    vector<StepperPhase> phases;
    const StepperPhase *phase; // Current phase.

    Stepper stepper;

public:
    StepperState(double dtime, Stepper stepper_) ;

    void loadPhases(vector<StepperPhase> phases);
    void loadPhases(array<StepperPhase,3> phases);

    void next_phase();

    int next(double dtime);

    inline int isDone() const { return done ? 1 : 0; };

    Stepper &getStepper();

    friend ostream &operator<<( ostream &output, const StepperState &ss );
};

class SegmentStepper {

public:

    explicit SegmentStepper(Planner &planner, Hardware& hardware);

    void reloadJoints();

    int next(double dtime);

    void clearSteps();

    int getActiveAxes() const { return activeAxes;}
    double getTime() const { return time; }

    uint32_t getLastCompleteSegmentNumber() const { return last_complete_segment;}

    const vector<StepperState> &getStepperStates() const;

    friend ostream &operator<<(ostream &output, const SegmentStepper &s);

private:

    Planner & planner;
    Hardware &hw;
    vector<StepperState> stepperStates;

    uint32_t last_complete_segment;




private:
    int activeAxes = 0;
    double time;

};