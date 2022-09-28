#include "planner.h"
#include <tuple>
#include <utility>
#include "stepper.h"
#include <iostream>

StepperState::StepperState(double dtime_, Stepper stepper_) : dtime(dtime_), stepper(stepper_) {

}


void StepperState::loadPhases(vector<StepperPhase> phases_) {

    phases = std::move(phases_);
    phases_left = phases.size();
    phase_n = 0;
    done = false;
}

void StepperState::loadPhases(array<StepperPhase, 3> phases_) {
    loadPhases(vector<StepperPhase>(phases_.begin(), phases_.end()));

}


void StepperState::next_phase() {
    phase = &phases[phase_n];

    direction = sign(phase->x);
    steps_left = abs(phase->x);

    t_f = (phase->vi + phase->vf) != 0 ? fabs((2.f * (double) steps_left) / (phase->vi + phase->vf)) : 0;
    a = t_f != 0 ? (phase->vf - phase->vi) / t_f : 0;

    phase_t = 0;

    double v = a * dtime + phase->vi;
    delay = v != 0 ? fabs(1 / v) : 0;
    delay_counter += dtime;

    done = false;

    phase_n += 1;
    phases_left -= 1;

    stepper.setDirection(direction);

}

int StepperState::next(double dtime) {

    if (steps_left <= 0) {
        if (done or phases_left == 0) {
            done = true;
            return 0;
        } else {
            next_phase();
        }
    }

    if (delay_counter > delay) {
        delay_counter -= delay;
        steps_left -= 1;
        steps_stepped += 1;

        stepper.setStep();

    } else {
        stepper.clearStep();
    }

    double v = phase->vi + a * phase_t;

    delay = v != 0 ? abs(1 / v) : 1;
    delay_counter += dtime;

    t += dtime;
    phase_t += dtime;

    return 1;
}


Stepper StepperState::getStepper() {
    return stepper;
}





SegmentStepper::SegmentStepper(Planner &planner, Hardware &hardware) : planner(planner), hw(hardware) {
    reloadJoints();
}

void SegmentStepper::reloadJoints(){
    auto steppers = hw.getSteppers();

    stepperStates.clear();

    for (const Joint &j: planner.getJoints()) {
        int jn = j.n;
        stepperStates.emplace_back(hw.getDTime(), hw.getStepper(jn));
    }
}

int SegmentStepper::next(double dtime) {

    time += dtime;

    if (activeAxes == 0 && !planner.segments.empty()) {

        Segment &seg = planner.segments.front();

        auto bi = seg.blocks.begin();
        for(StepperState &ss: stepperStates){
            if(bi != seg.blocks.end()){
                ss.loadPhases(bi->getStepperPhases());
                bi++;
            }
        }

    }

    activeAxes = 0;
    for (StepperState &s: stepperStates) {
        activeAxes += s.next(dtime);
    }

    if (activeAxes == 0 && !planner.segments.empty()) {
        planner.segments.pop_front();
        hw.signalSegmentComplete();
    }

    return (int) activeAxes;

}

const vector<StepperState> &SegmentStepper::getStepperStates() const {
    return stepperStates;
}

ostream &operator<<(ostream &output, const Stepper &s) {
    output << "[Stepper #"
           << (int)s.axis << " "
           << (int)s.step_pin << " "
            << (int)s.direction_pin << " "
           << (int)s.enable_pin << " "

           << " adr=" << (void*)&s
           << " ]";
    return output;
}

ostream &operator<<( ostream &output, const StepperState &ss ) {
    output << "[StepSt " << (int)ss.stepper.axis <<
            " sp=" << (int)ss.stepper.step_pin <<
            " dp=" << (int)ss.stepper.direction_pin <<
            " sl=" << ss.steps_left <<
            " ss=" << ss.steps_stepped <<
            " ssadr=" << (void*)&ss <<
            " stadr=" << (void*)&ss.stepper <<
            "]";

    return output;
}