#include "planner.h"
#include <tuple>
#include <utility>
#include "stepper.h"
#include <iostream>
#include "messageprocessor.h" // For logf

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

/**
 * @brief Return the expected length of the phase, calculated from the
 * distance and accelerations.
 * @param phase
 * @return
 */
double phase_ttf(const StepperPhase *phase){
    return (phase->vi + phase->vf) != 0 ? fabs((2.f * (double) abs(phase->x)) / (phase->vi + phase->vf)) : 0;
}

// Calc time from time-to-finish
double segment_ttf(const array<StepperPhase,3>& phases){
    return phase_ttf(&phases[0])+phase_ttf(&phases[1])+phase_ttf(&phases[2]);
}

// Calc time from phase times
double segment_t(const array<StepperPhase,3>& phases){
    return phases[0].t + phases[1].t + phases[2].t;
}



void StepperState::nextPhase() {
    phase = &phases[phase_n];

    direction = sign(phase->x);
    steps_left = abs(phase->x);

    phase_t = 0;
    done = false;

    // Expected time to finish.
    t_f = phase_ttf(phase);
    a = t_f != 0 ? (phase->vf - phase->vi) / t_f : 0;


    double v = a * dtime + phase->vi;
    delay = (v != 0) ? fabs(1 / v) : 0;
    delay_counter += dtime;
    clear_timer = 0;

    stepper.setDirection(direction);

    phase_n += 1;
    phases_left -= 1;
    _printed_done = false;
}

inline void StepperState::clearStep(){

    if (step_val == HIGH){
        stepper.clearStep();
        step_val = LOW;
    }

};

int StepperState::next(double dtime) {

    if (steps_left <= 0) {
        if (done or phases_left == 0) {
            done = true;
            if( !_printed_done){
                //cerr << "StepperState::next      "<<phase_t<<" "<<t_f << " | "<< x_err << endl;
                _printed_done = true;
            }
            return 0;
        } else {
            nextPhase();
        }
        clearStep();
    }

    if (delay_counter > delay and steps_left > 0) {
        // else if so if the step gets cleared , it doesn't get immediately re-set,
        // to soon for the stepper driver to notice.
        delay_counter -= delay;
        steps_left -= 1;
        steps_stepped += 1;
        step_val = HIGH;
        stepper.setStep();

        clear_timer = phase_t + PULSE_WIDTH;

    } else if (clear_timer != 0 && phase_t > clear_timer) {
        clearStep();
        clear_timer = 0;
    }

    double v = phase->vi + a * phase_t;

    delay = v != 0 ? abs(1 / v) : 1;

    delay_counter += dtime;
    phase_t += dtime;

    next_calls += 1;

    //calc_x = (a * pow(phase_t, 2))/2 + phase->vi*phase_t;
    //x_err = steps_stepped - calc_x;

    return 1;
}

Stepper &StepperState::getStepper() {
    return stepper;
}

SegmentStepper::SegmentStepper(Planner &planner, Hardware &hardware) : planner(planner), hw(hardware) {
    reloadJoints();
}

void SegmentStepper::reloadJoints() {

    stepperStates.clear();

    for (const Joint &j: planner.getJoints()) {
        int jn = j.n;
        stepperStates.emplace_back(hw.getDTime(), hw.getStepper(jn));
    }
}

int SegmentStepper::next(double dtime) {

    time += dtime;

    if (planner.isEmpty()) {
        return 0;
    }

    // Run the steppers through the next time step
    activeAxes = 0;
    if (current_segment != nullptr) {
        for (StepperState &s: stepperStates) {
            activeAxes += s.next(dtime);
        }
    }
    last_active_axes = activeAxes;

    // We were running a segment, but all the axes are now done,
    // so we can pop off the front segment, in perparation for
    // the next one
    if (current_segment != nullptr && activeAxes == 0) {
        last_complete_segment = planner.getFront().getN();
        planner.popFront();
        current_segment = nullptr;
    }

    // There is no current segment, and the queue isn't empty, so load the next
    // segment
    if (current_segment == nullptr && !planner.isEmpty()) {
        current_segment = &planner.getFront();

        auto bi = current_segment->blocks.begin();

        //cerr << "SegmentStepper::next #"<< current_segment->getN()<<endl;
        for (StepperState &ss: stepperStates) {
            if (bi != current_segment->blocks.end()) {
                auto phases = bi->getStepperPhases();
                ss.loadPhases(phases);
                 //cerr << " t=(" <<segment_ttf(phases)<<","<<segment_t(phases)<<") "<<endl;
                bi++;
            }

            ss.getStepper().enable();
        }

    }

    return (int) activeAxes;
}

void SegmentStepper::clearSteps() {
    for (StepperState &ss: stepperStates) {
        ss.getStepper().clearStep();
    }
}

vector<StepperState> &SegmentStepper::getStepperStates() {
    return stepperStates;
}

void SegmentStepper::enable() {
    for (StepperState &ss: stepperStates) {
        ss.getStepper().enable();
    }
}

void SegmentStepper::disable() {
    for (StepperState &ss: stepperStates) {
        ss.getStepper().disable();
    }
}

ostream &operator<<(ostream &output, const SegmentStepper &s) {
    output << "[SegStep last=" << (int) s.getLastCompleteSegmentNumber() << " t=" << s.getTime() << "]";
    return output;
}

ostream &operator<<(ostream &output, const Stepper &s) {
    output << "[Stepper #"
           << (int) s.axis << " "
           << (int) s.step_pin << " "
           << (int) s.direction_pin << " "
           << (int) s.enable_pin << " "

           << " adr=" << (void *) &s
           << " ]";
    return output;
}

ostream &operator<<(ostream &output, const StepperState &ss) {
    output << "[StepSt " << (int) ss.stepper.axis <<
           " sp=" << (int) ss.stepper.step_pin <<
           " dp=" << (int) ss.stepper.direction_pin <<
           " sl=" << ss.steps_left <<
           " ss=" << ss.steps_stepped <<
           " ssadr=" << (void *) &ss <<
           " stadr=" << (void *) &ss.stepper <<
           "]";

    return output;
}


