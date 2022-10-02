#include <sstream>
#include <functional>
#include <iostream>
#include <climits>
#include <vector>

#include "loop.h"
#include "config.h"
#include "types.h"


using namespace std;
const int UPDATE_TIMER = 1;
const int UPDATE_INTERVAL = 10; // milliseconds

const int STATE_LOG_TIMER = 2;
const int STATE_LOG_INTERVAL = 500; // milliseconds

void Loop::setup() {
    hw.setMillisZero(UPDATE_TIMER);
}

void Loop::loopOnce() {

    // Step is not using timer features b/c we also need dt
    tmicros t = hw.micros();
    tmicros dt = t - last_step_time;

    if (running && !empty) {

        auto activeAxes = ss.next(float(dt) / float(TIMEBASE));

        if (activeAxes == 0) {
            hw.signalSegmentComplete();
            pl.updateCurrentState(current_state);
            mp.update(t, current_state);
            mp.sendDone(ss.getLastCompleteSegmentNumber());

            if (pl.isEmpty()) {
                mp.sendEmpty(ss.getLastCompleteSegmentNumber());
                empty = true;
                hw.disableSteppers();
            }
        }

        last_step_time = t;
    }

    if (hw.everyMs(UPDATE_TIMER, UPDATE_INTERVAL)) {
        hw.update();
        mp.update(t, current_state);

        if (!mp.empty()) {
            processMessage(mp.firstMessage());
            mp.pop();
        }

        hw.blink(running, empty);
    }

    static string last;
    if (hw.everyMs(STATE_LOG_TIMER, STATE_LOG_INTERVAL)) {
        pl.updateCurrentState(current_state);
        stringstream strstr;
        strstr  << " running: " << (int) running << " empty: " << (int) empty << " " << current_state;

        if (last != strstr.str()) {
            last = strstr.str();
            log(last);
        }
    }


}

void Loop::processMessage(Message &m) {


    stringstream strstr;
    strstr << "Loop Message: " << m << endl;
    log(strstr);


    switch (m.header.code) {
        case CommandCode::CONFIG:
            setConfig(*m.asConfig());

            break;
        case CommandCode::AXES:
            setAxisConfig(*m.asAxisConfig());
            break;

        case CommandCode::RMOVE:
        case CommandCode::AMOVE:
        case CommandCode::JMOVE:
        case CommandCode::HMOVE:
            processMove(m);

            break;

        case CommandCode::STOP:
            running = false;
            break;

        case CommandCode::RUN:
            running = true;
            break;

        case CommandCode::RESET:
            break;

        case CommandCode::ZERO:
            break;

        case CommandCode::INFO:
            printInfo();
            break;

        default:
            // Should not be getting any other types.
            break;
    }
}

// Turn a move command into a move and add it to the planner
void Loop::processMove(Message &mesg) {

    PacketHeader ph = mesg.header;
    auto mv = mesg.asMoves();

    Move move(ph.seq, mv->segment_time, MoveType::none, MoveArray(mv->x, mv->x + config.n_axes));

    stringstream ss;
    ss << "Loop::processMove: " << move;
    log(ss);

    switch (ph.code) {
        case CommandCode::RMOVE:
            move.move_type = MoveType::relative;
            break;

        case CommandCode::HMOVE:
            move.move_type = MoveType::home;
            break;

        case CommandCode::AMOVE:
            move.move_type = MoveType::absolute;
            break;

        case CommandCode::JMOVE:
            move.move_type = MoveType::jog;
            break;

        default:;
    }

    /*
    for (int axis = 0; axis < config.n_axes; axis++){
        move.x[axis] = mv->x[axis];
        // FIXME! This position update will only work for relative moves
        current_state.planner_positions[axis] += mv->x[axis];
    }
    */

    empty = false;

    pl.move(move);

    pl.updateCurrentState(current_state);


}

void Loop::setJoints() {
    vector<Joint> joints;
    for (int i = 0; i < config.n_axes; i++) {
        joints.emplace_back(i, axes_config[i].v_max, axes_config[i].a_max);
    }

    pl.setJoints(joints);
}

void Loop::setConfig(const Config &config_) {
    config = config_;

    hw.setConfig(config);
    ss.reloadJoints();
}

void Loop::setAxisConfig(const AxisConfig &ac) {
    hw.setAxisConfig(ac);
    ss.reloadJoints();
    if (ac.axis < N_AXES) {
        axes_config[ac.axis] = ac;
    }
    setJoints();
}

/**************************/

/**
 * @brief Remove all of the segments from the queue
 * 
 */
void Loop::reset() {
    //sd.clear();
}

void Loop::zero() {
    //sd.zero();
}

void Loop::enable() {
    //sd.enable();
}

void Loop::disable() {
    //sd.disable();
}


#define rstss(ss) ss.str( std::string() ); ss.clear(); // reset the string stream

void Loop::printInfo() {

    stringstream ss;

    ss << *this;
    mp.sendMessage(ss);

    rstss(ss)
    ss << pl;
    mp.sendMessage(ss);

    rstss(ss)
    ss << config;
    mp.sendMessage(ss);

    rstss(ss)
    for (AxisConfig &as: axes_config) {
        ss << as << endl;
    }
    mp.sendMessage(ss);

    rstss(ss)
    ss << current_state;
    mp.sendMessage(ss);

}


ostream &operator<<(ostream &output, const Loop &p) {

    output << "[Loop " <<
           " run=" << (int) p.running <<
           " stop=" << (int) p.is_stopped <<
           " empty=" << (int) p.empty <<
           " lseg=" << (int) p.last_seg_num <<
           "]";

    return output;
}


