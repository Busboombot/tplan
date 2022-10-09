
#include <sstream>
#include <functional>
#include <iostream>
#include <climits>
#include <vector>

#include "loop.h"
#include "config.h"
#include "types.h"
#include "Hardware.h"

#if defined(TRJ_ENV_ARDUINO) && defined(TRJ_DEBUG)
#include "Arduino.h"
#endif

using namespace std;
//const int UPDATE_TIMER  // Defined in Hardware.h
const int UPDATE_INTERVAL = 20; // milliseconds

// const int STATE_LOG_TIMER // Defined in Hardware.h
const int STATE_LOG_INTERVAL = 500; // milliseconds

//#define DEBUG_1 5
//#define DEBUG_2 6

void Loop::setup() {
    hw.setMillisZero(UPDATE_TIMER);
#if defined(TRJ_ENV_ARDUINO) && defined(TRJ_DEBUG)
    //hw.setPinMode(DEBUG_1, OUTPUT );
    //hw.setPinMode(DEBUG_2, OUTPUT );
#endif

}

float mean_dt = 0;

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
                current_state.flags.set((size_t)CSFLags::EMPTY, true);
                hw.disableSteppers();
            }
        }

        last_step_time = t;
    }

    if (hw.everyMs(UPDATE_TIMER, UPDATE_INTERVAL) ) {

        hw.update();

        mp.update(t, current_state);

        if (!mp.empty()) {
            processMessage(mp.firstMessage());
            mp.pop();
        }

        hw.blink(running, empty);

        empty = pl.isEmpty();

        current_state.flags.set((size_t)CSFLags::EMPTY, empty);
        current_state.flags.set((size_t)CSFLags::RUNNING, running);
    }

    static string last;
    if (hw.everyMs(STATE_LOG_TIMER, STATE_LOG_INTERVAL)) {

        pl.updateCurrentState(current_state);
        if (config.debug_print) {
            stringstream strstr;
            strstr << " running: " << (int) running << " empty: " << (int) empty << " " << current_state;

            if (last != strstr.str()) {
                last = strstr.str();
                log(last);
            }
        }

    }
}

void Loop::processMessage(Message &m) {

    if (config.debug_print) {
        stringstream ss;
        ss << "Loop::processMessage " << m << current_state << endl;
        log(ss);
    }

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
        case CommandCode::VMOVE:
            processMove(m);
            break;

        case CommandCode::STOP:
            running = false;
            break;

        case CommandCode::RUN:
            running = true;
            break;

        case CommandCode::RESET:
            processReset();
            break;

        case CommandCode::ZERO:
            pl.zero();
            break;

        case CommandCode::SET:
            pl.setPositions(MoveVector(m.asMoves()->x, m.asMoves()->x+(size_t)N_AXES));
            break;

        case CommandCode::INFO:
            printInfo();
            break;

        case CommandCode::QUEUE:
            printQueue();
            break;

        default:
            // Should not be getting any other types.
            break;
    }
}

// Turn a move command into a move and add it to the planner
void Loop::processMove(Message &mesg) {

    PacketHeader ph = mesg.header;
    MoveType mt;
    auto mvp = mesg.asMoves();
    auto mv = MoveVector(mvp->x, mvp->x + config.n_axes);

    switch (ph.code) {
        case CommandCode::RMOVE:
            // Do nothing, move is OK as is.
            mt = MoveType::relative;
            break;

        case CommandCode::HMOVE:
            mt = MoveType::home;
            break;

        case CommandCode::AMOVE:
            mv -= pl.getPlannerPosition();
            mt = MoveType::absolute;
            break;

        case CommandCode::JMOVE:
            pl.truncateTo(3);
            mt = MoveType::velocity;
            break;

        case CommandCode::VMOVE:
            mt = MoveType::velocity;
            break;

        default:;
        mt = MoveType::none;
    }

    Move move(ph.seq, mvp->segment_time, MoveType::none, mv );

    move.move_type = mt;

    empty = false;
    current_state.flags.set((size_t)CSFLags::EMPTY, false);

    if (move.move_type == MoveType::velocity){
        pl.vmove(move);
    } else {
        pl.move(move);
    }

    pl.updateCurrentState(current_state);

}

void Loop::processReset(){
    pl.reset(running);
};

void Loop::processSet(Message& message){

    if (message.header.code == CommandCode::ZERO){

    } else {

    }

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

void Loop::printQueue() {


    size_t n = 30;
    for (auto &s: pl.getSegments()){
        stringstream ss;
        ss <<  s << endl;
        log(ss);
        if(n-- == 0){
            break;
        }
    }
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


