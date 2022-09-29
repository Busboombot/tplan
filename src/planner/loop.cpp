#include <sstream>
#include <functional>
#include <iostream>
#include <climits>
#include <vector>

#include "loop.h"
#include "config.h"
#include "types.h"

#ifdef TRJ_ENV_HOST

#endif

using namespace std;
const int UPDATE_TIMER = 1;
const int UPDATE_INTERVAL = 10; // milliseconds

void Loop::setup(){
    hw.setMillisZero(UPDATE_TIMER);
}

void Loop::loopOnce(){

    // Step is not using timer features b/c we also need dt
    tmicros t = hw.micros();
    tmicros dt = t-last_step_time;

    if(running && !empty ) {

        auto activeAxes = ss.next(float(dt)/float(TIMEBASE) );

        if(activeAxes == 0){
            hw.signalSegmentComplete();
            cout << "Send Done" << endl;
            mp.sendDone(ss.getLastCompleteSegmentNumber());

            if(pl.isEmpty()){
                cout << "Send Empty" << endl;
                mp.sendEmpty(ss.getLastCompleteSegmentNumber());
                empty = true;
            }
        }

        last_step_time = t;
    }

    if(hw.millisSince(UPDATE_TIMER) > UPDATE_INTERVAL){

        hw.update();
        hw.setRunningLed(running);

        mp.update(t, current_state);

        if(!mp.empty()) {
            processMessage(mp.firstMessage());
            mp.pop();
        }

        hw.blink(running, empty);

        hw.setMillisZero(UPDATE_TIMER);
    }
}

void Loop::processMessage(Message &m) {

#ifdef TRJ_ENV_HOST
    stringstream strstr;
    strstr << "Loop Message: " << m << endl;
    mp.sendMessage(strstr);
#endif

    switch(m.header.code){
        case CommandCode::CONFIG:
            config = *m.asConfig();
            hw.setConfig(config);
            ss.reloadJoints();
            break;
        case CommandCode::AXES:
            hw.setAxisConfig(*m.asAxisConfig());
            ss.reloadJoints();
            if(m.asAxisConfig()->axis <N_AXES){
                axes_config[m.asAxisConfig()->axis] = *m.asAxisConfig();
            }
            setJoints();
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
            break;

        default:
            // Should not be getting any other types.
            break;
    }
}

// Turn a move command into a move and add it to the planner
void Loop::processMove(Message &mesg){

    PacketHeader ph = mesg.header;
    auto mv = mesg.asMoves();

    Move move(config.n_axes, ph.seq, mv->segment_time, 0);

    switch(ph.code){
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

        default: ;
    }

    for (int axis = 0; axis < config.n_axes; axis++){
        move.x[axis] = mv->x[axis];
        // FIXME! This position update will only work for relative moves
        current_state.planner_positions[axis] += mv->x[axis];
    }

    empty = false;

    current_state.queue_length = pl.getQueueSize();
    current_state.queue_time = pl.getQueueTime();

    pl.move(move);


}

void Loop::setJoints(){
    vector<Joint> joints;
    for(int i = 0; i < config.n_axes; i++){
        joints.emplace_back(i, axes_config[i].v_max,  axes_config[i].a_max);
    }

    pl.setJoints(joints);
}

/**************************/

/**
 * @brief Remove all of the segments from the queue
 * 
 */
void Loop::reset(){
  //sd.clear();
}

void Loop::zero(){
  //sd.zero();
}

void Loop::enable() { 
    //sd.enable();
}
void Loop::disable() {
    //sd.disable();
}



#define rstss(ss) ss.str( std::string() ); ss.clear(); // reset the string stream

void Loop::printInfo(){

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
    for(AxisConfig &as : axes_config){
        ss << as <<endl;
    }
    mp.sendMessage(ss);

    rstss(ss)
    ss << current_state;
    mp.sendMessage(ss);

  /*



  auto& planner = sd.getPlanner();

  sdp.printf("===== Configuration ======\n"
            "Queue Size : %d\r\n"
            "Queue Time : %d\r\n"
            "Running    : %d\r\n"
            "N Axes     : %d\r\n"
            "Intr Delay : %d\r\n"
            "Seg Pin    : %d\r\n"
            "Lim Pin    : %d\r\n"
            "Debug print: %d\r\n"
            "Debug tick : %d\r\n",
           planner.getQueueSize(), planner.getQueueTime(), running, 
           config.n_axes, config.interrupt_delay, 
           config.segment_complete_pin, config.limit_pin,
           config.debug_print, config.debug_tick) ;
  
  for( int i = 0 ; i < config.n_axes; i++){
    Stepper *s = getStepper(i);
    AxisConfig as = s->getConfig();
    StepperState &state = sd.getState(i);
    const Joint &j = planner.getJoint(i);


    sdp.printf("-- Axis %d \r\n"
            "SDE        : %d %d %d\r\n"
            "Hi Val     : %d %d %d\r\n"
            "Out Mode   : %d %d %d\r\n"
            "A V Max    : %d %d\r\n"
            "Position   : %d\r\n",
            j.n,
            as.step_pin, as.direction_pin, as.enable_pin, 
            as.step_high_value, as.direction_high_value, as.enable_high_value, 
            as.step_output_mode, as.direction_output_mode, as.enable_output_mode,
            static_cast<int>(j.a_max), static_cast<int>(j.v_max),
            state.getPosition()); 
  }
  
  stringstream ss;
  ss << sd << endl;
  
  std::string line;
  while (std::getline(ss, line, '\n')) {
    sdp.sendMessage(line.c_str());
  }
*/
}


ostream &operator<<(ostream &output, const Loop &p) {

    output << "[Loop " <<
           " run=" << (int)p.running <<
           " stop=" << (int)p.is_stopped <<
           " empty=" << (int)p.empty <<
           " lseg=" << (int)p.last_seg_num <<
           "]";

    return output;
}


