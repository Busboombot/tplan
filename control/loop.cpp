#include <sstream>
#include <functional>
#include <iostream>
#include <limits.h>
#include <vector>

#include "loop.h"
#include "trj_move.h"
#include "messageprocessor.h"

#include "trj_config.h"
#include "trj_debug.h"

using namespace std;

void Loop::setup(){


}

#define SIGNAL_INTERVAL 500

void Loop::loopOnce(){
    static tmillis last_step_time = hw.millis();
    static tmillis last_signal_time = hw.millis();

    tmillis t = hw.millis();

    tmillis dt = t-last_step_time;

    // The hardware update clears signal pins, so it should go first, so
    // the signals will be on for a while.
    hw.update();

    if (t-last_signal_time > SIGNAL_INTERVAL){
        hw.setEmptyLed(pl.empty());
        hw.setRunningLed(running);
    }

    mp.update(t, current_state);

    if(dt >= config.interrupt_delay) {
        ss.next(dt);
    }

    if(!mp.empty()){
        processMessage(mp.firstMessage());
        mp.pop();
    }

    last_step_time = t;

    /*
    // Blink the LED and toggle a debugging pin, to show we're not crashed.

    if (running){
      sd.update(); // Update the steppers

      int seq = sd.checkIsDone();
      if (seq >=0){

          signalSegmentComplete();
      }

      if (sd.checkIsEmpty()){
        disable();
        sdp.sendEmpty(sd.getPlanner().getCurrentPhase().seq, current_state);
      }
    } else {

    }

    if(limitChanges > 0){
      if(sd.getPlanner().getCurrentMoveType() == Move::MoveType::home){
        reset();
      }
      limitChanges = 0;
    }
     */

}

void Loop::processMessage(Message &m) {


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
            break;

        case CommandCode::RMOVE:
        case CommandCode::AMOVE:
        case CommandCode::JMOVE:
        case CommandCode::HMOVE:
            processMove(m);
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

    /*
    auto &planner = sd.getPlanner();
    current_state.queue_length = planner.getQueueSize();
    current_state.queue_time = planner.getQueueTime();

    sd.push(move);
     */
}


/**************************/


//void clearSegmentCompleteISR(){ mainLoop.clearSegmentComplete();}

void limitChangedISR() {
  //mainLoop.limitChanged();
}






// Start the timers, if there are segments available. 
void Loop::start(){ 
  running = true;
  
}

void Loop::stop(){ 
  running = false;
}

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




void Loop::printInfo(){
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

