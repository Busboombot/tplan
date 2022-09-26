#include <Arduino.h>

#include <sstream>
#include <functional>
#include <iostream>
#include <limits.h>
#include <vector>

#include <avr/io.h>
#include <avr/interrupt.h>

#include "trj_loop.h"
#include "trj_move.h"
#include "trj_messageprocessor.h"
#include "trj_fastset.h"
#include "trj_bithacks.h"
#include "trj_config.h"
#include "trj_debug.h"

using namespace std;

extern Loop  mainLoop;

CurrentState current_state;

void clearSegmentCompleteISR(){ mainLoop.clearSegmentComplete();}

void limitChangedISR() {
  mainLoop.limitChanged();
  }

inline void Loop::signalSegmentComplete(){

  for(int i = 0; i < config.n_axes; i++){
    current_state.positions[i] = sd.getState(i).getPosition();
  }

  auto& planner = sd.getPlanner();

  current_state.queue_length = planner.getQueueSize();
  current_state.queue_time = planner.getQueueTime();

  sdp.sendDone(planner.getCurrentPhase().seq, current_state);

  if(config.segment_complete_pin > 0){
    digitalWriteFast(config.segment_complete_pin, HIGH);
    segmentCompleteTimer.begin(clearSegmentCompleteISR,4); 
  }
}

inline void Loop::clearSegmentComplete(){
  if(config.segment_complete_pin > 0){
    digitalWriteFast(config.segment_complete_pin, LOW);
  }
  segmentCompleteTimer.end(); 
}

void Loop::limitChanged(){
  limitChanges++;
}


#define PATTERN_SIZE 20
#define BASE_DELAY 2000/PATTERN_SIZE // Pattern runs over 2,000 ms

int blink_patterns[4][PATTERN_SIZE] = {
  {1,0,1,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0},  // !empty & !running: 4 fast blinks
  {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,0},  // !empty & running: continuous fast blink
  {1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0},  // empty  & !running: long, slow blink
  {1,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0},  // empty  & running: 1 per second
};

/**
 * @brief Blink the onboard LED and set the Empty and Running LEDs. 
 * 
 * @param running 
 * @param empty 
 */
void blink(bool running, bool empty){

  static int pattern_index = 0;
  static int db_print = 0;
  static unsigned long last = millis();

  int pattern = ((int)empty)<<1 | ((int)running);

  digitalWrite(EMPTY_PIN, empty);
  digitalWrite(RUNNING_PIN, running);

  if( (millis() - last) > BASE_DELAY  ){
    
    pattern_index = (pattern_index+1)%PATTERN_SIZE;
    digitalWrite(LED_BUILTIN, blink_patterns[pattern][pattern_index] );
    last = millis();
    db_print++;
  }
}

/* Run one iteration of the main loop
*/


void cycleLeds(){

  vector<int> pins{LED_BUILTIN,EMPTY_PIN, RUNNING_PIN, Y_SIG_PIN,B_SIG_PIN };
  bool tog = true;
  // Cycle thorugh the LEDs
  for (int i = 0; i < 4; i++){
    for(int p: pins){
      digitalWrite(p, tog);
      delay(75);
    }
    tog = !tog;

  }
}
void Loop::setup(){

  cycleLeds();

  start_usince(); // set the initial time for usince();
}

void Loop::loopOnce(){
    // Blink the LED and toggle a debugging pin, to show we're not crashed. 
    
    blink(running, sd.isEmpty());

    sdp.update();  // Get serial data and update queues 
    
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
  sd.clear();
}

void Loop::zero(){
  sd.zero();
}

void Loop::enable() { 
    sd.enable();
}
void Loop::disable() {
    sd.disable();
}

void Loop::setConfig(Config* config_){
  

  config = *config_;

  if (config.segment_complete_pin > 0){
    pinMode(config.segment_complete_pin, OUTPUT);
  }

  if (config.limit_pin > 0){
    pinMode(config.limit_pin, INPUT);
    attachInterrupt(config.limit_pin, limitChangedISR, RISING);
  }

  sd.setNAxes(config.n_axes);
  sd.setPeriod(config.interrupt_delay);
}

/**
 * @brief Configure a stepper for an axis. Creates a new StepperInterface object for the axis
 * 
 * @param as Axis configuration object
 * @param eeprom_write If true, write positions to the eeprom. 
 */
void Loop::setAxisConfig(AxisConfig* as){

  if(as->axis < config.n_axes){
    sd.setAxisConfig(as->axis, as->v_max, as->a_max);
    if ( steppers[as->axis] != nullptr){
      delete steppers[as->axis];
    }

    steppers[as->axis] = new StepDirectionStepper(*as);

    sd.setStepper(as->axis, steppers[as->axis]);
  
    axes_config[as->axis] = *as;
  }
}

// Turn a move command into a move and add it to the planner
void Loop::processMove(const uint8_t* buffer_, size_t size){

    PacketHeader *ph = (PacketHeader*)buffer_;
    Moves *m = (Moves*)(buffer_ + sizeof(PacketHeader));
  
    Move move(getConfig().n_axes, ph->seq, m->segment_time, 0);


    switch(ph->code){
        case CommandCode::RMOVE:
        move.move_type = Move::MoveType::relative;
        break;

        case CommandCode::HMOVE:
        move.move_type = Move::MoveType::home;
        break;
        
        case CommandCode::AMOVE:
        move.move_type = Move::MoveType::absolute;
        break;
        
        case CommandCode::JMOVE:
        move.move_type = Move::MoveType::jog;
        break;
        
        default: ; 
    }

    for (int axis = 0; axis < getConfig().n_axes; axis++){
      move.x[axis] = m->x[axis];
      // FIXME! This position update will only work for relative moves
      current_state.planner_positions[axis] += m->x[axis];
    }
    
    auto &planner = sd.getPlanner();
    current_state.queue_length = planner.getQueueSize();
    current_state.queue_time = planner.getQueueTime();

    sd.push(move);
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
