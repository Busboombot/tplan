#include <Arduino.h>
#include <vector>
#include "blink.h"
#include "trj_const.h"

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

std::vector<int> pins{LED_BUILTIN,EMPTY_PIN, RUNNING_PIN, Y_SIG_PIN,B_SIG_PIN };

/**
 * @brief Initial cycle through the LEDs to demonstrate that they are working
 * 
 */
void cycleLeds(){

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