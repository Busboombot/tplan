#define MESSAGE_BUF_SIZE 254
#define MAX_PAYLOAD_SIZE  (MESSAGE_BUF_SIZE-6) // 4 for header, 1 start, one zero at the end. 
#define N_AXES 6
#define INTERRUPT_DELAY 4 // 4 microseconds
#define TIMEBASE 1000000 // 1m microseconds per second
// LEDS
#define EMPTY_PIN 30 // RED
#define RUNNING_PIN 28 // GREEN
#define Y_SIG_PIN 26 // YELLOW
#define B_SIG_PIN 31 // BLUE


#define MAIN_SERIAL_BAUD 115200

#define SERIAL1_IS_DEBUG true
#define DEBUG_SERIAL_BAUD 115200

#ifndef LOW
#define LOW 0
#define HIGH 1
#endif

