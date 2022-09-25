#define MESSAGE_BUF_SIZE 254
#define MAX_PAYLOAD_SIZE  (MESSAGE_BUF_SIZE-6) // 4 for header, 1 start, one zero at the end. 
#define N_AXES 6
#define INTERRUPT_DELAY 4 // 4 microseconds