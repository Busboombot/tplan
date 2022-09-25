#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>

#include "trj_debug.h"

extern Stream &debug_serial;
char printf_buffer[1024];

// Set or clear externally to turn printing off and on
bool ser_printf_flag = true;

int debug_state_1 = LOW;
int debug_state_2 = LOW;
int debug_state_3 = LOW;
int debug_state_4 = LOW;


#if SER_PRINT_ENABLED
// Printf to the debug serial port
void ser_printf(const char* fmt, ...){

    if (!ser_printf_flag){
        return;
    }

    va_list args;
    va_start(args,fmt);
    vsprintf(printf_buffer, fmt,args);
    va_end(args);
    debug_serial.println(printf_buffer);
    debug_serial.flush();
}
#else
void ser_printf(const char* fmt, ...){}
#endif

