
#include <cstdarg>
#include <cstdio>
#include <map>
#include "debug.h"
#include "types.h"

extern char printf_buffer[1024];

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

static std::map<CommandCode, string> cmdmap;

const std::map<CommandCode,string>& commandMap() {
    if(cmdmap.empty()) {
        cmdmap[CommandCode::ACK] =      "ACK";
        cmdmap[CommandCode::NACK] =     "NACK";
        cmdmap[CommandCode::DONE] =     "DONE";
        cmdmap[CommandCode::EMPTY] =    "EMPTY";
        cmdmap[CommandCode::RMOVE] =    "RMOVE";
        cmdmap[CommandCode::AMOVE] =    "AMOVE";
        cmdmap[CommandCode::JMOVE] =    "JMOVE";
        cmdmap[CommandCode::HMOVE] =    "HMOVE";
        cmdmap[CommandCode::VMOVE] =    "VMOVE";
        cmdmap[CommandCode::RUN] =      "RUN";
        cmdmap[CommandCode::STOP] =     "STOP";
        cmdmap[CommandCode::RESET] =    "RESET";
        cmdmap[CommandCode::ZERO] =     "ZERO";
        cmdmap[CommandCode::CONFIG] =   "CFG";
        cmdmap[CommandCode::AXES] =     "AXES";
        cmdmap[CommandCode::MESSAGE] =  "MESG";
        cmdmap[CommandCode::ERROR] =    "ERR";
        cmdmap[CommandCode::ECHO] =     "ECHO";
        cmdmap[CommandCode::DEBUG] =    "DEBUG";
        cmdmap[CommandCode::INFO] =     "INFO";
        cmdmap[CommandCode::QUEUE] =    "QUEUE";
        cmdmap[CommandCode::NOOP] =     "NOOP";

    }
    return cmdmap;
}



