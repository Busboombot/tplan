
#include <chrono>
#include <thread>
#include <unistd.h>
#include "util.h"
#include <iostream>
#include <string>
#include "col.h"
#include <vector>

using namespace std;

bool same_sign(float a, float b){
    return (a == 0) or (b == 0) or (sgn(a) == sgn(b));
}

int sign(int x) {
    if (x == 0) return 0;
    else if  (x > 0) return 1;
    else return -1;
}

int sign(double x) {
    if (x == 0) return 0;
    else if  (x > 0) return 1;
    else return -1;
}



vector<string> splitString(const string& str){
    vector<string> tokens;
 
    string::size_type pos = 0;
    string::size_type prev = 0;
    while ((pos = str.find('\n', prev)) != string::npos) {
        tokens.push_back(str.substr(prev, pos - prev));
        prev = pos + 1;
    }
    tokens.push_back(str.substr(prev));
 
    return tokens;
}


string yellow = col::make(col::yellow, col::def, false, false, false);
string green  = col::make(col::green,  col::def, false, false, false);
string blue  = col::make(col::blue,  col::def, false, false, false);
string blue_bg  = col::make(col::light_yellow,  col::light_blue, true, false, false);
string yelgr  = col::make(col::yellow,  col::dark_gray, true, false, false);
string creset = "\x1b[0m";
string overwrite = "\033[F";


uint8_t crc8(vector<u_int8_t> data){
    u_int8_t crc = 0xff;
    size_t i, j;
    for(u_int8_t d: data){
        crc ^= d;
        for (j = 0; j < 8; j++) {
            if ((crc & 0x80) != 0)
                crc = (uint8_t)((crc << 1) ^ 0x31);
            else
                crc <<= 1;
        }
    }
    return crc;
}


