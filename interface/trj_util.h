#pragma once

#include <vector>
#include <iostream>
#include <iomanip>
#include <cstdint> 
#include <math.h> 
#include <chrono>
#include <stdexcept>
#include <vector>

using namespace std;

typedef chrono::milliseconds ms;
typedef chrono::microseconds us;
typedef chrono::steady_clock steadyClock;
typedef chrono::duration<uint64_t, micro> duration;



long micros();
void start_usince();
uint32_t usince();

#ifdef TRJ_ENV_HOST
void delay(uint32_t ms);
void delayMicroseconds(uint32_t us);
#endif

extern int here_count;
#define HERE(x) cout << "!!!HERE!!! " << x << " " << here_count++ << endl;

// Convert seconds to ticks, usually microseconds
#define SEC_TO_TICKS(v) (static_cast<int>(rint(v*1e6)))

using IntVec = vector<int32_t> ;

// Return -1 , 0  or 1 to indicate the sign of the number. 
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

bool same_sign(float a, float b);
int sign(int a);
int sign(double a);

// To convert class enums to ints. 
//template <typename E>
//constexpr typename underlying_type<E>::type to_underlying(E e) noexcept {
//    return static_cast<typename underlying_type<E>::type>(e);
//};

vector<string> splitString(const string& str);


// ANSI colors
extern string yellow;
extern string green;
extern string blue;
extern string blue_bg;
extern string yelgr;
extern string creset;
extern string overwrite;

uint8_t crc8(vector<u_int8_t> data);