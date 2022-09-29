#pragma once

#include <vector>
#include <iostream>
#include <iomanip>
#include <cstdint> 
#include <cmath>
#include <chrono>
#include <stdexcept>
#include <vector>

using namespace std;

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