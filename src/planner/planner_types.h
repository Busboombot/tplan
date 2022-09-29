#pragma once
#include <vector>
#include <cmath>
#include "types.h"

// Teensy 4.1 has very fast 32 float, but 64 bit floats  less so:
// "The FPU performs 32 bit float and 64 bit double precision math in hardware.
// 32 bit float speed is approximately the same speed as integer math. 64 bit
// double precision runs at half the speed of 32 bit float."

using trj_float_t = double;

using velocity_t = trj_float_t;
using VelocityVector = std::vector<velocity_t>;


constexpr int BV_PRIOR = -1;
constexpr int BV_NEXT = -2;
constexpr int BV_V_MAX = -3;
constexpr int BV_NAN = -4;




