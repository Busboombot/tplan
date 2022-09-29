#pragma once
#include <iostream>
#include <string>
#include <limits>
#include <map>
#include <chrono>
#include <thread>

#include <catch2/catch_test_macros.hpp>

#include "types.h"
#include "messageprocessor.h"
#include "PacketSerial.h"
#include "HostHardware.h"

#include "loop.h"

extern Config defaultConfig(uint8_t n_axes);
extern AxisConfig defaultAxisConfig(uint8_t axis);
extern void pushConfig(MockPacketSerial &mps, int axes, int period);
extern void pushAxisConfig(MockPacketSerial &mps, int axis);
extern void pushMove(MockPacketSerial &mps, CommandCode cmd, Moves m);
extern void pushMessage(MockPacketSerial &mps, CommandCode cmd);

using sclock = std::chrono::system_clock;
using sec = std::chrono::duration<double>;
using us = std::chrono::duration<double, std::micro>;

vector<Move> get2Moves();
vector<Joint> get2Joints();
vector<Joint> get4Joints();

std::vector<int> extractIntegerWords(const string& str);