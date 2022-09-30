#pragma once
#include "../../../../../../../Library/Developer/CommandLineTools/SDKs/MacOSX12.3.sdk/usr/include/c++/v1/iostream"
#include "../../../../../../../Library/Developer/CommandLineTools/SDKs/MacOSX12.3.sdk/usr/include/c++/v1/string"
#include "../../../../../../../Library/Developer/CommandLineTools/SDKs/MacOSX12.3.sdk/usr/include/c++/v1/limits"
#include "../../../../../../../Library/Developer/CommandLineTools/SDKs/MacOSX12.3.sdk/usr/include/c++/v1/map"
#include "../../../../../../../Library/Developer/CommandLineTools/SDKs/MacOSX12.3.sdk/usr/include/c++/v1/chrono"
#include "../../../../../../../Library/Developer/CommandLineTools/SDKs/MacOSX12.3.sdk/usr/include/c++/v1/thread"

#include "../../../../../../../usr/local/include/catch2/catch_test_macros.hpp"

#include "../planner/types.h"
#include "../planner/messageprocessor.h"
#include "../target/host/PacketSerial.h"
#include "../target/host/HostHardware.h"

#include "../planner/loop.h"

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