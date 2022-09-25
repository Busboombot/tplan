#include <iostream>
#include <sstream>
#include <string>
#include <catch2/catch_test_macros.hpp>
#define CATCH_CONFIG_MAIN

#include <iostream>
#include "trj_types.h"
#include "messageprocessor.h"
#include "PacketSerial.h"

using namespace std;

typedef struct ACMessage{
    PacketHeader header;
    AxisConfig aconfig{0};

} ACMessage;

typedef struct ConfigMessage{
    PacketHeader header;
    Config config{0};
} ConfigMessage;

typedef struct MoveMessage{
    PacketHeader header;
    Moves moves{0};
} MoveMessage;


TEST_CASE("Basic Message Processor", "[mproc]")
{
    Stream stream;
    MessageProcessor mp(stream);

    MoveMessage mm;

    mp.processPacket((const uint8_t* )&mm, sizeof(mm));

    cout << "Messages: " << mp.availableMessages() << endl;
}