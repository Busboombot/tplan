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
    ACMessage(){
        header.code = CommandCode::AXES;
    }
} ACMessage;

typedef struct ConfigMessage{
    PacketHeader header;
    Config config{0};
    ConfigMessage(){
        header.code = CommandCode::CONFIG;
    }
} ConfigMessage;

typedef struct MoveMessage{
    PacketHeader header;
    Moves moves{0};
    MoveMessage(){
        header.code = CommandCode::RMOVE;

    }
} MoveMessage;


/* Check that a few message types can pass from processPacket()
 * to the output Stream */
TEST_CASE("Basic Message Processor", "[mproc]")
{

    MockPacketSerial mps;
    MessageProcessor mp(static_cast<IPacketSerial &>(mps));

    Moves moves;
    mps.push(PacketHeader(1,CommandCode::RMOVE),
                     (char*)&moves, sizeof(moves) );

    string str1("Hello ... Hello");
    mps.push(PacketHeader(2,CommandCode::ECHO),
                     (char*)str1.data(), str1.length() );

    string str2("More Cowbell");
    mps.push(PacketHeader(3,CommandCode::MESSAGE),
                     (char*)str2.data(), str2.length() );

    mp.updateAll();

    cout << "In Messages: " << mp.availableMessages() << endl;
    //REQUIRE(mp.availableMessages()==2);
    while(!mp.empty()){
        cout << "In: " << mp.firstMessage() << endl;
        mp.pop();
    }

    cout << "----";
    cout << "Out Messages: " << mps.outgoing.size() << endl;
    while(!mps.outgoing.empty()){
        cout << "Out: " << Message(mps.outgoing.front()) << endl;
        mps.outgoing.pop_front();
    }
    //REQUIRE(stream.messages.size()==3);





}