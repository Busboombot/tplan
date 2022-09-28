#include <iostream>
#include <string>
#include <limits>
#include <catch2/catch_test_macros.hpp>

#include "trj_types.h"
#include "messageprocessor.h"
#include "PacketSerial.h"

#define CATCH_CONFIG_MAIN

using namespace std;

/* Check that a few message types can pass from processPacket()
 * to the output Stream */
TEST_CASE("Basic Message Processor", "[mproc]")
{

    MockPacketSerial mps;
    CurrentState cs;
    MessageProcessor mp(static_cast<IPacketSerial &>(mps));
    tmillis t = 0;

    Moves moves;
    mps.push(PacketHeader(1,CommandCode::RMOVE),(char*)&moves, sizeof(moves) );
    cs.queue_length++;
    mp.updateAll(t,cs);

    string str1("Hello ... Hello");
    mps.push(PacketHeader(2,CommandCode::ECHO),
                     (char*)str1.data(), str1.length() );

    cs.queue_length++;
    mp.updateAll(t,cs);
    string str2("More Cowbell");
    mps.push(PacketHeader(3,CommandCode::MESSAGE),
                     (char*)str2.data(), str2.length() );

    cs.queue_length++;
    mp.updateAll(t,cs);

    AxisConfig ac;
    ac.axis=37;
    ac.step_pin = 12;
    ac.direction_pin = 11;
    ac.enable_pin = 13;
    mps.push(PacketHeader(3,CommandCode::AXES),(char*)&ac, sizeof(ac));

    cs.queue_length++;
    mp.updateAll(t,cs);

    Config cfg;
    cfg.n_axes = 7;
    cfg.interrupt_delay=24;
    cfg.limit_pin=16;
    cfg.segment_complete_pin=17;
    mps.push(PacketHeader(3,CommandCode::CONFIG),(char*)&cfg, sizeof(cfg));

    cs.queue_length++;
    mp.updateAll(t,cs);

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

    mp.printf("Hi Mom! We have %d messages left!\n", mps.outgoing.size());

}

TEST_CASE("Message Processor Text Messages", "[mproc]")
{

    MockPacketSerial mps;
    MessageProcessor mp(static_cast<IPacketSerial &>(mps));

    mp.printf("M1: Hi Mom! We have %d messages left!\n", mps.outgoing.size());
    mp.sendMessage("M2: Another text message\n");

    stringstream  ss;

    ss << "M3: A Multi" << " " << "Line message!" << endl;
    ss << "This is the second line! "<< endl;
    ss << "This is the Third line! "<< endl;

    mp.sendMessage(ss);

    // Should not see these messages

    log("XXX Should not see this\n");
    string s("XXX Should not see this\n");
    log(s);

    message_processor = &mp;

    log("Should see this\n");
    string s2("Should see this\n");
    log(s2);

}

TEST_CASE("Int Timing Test", "[scratch]"){

    using T = long double;
    T mx = numeric_limits<T>::max();
    T last = mx - 50;
    T now =  mx + 50;

    cout << last<< " " << now<< " " << float(mx/1000/60/60/24)/365. << " " << numeric_limits<T>::digits << endl;
    cout << "DIFF " << now - last << endl;



}
