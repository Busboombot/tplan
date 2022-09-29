#include <iostream>
#include <string>
#include <limits>
#include <map>
#include <chrono>
#include <thread>

#include <catch2/catch_test_macros.hpp>

#include "trj_types.h"
#include "messageprocessor.h"
#include "PacketSerial.h"
#include "HostHardware.h"

#define CATCH_CONFIG_MAIN

using namespace std;
using namespace std::chrono;

Config defaultConfig(uint8_t n_axes){

    uint8_t base_pin = (((N_AXES+1)*3)+0);

    return Config {
            n_axes, // uint8_t n_axes = 0;         // Number of axes
            4,      // uint8_t interrupt_delay = INTERRUPT_DELAY;    // How often interrupt is called, in microseconds
            uint8_t(base_pin+0),     // uint8_t segment_complete_pin = 0; // Pin on which to signal that a segment is complete
            uint8_t(base_pin+1),     // uint8_t limit_pin = 0; // Pin to recieve signals that the encoder foind a limit
            uint8_t(base_pin+2), // uint8_t yellow_led_pin=0;
            uint8_t(base_pin+3), // uint8_t blue_led_pin=0;
            uint8_t(base_pin+4), // uint8_t running_led_pin=0;
            uint8_t(base_pin+5), // uint8_t empty_led_pin=0;
            uint8_t(base_pin+6), // uint8_t builtin_led_pin=0;
            false,  // bool debug_print = true;
            false,  // bool debug_tick = true;
    };
}

AxisConfig defaultAxisConfig(uint8_t axis){

    return {

            axis,           // uint8_t axis=0;           // Axis number

            static_cast<uint8_t>(((axis+1)*3)+0),     // uint8_t step_pin=0;       // Step output,
            static_cast<uint8_t>(((axis+1)*3)+1),     // uint8_t direction_pin=0;  // Direction output
            static_cast<uint8_t>(((axis+1)*3)+2),     // uint8_t enable_pin=0;

            HIGH,           // step_high_value=0; // Whether step is HIGH or LOW when enabled.
            HIGH,           // uint8_t direction_high_value=0;
            HIGH,           // uint8_t enable_high_value=0;

            0,              // uint8_t step_output_mode=0; // OUTPUT or OUTPUT_OPEN_DRAIN
            0,              // uint8_t direction_output_mode=0;
            0,              // uint8_t enable_output_mode=0;

            0xDE,
            0XAD,

            5000,          // uint32_t v_max=0;
            100000         //  a_max=0;
    };
}

int seq_id = 0;

void pushConfig(MockPacketSerial &mps, int axes, int period=5){
    Config config = defaultConfig(axes);
    config.interrupt_delay = period;
    mps.push(PacketHeader(seq_id++, CommandCode::CONFIG), (char *) &config, sizeof(Config));
}

void pushAxisConfig(MockPacketSerial &mps, int axis){
    AxisConfig ac = defaultAxisConfig(axis);
    mps.push(PacketHeader(seq_id++, CommandCode::AXES), (char *) &ac, sizeof(AxisConfig));
}

void pushMove(MockPacketSerial &mps, CommandCode cmd, Moves m){
    mps.push(PacketHeader(seq_id++, cmd), (char *) &m, sizeof(Moves));
}

void pushMessage(MockPacketSerial &mps, CommandCode cmd){
    mps.push(PacketHeader(seq_id++, cmd), (char *) 0, 0);
}


/* Check that a few message types can pass from processPacket()
 * to the output Stream */
TEST_CASE("Basic Message Processor", "[mproc]")
{

    MockPacketSerial mps;
    CurrentState cs;
    MessageProcessor mp(static_cast<IPacketSerial &>(mps));
    tmillis t = 0;

    Moves moves;
    mps.push(PacketHeader(1, CommandCode::RMOVE), (char *) &moves, sizeof(moves));
    cs.queue_length++;
    mp.updateAll(t, cs);

    string str1("Hello ... Hello");
    mps.push(PacketHeader(2, CommandCode::ECHO),
             (char *) str1.data(), str1.length());

    cs.queue_length++;
    mp.updateAll(t, cs);
    string str2("More Cowbell");
    mps.push(PacketHeader(3, CommandCode::MESSAGE),
             (char *) str2.data(), str2.length());

    cs.queue_length++;
    mp.updateAll(t, cs);

    AxisConfig ac;
    ac.axis = 37;
    ac.step_pin = 12;
    ac.direction_pin = 11;
    ac.enable_pin = 13;
    mps.push(PacketHeader(3, CommandCode::AXES), (char *) &ac, sizeof(ac));

    cs.queue_length++;
    mp.updateAll(t, cs);

    Config cfg;
    cfg.n_axes = 7;
    cfg.interrupt_delay = 24;
    cfg.limit_pin = 16;
    cfg.segment_complete_pin = 17;
    mps.push(PacketHeader(3, CommandCode::CONFIG), (char *) &cfg, sizeof(cfg));

    cs.queue_length++;
    mp.updateAll(t, cs);

    cout << "In Messages: " << mp.availableMessages() << endl;
    //REQUIRE(mp.availableMessages()==2);
    while (!mp.empty()) {
        cout << "In: " << mp.firstMessage() << endl;
        mp.pop();
    }

    cout << "----";
    cout << "Out Messages: " << mps.outgoing.size() << endl;
    while (!mps.outgoing.empty()) {
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

    stringstream ss;

    ss << "M3: A Multi" << " " << "Line message!" << endl;
    ss << "This is the second line! " << endl;
    ss << "This is the Third line! " << endl;

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
