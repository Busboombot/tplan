#include "types.h"
#include "debug.h"
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <functional>

using namespace std;

void padTo(std::string &str, const size_t num, const char paddingChar = ' ') {
    if (num > str.size())
        str.insert(0, num - str.size(), paddingChar);
}

MoveVector &operator-=(MoveVector &a, const MoveArray &b) {
    std::transform(a.begin(), a.end(), b.begin(), a.begin(), std::minus<AxisPos>());
    return a;
}

MoveVector &operator+=(MoveVector &a, const MoveArray &b) {
    std::transform(a.begin(), a.end(), b.begin(), a.begin(), std::plus<AxisPos>());
    return a;
}

MoveArray &operator+=(MoveArray &a, const MoveVector &b) {
    std::transform(a.begin(), a.end(), b.begin(), a.begin(), std::plus<AxisPos>());
    return a;
}

MoveArray &operator-=(MoveArray &a, const MoveVector &b) {
    std::transform(a.begin(), a.end(), b.begin(), a.begin(), std::minus<AxisPos>());
    return a;
}

MoveArray &operator+=(MoveArray &a, const MoveArray &b) {
    std::transform(a.begin(), a.end(), b.begin(), a.begin(), std::plus<AxisPos>());
    return a;
}

MoveArray &operator-=(MoveArray &a, const MoveArray &b) {
    std::transform(a.begin(), a.end(), b.begin(), a.begin(), std::minus<AxisPos>());
    return a;
}


ostream &operator<<(ostream &output, const Moves &m) {
    output << "[Moves t=" << m.segment_time << " (";
    for (auto e: m.x) output << e << ",";
    output << ")]";
    return output;
}

ostream &operator<<(ostream &output, const MoveVector &m) {
    output << "[MoveVector  (";
    for (auto e: m) output << e << ",";
    output << ")]";
    return output;
}

ostream &operator<<(ostream &output, const MoveArray &m) {
    output << "[MoveArray  (";
    for (auto e: m) output << e << ",";
    output << ")]";
    return output;
}


ostream &operator<<(ostream &output, const AxisConfig &ac) {
    output << "[AxisConfig " << (int) ac.axis <<
           " sp=" << (int) ac.step_pin <<
           " so=" << (int) ac.step_output_mode <<
           " dp=" << (int) ac.direction_pin <<
           " do=" << (int) ac.direction_output_mode <<
           " ep=" << (int) ac.enable_pin <<
           " eh=" << (int) ac.enable_high_value <<
           " em=" << (int) ac.enable_output_mode <<
           " vm=" << ac.v_max <<
           " am=" << ac.a_max <<
           "]";
    return output;
}

ostream &operator<<(ostream &output, const Config &c) {

    output << "[Config axes=" << (int) c.n_axes <<
           " id=" << (int) c.interrupt_delay <<
           " sp=" << (int) c.segment_complete_pin <<
           " lp=" << (int) c.limit_pin <<
           " dp=" << (int) c.debug_print <<
           " dt=" << (int) c.debug_tick <<
           "]";
    return output;
}

ostream &operator<<(ostream &output, const CurrentState &cs) {

    output << "[CurrentState ql=" << cs.queue_length << " qt=" << double(cs.queue_time) / TIMEBASE << " (";
    for (int i = 0; i < N_AXES; i++) { output << cs.positions[i] << " "; }
    output << ") (";
    for (int i = 0; i < N_AXES; i++) { output << cs.planner_positions[i] << " "; }
    output << ")]";
    return output;
}


ostream &operator<<(ostream &output, const Message &m) {

    auto cmdmap = commandMap();
    string cmdName = cmdmap[m.header.code];
    padTo(cmdName, 6);
    cmdName += " ";

    output << "[M " << setw(3) << m.header.seq << " " << setw(3) << (int) m.header.code;

    switch (m.header.code) {
        case CommandCode::MESSAGE:
        case CommandCode::ECHO:
            output << cmdName << m.asString();
            break;
        case CommandCode::RMOVE:
        case CommandCode::AMOVE:
        case CommandCode::JMOVE:
        case CommandCode::HMOVE:
        case CommandCode::VMOVE:
            output << cmdName << *(m.asMoves());
            break;
        case CommandCode::AXES:
            output << cmdName << *(m.asAxisConfig());
            break;
        case CommandCode::CONFIG:
            output << cmdName << *(m.asConfig());
            break;
        case CommandCode::ACK:
            output << cmdName << *(m.asCurrentState());
            break;


        default:
            output << cmdName;

    }

    output << "]";
    return output;

}

map<CommandCode, MoveType> cmdmove_map = {
        {CommandCode::RMOVE, MoveType::relative},
        {CommandCode::HMOVE, MoveType::home},
        {CommandCode::AMOVE, MoveType::absolute},
        {CommandCode::JMOVE, MoveType::jog},
        {CommandCode::VMOVE, MoveType::velocity}
};
