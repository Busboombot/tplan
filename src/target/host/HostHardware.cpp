#include <ratio>
#include <chrono>
#include <string>
#include <sstream>
#include <iostream>
#include <thread>

#include "HostHardware.h"

#include "util.h"

using namespace std;
using namespace std::chrono;

HostHardware::HostHardware() : Hardware() {
    t0 = steady_clock::now();
}

void HostHardware::update() {

}

void HostHardware::dumpPinCounts() {

    for (auto &i: highCount) {
        auto lc = lowCount[i.first];
        auto mc = missCount[i.first];
        cout << "Pin " << setw(3) << (int) i.first <<
             " H: " << setw(3) << (int) i.second <<
             " L: " << setw(3) << (int) lc <<
             " M: " << setw(3) << (int) mc << endl;
    }

}

void HostHardware::printPins() {
    static string last;
    stringstream ss;

    for (int i = 0; i < pins.size(); i++) {
        auto v = pins[i];

        if (v) {
            ss << blue_bg << (int) i << creset;
            //ss << setw(3) << (int)i;
        } else {
            ss << (int) i;
            //ss << "   ";
        }
    }

    if (ss.str() != last) {
        last = ss.str();
        cout << setw(9) << micros() << " " << last << endl << flush;
    }
}

void HostHardware::writePin(Pin pin, PinVal value) {

    if (pin > 64) {
        return;
    }

    if (pin >= pins.size()) {
        pins.resize(pin+1, 0);
    }

    // Only record if there was a transition
    if (value && pins[pin] == LOW) {
        highCount[pin]++;
    } else if ( !value && pins[pin] == HIGH) {
        lowCount[pin]++;
    } else {
        missCount[pin]++;
    }

    pins[pin] = value;

    if (print_pin_change) {
        printPins();
    }

}

int HostHardware::readPin(Pin pin) {
    return 0;
}

void HostHardware::signalRunning(bool v) {

}

void HostHardware::signalEmpty(bool v) {

}

void HostHardware::signalError(bool v) {

}

tmillis HostHardware::millis() {
    if (use_system_time) {
        auto t1 = steady_clock::now();
        return (tmillis) std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    } else {
        return hw_time/1000;
    }

}

tmicros HostHardware::micros() {
    if (use_system_time) {
        auto t1 = steady_clock::now();
        return (tmicros) std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
    } else {
        return hw_time;
    }
}

void HostHardware::delayMillis(uint32_t v) {
    if (use_system_time) {
        std::this_thread::sleep_for(std::chrono::milliseconds(v));
    } else {
        hw_time += (v*1000);
    }
}

void HostHardware::delayMicros(uint32_t v) {
    if (use_system_time) {
        std::this_thread::sleep_for(std::chrono::microseconds (v));
    } else {
        hw_time += v;
    }
}

bool HostHardware::limitChanged() {
    return false;
}

void HostHardware::stepTime(tmicros dt){
    hw_time += dt;
}