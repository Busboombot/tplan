//
// Created by Eric Busboom on 9/26/22.
//

#include "TargetHardware.h"


/*
 *
 *   if (config.segment_complete_pin > 0){
    pinMode(config.segment_complete_pin, OUTPUT);
  }

  if (config.limit_pin > 0){
    pinMode(config.limit_pin, INPUT);
    attachInterrupt(config.limit_pin, limitChangedISR, RISING);
  }

  sd.setNAxes(config.n_axes);
  sd.setPeriod(config.interrupt_delay);

 */

void TargetHardware::update() {

}

void TargetHardware::writePin(PinVal pin, PinVal value) {

}

int TargetHardware::readPin(PinVal pin) {
    return 0;
}

void TargetHardware::signalRunning(bool v) {

}

void TargetHardware::signalEmpty(bool v) {

}

void TargetHardware::signalError(bool v) {

}


tmillis TargetHardware::millis() {
    return 0;
}

tmicros TargetHardware::micros() {
    return 0;
}


void TargetHardware::delayMillis(uint32_t v) {

}

void TargetHardware::delayMicros(uint32_t v) {

}

bool TargetHardware::limitChanged() {
    return false;
}

