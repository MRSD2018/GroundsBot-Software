#ifndef GRUDSBY_ARDUINO_H
#define GRUDSBY_ARDUINO_H

#include <Arduino.h>
#include <ros.h>
#include <grudsby_lowlevel/ArduinoVel.h>
#include <grudsby_lowlevel/ArduinoResponse.h>

void velCallback(const grudsby_lowlevel::ArduinoVel& msg);

void ISR_1();
void ISR_2();

void publishStatus();

#endif