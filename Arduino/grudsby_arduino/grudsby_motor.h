#ifndef GRUDSBY_MOTOR_H
#define GRUDSBY_MOTOR_H

#include "settings.h"
#include <Arduino.h>

namespace grudsby {

static const int timeout = 100;
static unsigned long lchangetime = 0;
static unsigned long rchangetime = 0;
static int lastlval;
static int lastrval;

static int cooldown_val;

bool checkChangeDir(int val, char side);
bool checkSafe(int val, char side);
void writeDirPWM(int lval, int rval);

}


#endif