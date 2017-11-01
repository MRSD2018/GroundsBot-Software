#include "grudsby_motor.h"

using namespace grudsby;

bool grudsby::checkChangeDir(int val, char side) {
	int* lastval = 0;
	unsigned long* changetime;
	if (side == 'l') {
		lastval = &lastlval;
		changetime = &lchangetime;
	}
	else if (side == 'r') {
		lastval = &lastrval;
		changetime = &rchangetime;
	}


	if ((val > 0 == *lastval > 0) && millis() > *changetime + timeout){
		return true;
	}
	else if (val > 0 != *lastval > 0) {
		*lastval = val;
		*changetime = millis();
	}


	return false;
}


//true if safe
bool grudsby::checkSafe(int val, char side) {
	bool isSafe = true;

	isSafe = isSafe && checkChangeDir(val, side);

	return isSafe;
}

void grudsby::writeDirPWM(int lval, int rval) {
  if (lval > 0) {
    digitalWrite(DIR1, HIGH);
    analogWrite(PWM1, lval);
  }
  else if (lval < 0) {
    digitalWrite(DIR1, LOW);
    analogWrite(PWM1, abs(lval));
  }
  else {
    analogWrite(PWM1, 0); 
  }

  int rv = rval;
}