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
	char side = 'n';
	int DIRPIN = 0;
	int PWMPIN = 0;
	int val  = 0;

	for (int i=0; i < 2; i++) {
		if (i == 0) {
			//set to left vals 
			side = 'l';
			DIRPIN = DIR1;
			PWMPIN = PWM1;
			val = lval;
		}
		else if (i == 1) {
			//set to right vals 
			side = 'r';
			DIRPIN = DIR2;
			PWMPIN = PWM2;
			val = rval;
		}

		//move motors 
		if (checkSafe(val, side)) {
			if (val > 0) {
				digitalWrite(DIRPIN, HIGH);
				analogWrite(PWMPIN, val);
			}
			else if (val < 0) {
				digitalWrite(DIRPIN, LOW);
				analogWrite(PWMPIN, abs(val));
			}
			else {
				analogWrite(PWMPIN, 0);
			}
		}
	}
}