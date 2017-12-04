#include "grudsby_motor.h"

using namespace grudsby;

bool Motor::checkChangeDir(int val) 
{
	if (((val > 0) == (this->lastval > 0)) && (millis() > (this->changetime + this->timeout))){
		return true;
	}
	else if ((val > 0) !=(this->lastval > 0)) {
		this->lastval = val;
		this->changetime = millis();
	}


	return false;
}


//true if safe
bool Motor::checkSafe(int val) 
{
	bool isSafe = true;

	isSafe = isSafe && checkChangeDir(val);

	return isSafe;
}

DirPWMMotor::DirPWMMotor(int dirPin, int pwmPin){
	this->DIRPIN = dirPin;
	this->PWMPIN = pwmPin;
}

void DirPWMMotor::writeVal(int val) 
{
	if (checkSafe(val)) {
		if (val > 0) {
			writeDirPWM(true, min(val, MAXPWM));
			cooldown_val = val;
		}
		else if (val < 0) {
			writeDirPWM(false, max(abs(val), MINPWM));
			cooldown_val = abs(val);
		}
		else {
			writeDirPWM(true, cooldown_val); //direction is ignored
			cooldown_val = max(0, cooldown_val - COOLDOWN_RATE);
		}
	}
}

void DirPWMMotor::writeDirPWM(bool forward, int pwm) 
{
	if (abs(pwm) > 0) {
		if (forward) {
			digitalWrite(DIRPIN, HIGH);
			analogWrite(PWMPIN, pwm);
		}
		else if (!forward) {
			digitalWrite(DIRPIN, LOW);
			analogWrite(PWMPIN, pwm);
		}

	}
	else {
		analogWrite(PWMPIN, pwm);
	}
}

RCMotor::RCMotor(int pin) {
  servoPin = pin;
	this->attachServo();
}

void RCMotor::writeVal(int val) 
{
	int command = map(val, -255, 255, RC_MIN, RC_MAX);
	if (val > 0) {
		writeRC(min(command, RC_MAX));
	}
	else if (val < 0) {
		writeRC(max(command, RC_MIN));
	}
	else {
		writeRC(RC_STOP);
	}
} 

void RCMotor::attachServo() {
  this->servo.attach(servoPin);
  this->writeVal(0);
}

void RCMotor::detachServo() {
  this->servo.detach();
}

void RCMotor::writeRC(int rc){
	//1250 is full back, 1750 full forward
	if (servo.attached()) {
		servo.writeMicroseconds(rc);
	}
}
