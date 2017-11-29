#ifndef GRUDSBY_MOTOR_H
#define GRUDSBY_MOTOR_H

#include "settings.h"
#include <Arduino.h>
#include "Servo.h" 


namespace grudsby {

	class Motor
	{
	public: 
		Motor(){};
		virtual ~Motor(){};
		virtual void writeVal(int val) = 0;

	protected: 
		bool checkChangeDir(int val);
		bool checkSafe(int val);

		const int timeout = 100;
		const int COOLDOWN_RATE = 20;

		unsigned long changetime = 0;
		unsigned int lastval;
		int cooldown_val;
	};

	class DirPWMMotor : public Motor 
	{
	public: 
		DirPWMMotor(int dirPin, int pwmPin);
		void writeVal(int val);
	private:
		void writeDirPWM(bool dir, int pwm);

		int DIRPIN;
		int PWMPIN;
		const int MAXPWM = 255;
		const int MINPWM = 0;
	};

	class RCMotor : public Motor 
	{
	public:
		RCMotor(int pin);
		void writeVal(int val);
	private: 
		void writeRC(int rc);

		Servo servo;
		const int RC_MAX = 1870;
		const int RC_MIN = 1070;
		const int RC_STOP = 1470;
	};




	
}


#endif
