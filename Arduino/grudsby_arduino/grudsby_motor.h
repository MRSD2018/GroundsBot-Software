#ifndef GRUDSBY_MOTOR_H
#define GRUDSBY_MOTOR_H

#include "settings.h"
#include <Arduino.h>

namespace grudsby {

	class IMotor
	{
	public: 
		virtual void writeVal(int val) = 0;

	protected: 
		const timeout = 100;
		unsigned long changetime = 0;
		unsigned int lastval;
		int cooldown_val;

		bool checkChangeDir(int val);
		bool checkSafe(int val);
	}

	class DirPWMMotor : public IMotor 
	{
	public: 
		void writeVal(int val);
	private:
		void writeDirPWM(bool dir, int pwm);
	}

	class RCMotor : public IMotor 
	{
	public:
		void writeVal(int val);
	private: 
		const int RC_MAX = 1750;
		const int RC_MIN = 1250;
		const int RC_STOP = 1500;

		void writeRC(int rc);

	}




	bool checkChangeDir(int val, char side);
	bool checkSafe(int val, char side);
	void writeDirPWM(int lval, int rval);
	void writeRC(int m1, int m2);

}


#endif