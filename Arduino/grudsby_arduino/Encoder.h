/*
	Encoder.h - Library for reading velocity and position of GroundsBot motors
	David Evans, November 2, 2017
*/
#ifndef Encoder_h
#define Encoder_h
#include "Arduino.h"


class Encoder
{
	public:
		Encoder(int channel_a, int channel_b);
		long get_position();
		int get_velocity();

		void encoder_update();
		void velocity_update();
	
		int channel_a_pin;
		int channel_b_pin;
	private:
		long position;
		int velocity;
		long prev_timer_position;
		

		int aLastState;

};

void init_timer(int frequency);
#endif