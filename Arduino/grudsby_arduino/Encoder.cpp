/*
	Encoder.h - Library for reading velocity and position of GroundsBot motors
	David Evans, November 2, 2017
*/
#include "Arduino.h"
#include "Encoder.h"

Encoder::Encoder(int channel_a, int channel_b)
{
	channel_a_pin = channel_a;
	channel_b_pin = channel_b;

	pinMode(channel_a_pin, INPUT_PULLUP);
	pinMode(channel_b_pin, INPUT);
	
	//Doesn't seem like you can create an interrupt within a class
	//attachInterrupt(digitalPinToInterrupt(channel_a_pin), Encoder::encoder_update, CHANGE);
	

	aLastState = digitalRead(channel_a_pin);
	position = 0;
	prev_timer_position = 0;
	velocity =0;
}
/*
	Called by the assigned pin interrupt to update the encoder position

	params: void
	return: void
*/
void Encoder::encoder_update()
{
  int aState = digitalRead(channel_a_pin);

  if (aState != aLastState)
  {
    int bState = digitalRead(channel_b_pin);

    if(bState != aState)
    {
      position++;
    }
    else
    {
      position--;
    }
    
    aLastState = aState;
  }
}
/*
	Called by the timer interrupt to update the encoder velocity

	params: void
	return: void
*/
void Encoder::velocity_update()
{
	velocity = position - prev_timer_position;
	prev_timer_position = position;
}

/*
	Keeps track of the position in ticks on the encoder from the encoder's starting position

	params: void
	return: the current position of the encoder in ticks from start
*/
long Encoder::get_position()
{
	return position;
}

/*
	Keeps track of the velocity of the encoder in ticks per time period
	The time period is defined by the timer frequency

	params: void
	return: the current velocity of the encoder in ticks per time period
*/
int Encoder::get_velocity()
{
	return velocity;
}

/*
	Initializes the timer interrupt to trigger at a given frequency

	params:
		frequency: the frequency in hertz the timer will interrupt
	return: void
*/
void init_timer(int frequency)
{
	// initialize Timer1
  noInterrupts(); // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  
  int prescaler = 256;
  int register_value = (16000000/prescaler)/frequency;
  OCR1A = register_value; // equal to clock frequency/prescaler/frequency
  TCCR1B |= (1 << WGM12); // CTC mode
  TCCR1B |= (1 << CS12); // 256 prescaler
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  interrupts(); // enable all interrupts

}