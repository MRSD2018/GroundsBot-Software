#include "Encoder.h"

Encoder encoder1(2, 4);
Encoder encoder2(3, 5);

void setup()
{
 Serial.begin(9600);
 
 attachInterrupt(digitalPinToInterrupt(encoder1.channel_a_pin), ISR_1, CHANGE);
 attachInterrupt(digitalPinToInterrupt(encoder2.channel_a_pin), ISR_2, CHANGE);
  
  
  init_timer(10);
}

void loop()
{
  Serial.println("Motor 1:");
  Serial.print("Position: ");
  Serial.println(encoder1.get_position());
  Serial.print("Velocity: ");
  Serial.println(encoder1.get_velocity());
  Serial.println(" ");
  Serial.println("Motor 2:");
  Serial.print("Position: ");
  Serial.println(encoder2.get_position());
  Serial.print("Velocity: ");
  Serial.println(encoder2.get_velocity());

  Serial.println("_________________________");
  delay(100);

}

void ISR_1()
{
 encoder1.encoder_update(); 
}
void ISR_2()
{
 encoder2.encoder_update(); 
}
ISR(TIMER1_COMPA_vect) 
{
  encoder1.velocity_update(); 
  encoder2.velocity_update();  
} 

