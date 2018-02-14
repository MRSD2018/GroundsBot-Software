#ifndef SEGMENT_H
#define SEGMENT_H
#include<stdio.h>
#include "Vector2.hpp"

class Segment
{
  public:
    Segment(double ax, double ay, double bx, double by);
    Vector2 getA();
    Vector2 getB();
    double getLength();    
  
    Vector2 a;
    Vector2 b;
};
#endif
