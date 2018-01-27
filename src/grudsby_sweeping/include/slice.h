
#ifndef SLICE_H
#define SLICE_H
#include <stdio.h>
#include "Vector2.hpp"
#include "segment.h"

class Slice
{
  public:
    Slice(double vx, double vy, double px, double py);
    void increment(double tool_width);
    Vector2 findIntersection(Segment s);
//    double distanceToPoint(std::vector<double> point);

  private:
    Vector2 normal;
    Vector2 point;
};
#endif
