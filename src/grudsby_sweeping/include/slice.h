
#ifndef SLICE_H
#define SLICE_H
#include <stdio.h>
#include <vector>
#include "Vector2.hpp"
#include "segment.h"

class Slice
{
  public:
    Slice(double px, double py, double vx, double vy);
    void increment(double e);
    void setPoint(double px, double py);
    Vector2 findIntersection(Segment s);
    bool intersectsOnce(Segment s);
    double distanceToPoint(std::vector<double> point);
    int directionToPoint(std::vector<double> point);

    Vector2 normal;
    Vector2 point;
};
#endif
