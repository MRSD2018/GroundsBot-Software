#include "segment.h"

Segment::Segment(double ax, double ay, double bx, double by)
{
  a = Vector2(ax, ay);
  b = Vector2(bx, by);
}

double Segment::getLength()
{
  double length = sqrt((b.Y - a.Y)*(b.Y - a.Y) + (b.X - a.X)*(b.X - a.X));

  return length;
}
