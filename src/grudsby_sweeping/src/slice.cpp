#include "slice.h"
#include "Vector2.hpp"

Slice::Slice(double vx, double vy, double px, double py)
{
  normal = Vector2(vx, vy);
  point = Vector2(px, py);

}

Vector2 Slice::findIntersection(Segment s)
{
  Vector2 dslice = normal;
  Vector2 dsegment = s.b - s.a;
  
  if(Vector2::Cross(dslice, dsegment) != 0)
  {
    printf("gucci mane");
  }
 // std::vector<double> dsegment = {s.get_b()[0] = s.get_a()[0], s.get_b()[1] = s.get_a()[1]};
}
