#include "slice.h"
#include "Vector2.hpp"

Slice::Slice(double px, double py, double vx, double vy)
{
  normal = Vector2(vx, vy);
  point = Vector2(px, py);

}

void Slice::setPoint(double px, double py)
{
  point = Vector2(px, py);
}

bool Slice::intersectsOnce(Segment s)
{
  Vector2 v_slice = Vector2::Normal(this->normal);
  Vector2 v_seg = s.b - s.a;
 
  if(Vector2::Cross(v_slice, v_seg) != 0)
  {
    Vector2 intersectPoint = this->findIntersection(s);
  
    bool inSegXbound =  (intersectPoint.X <= s.a.X && intersectPoint.X >= s.b.X)||(intersectPoint.X >= s.a.X && intersectPoint.X <= s.b.X);
    bool inSegYbound =  (intersectPoint.Y <= s.a.Y && intersectPoint.Y >= s.b.Y)||(intersectPoint.Y >= s.a.Y && intersectPoint.Y <= s.b.Y);

    return inSegXbound && inSegYbound;
  }
  return false;
}

Vector2 Slice::findIntersection(Segment s)
{
  Vector2 intersectPoint = Vector2();
  Vector2 v_slice = Vector2::Normal(this->normal);
  Vector2 v_seg = s.b - s.a;
  
  if(Vector2::Cross(v_slice, v_seg) != 0)
  {
    Vector2 v_seg_slice = this->point - s.a;

    double t = Vector2::Cross(v_seg_slice, v_slice)/Vector2::Cross(v_seg, v_slice);
  
    intersectPoint = s.a + t*v_seg;
  }
  return intersectPoint;
}

void Slice::increment(double e)
{
  Vector2 incrementStep = e*this->normal/Vector2::Magnitude(this->normal);

  this->point = this->point + incrementStep;
}

double Slice::distanceToPoint(std::vector<double> point)
{
  Vector2 n = Vector2::Normalized(this->normal);
  Vector2 v = Vector2(point[0] - this->point.X, point[1] - this->point.Y);

  double d = Vector2::Dot(n, v);

  return d;
}

int Slice::directionToPoint(std::vector<double> point)
{
  if(distanceToPoint(point) > 0)
  {
    return 1;
  }
  else if(distanceToPoint(point) < 0)
  {
    return -1;
  }
  else
  {
    return 0;
  }
}
