#include <stdio.h>
#include "slice.h"
#include "segment.h"

int main()
{
  Slice slice = Slice(1, 0, 0, 0);
  Segment seg = Segment(0, 3, 4, 3);

  slice.findIntersection(seg);
  
  return 0;
}
