#ifndef BOUSTROPHEDON_H
#define BOUSTROPHEDON_H
#include <stdio.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include "slice.h"
#include "segment.h"
#include "Vector2.hpp"
#include "parsePath.h"
class Boustrophedon
{
  public:
    Boustrophedon(double implementWidth);
    std::string planPath(std::string region);
  private:
    double myImplementWidth;
};

#endif
