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
#include "grudsby_sweeping/SimpleLatLng.h"
#include "grudsby_sweeping/MowingPlan.h"
#include "ros/ros.h"
#include "ros/console.h"
#include "sign.h"

class Boustrophedon
{
  public:
    Boustrophedon(double implementWidth);
    std::string planPath(std::string region, grudsby_sweeping::MowingPlan& plan, grudsby_sweeping::MowingPlan& mowing_region);

  private:
    double myImplementWidth;
    int messageSequence;
};

#endif
