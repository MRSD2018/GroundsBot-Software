#ifndef PARSEPATH_H
#define PARSEPATH_H
#include <iostream>
#include <vector>
#include <string>
#include <stdio.h>
#include <sstream>
#include <iterator>
#include <math.h>
#include <algorithm>
class ParsePath
{
  public:
    ParsePath(std::string mowingPath,double implementWidthMeters);
    void getRegion(std::vector<std::vector<double>>& region);
    double getImplementWidth();
  private:
    std::vector<std::vector<double>> myRegion;
    double myImplementWidth;
};

#endif
