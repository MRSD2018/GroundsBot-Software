#ifndef PARSEPATH_H
#define PARSEPATH_H
#include <iostream>
#include <vector>
#include <string>
#include <stdio.h>
#include <sstream>
#include <iterator>
#include <math.h>
class ParsePath
{
  public:
    ParsePath(std::string mowingPath,std::string implementWidth);
    void getRegion(std::vector<std::vector<double>>& region);
    double getImplementWidth();
  private:
    std::vector<std::vector<double>> myRegion;
    double myImplementWidth;
};

#endif
