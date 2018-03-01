#include "parsePath.h"

void split(const std::string& s, char delimiter, std::vector<std::string>& tokens)
{
   std::string token;
   std::istringstream tokenStream(s);
   while (std::getline(tokenStream, token, delimiter))
   {
     tokens.push_back(token);
   }
}

double deg2rad(double val)
{
  return val*3.14159265359/180.0;
}

ParsePath::ParsePath(std::string mowingPath,double implementWidthMeters)
{
  ParsePath::parseLatLng(mowingPath, myRegion);
  int R = 6371; // Radius of the earth in km
  double dLon = deg2rad(1); 
  double a = 
    cos(deg2rad(myRegion[0][0])) * cos(deg2rad(myRegion[0][0])) * 
    sin(dLon/2) * sin(dLon/2); 
  double c = 2 * atan2(sqrt(a), sqrt(1-a)); 
  double d = R * c * 1000.0; // Distance in m
  
  // Parse implement width
  std::string::size_type sz;
  myImplementWidth = implementWidthMeters/d;
}

void ParsePath::parseLatLng(std::string mowingPath, std::vector<std::vector<double>> &myRegion)
{
  mowingPath.erase(remove_if(mowingPath.begin(), mowingPath.end(), isspace), mowingPath.end());
  
  // Parse points vector
  std::size_t first = mowingPath.find("[");
  std::size_t last = mowingPath.find("]");
  if ((first == std::string::npos) || (last ==std::string::npos))
  {
  
  }
  else 
  {
    std::string cut = mowingPath.substr(first+1,last-first-1);
    std::vector<std::string> results;
    split(cut,',', results); 
    for (int i = 0; i < floor(results.size()/2.0); i++)
    {
      std::size_t lat = results[2*i].find("lat\":");
      std::string latVal;
      std::string lngVal;
      if (lat == std::string::npos)
      {
        latVal = results[2*i+1].substr(5+results[2*i+1].find("lat\":"));
        lngVal = results[2*i].substr(5+results[2*i].find("lng\":"));
        lngVal = lngVal.substr(0,lngVal.size()-1);
      }
      else 
      { 
        latVal = results[2*i].substr(5+results[2*i].find("lat\":"));
        lngVal = results[2*i+1].substr(5+results[2*i+1].find("lng\":"));
        lngVal = lngVal.substr(0,lngVal.size()-1);
      }
      std::vector<double> newRow;
      std::string::size_type sz;
      newRow.push_back(std::stod(lngVal,&sz));
      newRow.push_back(std::stod(latVal,&sz)); 
      myRegion.push_back(newRow); 
    }
  }
}


void ParsePath::getRegion(std::vector<std::vector<double>>& region) 
{
  region = this->myRegion;
}

double ParsePath::getImplementWidth()
{
  return this->myImplementWidth;
}
