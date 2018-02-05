#include "boustrophedon.h"

Boustrophedon::Boustrophedon(double implementWidth) 
{
  myImplementWidth = implementWidth;
  messageSequence = 0;
}

std::string Boustrophedon::planPath(std::string region, grudsby_sweeping::MowingPlan& plan)
{
  
  ParsePath parser(region, myImplementWidth);

  //initialize polygon
  std::vector<std::vector<double>> polygon;
  parser.getRegion(polygon);
  
  //the width of the mowing implement
  double implementWidth = parser.getImplementWidth();

  //create vector of segments
  std::vector<Segment> segments;
  for(int i = 0; i < polygon.size() - 1; i++)
  {
    Segment newSegment = Segment(polygon[i][0], polygon[i][1],polygon[i+1][0],polygon[i+1][1]);
    segments.push_back(newSegment);
  }
  Segment newSegment = Segment(polygon[polygon.size()-1][0], polygon[polygon.size()-1][1],polygon[0][0],polygon[0][1]);
  segments.push_back(newSegment);
 
/*  for(int i = 0; i < segments.size(); i++)
  {
    printf("Segment %d: (%.2f, %.2f) (%.2f, %.2f)\n", i, segments[i].a.X, segments[i].a.Y, segments[i].b.X, segments[i].b.Y);
  }*/

  //find angle of slice (find the longest line segment and go parallel to it)
  Segment longestSegment = segments[0];
  for(int i = 0; i < segments.size(); i++)
  {
    
    if(segments[i].getLength() > longestSegment.getLength())
    {
      longestSegment = segments[i];
    }
  }
  //printf("Segment: (%.2f, %.2f) (%.2f, %.2f)\n", longestSegment.a.X, longestSegment.a.Y, longestSegment.b.X, longestSegment.b.Y);

  Vector2 orthogonal = Vector2::Normal(longestSegment.b - longestSegment.a);
  //find starting point of slice (start at min X and min Y, the find closest point to that line)
  double minXVal = polygon[0][0];
  double minYVal = polygon[0][1];
  for(int i = 1; i < polygon.size(); i++)
  {
    if(polygon[i][0] < minXVal)
    {
      minXVal = polygon[i][0];
    }
    if(polygon[i][1] < minYVal)
    {
      minYVal = polygon[i][1];
    }
  }

  Slice slice = Slice(minXVal, minYVal, orthogonal.X, orthogonal.Y);

  std::vector<double> closestPoint = polygon[0];
  double closestDistance = slice.distanceToPoint(polygon[0]);
  for(int i = 1; i < polygon.size(); i++)
  {
    double d = slice.distanceToPoint(polygon[i]);
    
    if(d < closestDistance)
    {
      closestDistance = d;
      closestPoint = polygon[i];
    }
  }
  slice.setPoint(closestPoint[0], closestPoint[1]);
 
  std::vector<Vector2> waypoints;
  //start slicing
  bool slicing = true;
  int numSlices = 0;
  while(slicing)
  {
    std::vector<Vector2> pointsList;
    slicing = false;
    for(int i = 0; i < segments.size(); i++)
    {
      if(slice.intersectsOnce(segments[i]))
      {
        
        //printf("Segment %d: (%.2f, %.2f) (%.2f, %.2f)\n", i, segments[i].a.X, segments[i].a.Y, segments[i].b.X, segments[i].b.Y);
        slicing = true;
        Vector2 intersectPoint = slice.findIntersection(segments[i]);
        if(std::find(pointsList.begin(), pointsList.end(), intersectPoint) == pointsList.end())       
        {
          waypoints.push_back(intersectPoint);
        }        
        pointsList.push_back(intersectPoint);

      }
    }
    
    //switch waypoints on every odd slice
    Vector2 holder = waypoints[2*numSlices + 1 - numSlices%2];
    waypoints[2*numSlices] = waypoints[2*numSlices + numSlices%2];
    waypoints[2*numSlices + 1] = holder;

    slice.increment(implementWidth);
    numSlices++;
  }
  std::stringstream ss;
  plan.waypoints.resize(0); 
  ss << "{\"coordinates\":[";
  for (int i = 0; i < waypoints.size()-1; i++)
  {
    ss << "{\"lat\":";
    ss << std::setprecision(18) << std::fixed << waypoints[i].Y;
    ss << ",\"lng\":";
    ss << std::setprecision(18) << std::fixed << waypoints[i].X;
    ss << "}";
    if (i < (waypoints.size()-2))
    {
      ss << ",";
    }
    grudsby_sweeping::SimpleLatLng newPoint;
    newPoint.latitude = waypoints[i].Y;
    newPoint.longitude = waypoints[i].X;
    plan.waypoints.push_back(newPoint); 
    //printf("Waypoint %d: (%.2f, %.2f)\n", i, waypoints[i].X, waypoints[i].Y);
  }
  ss << "],\"regionID\":\"sve\"}" << std::endl;
  plan.header.seq = messageSequence++;
  plan.header.stamp = ros::Time::now();
  

  Segment seg = Segment(-2, -1, 2, -1);

  Vector2 intersect = slice.findIntersection(seg);
  bool test = slice.intersectsOnce(seg);
  //printf("Intersection point: %.2f, %.2f", intersect.X, intersect.Y);

  return ss.str();
}
