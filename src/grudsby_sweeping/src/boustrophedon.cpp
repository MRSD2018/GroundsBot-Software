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
//  polygon = {{0,0}, {2, 0}, {4, 3}, {-2, 5}};
  //the width of the mowing implement
  double implementWidth = parser.getImplementWidth();
//  implementWidth = 1;

  //create vector of segments
  std::vector<Segment> segments;
  for(int i = 0; i < polygon.size() - 1; i++)
  {
    Segment newSegment = Segment(polygon[i][0], polygon[i][1],polygon[i+1][0],polygon[i+1][1]);
    segments.push_back(newSegment);
  }
  Segment newSegment = Segment(polygon[polygon.size()-1][0], polygon[polygon.size()-1][1],polygon[0][0],polygon[0][1]);
  segments.push_back(newSegment);
 
  for(int i = 0; i < segments.size(); i++)
  {
    ROS_INFO("Segment %d: (%.1f, %.1f) (%.1f, %.1f)\n", i, segments[i].a.X, segments[i].a.Y, segments[i].b.X, segments[i].b.Y);
  }

  //find angle of slice (find the longest line segment and go parallel to it)
  Segment longestSegment = segments[0];
  for(int i = 0; i < segments.size(); i++)
  {
    
    if(segments[i].getLength() > longestSegment.getLength())
    {
      longestSegment = segments[i];
    }
  }
  ROS_INFO("Longest Segment: (%.1f, %.1f) (%.1f, %.1f)\n", longestSegment.a.X, longestSegment.a.Y, longestSegment.b.X, longestSegment.b.Y);

  Vector2 orthogonal = Vector2::Normal(longestSegment.b - longestSegment.a);

  //find starting point of slice (start at min X and min Y, the find closest point to that line)
  double minXVal = polygon[0][0];
  double minYVal = polygon[0][1];
  for(int i = 1; i < polygon.size(); i++)
  {
    if(sign(orthogonal.X)*polygon[i][0] < sign(orthogonal.X)*minXVal)
    {
      minXVal = polygon[i][0];
    }
    if(sign(orthogonal.Y)*polygon[i][1] < sign(orthogonal.Y)* minYVal)
    {
      minYVal = polygon[i][1];
    }
  }
  
  ROS_INFO("Slice: (%.10f, %.10f) (%.10f, %.10f)\n", minXVal, minYVal, orthogonal.X, orthogonal.Y);
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
  ROS_INFO("Closest Point: (%.1f, %.1f)\n", closestPoint[0], closestPoint[1]);
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
        
        slicing = true;
        Vector2 intersectPoint = slice.findIntersection(segments[i]);
        if(std::find(pointsList.begin(), pointsList.end(), intersectPoint) == pointsList.end())       
        {
          pointsList.push_back(intersectPoint);
          ROS_INFO("Intersect Point: (%.10f, %.10f)\n", intersectPoint.X, intersectPoint.Y);
          //waypoints.push_back(intersectPoint);
          //ROS_ERROR("Waypoint: (%.10f, %.10f)\n", waypoints[waypoints.size()-1].X, waypoints[waypoints.size()-1].Y);
        }        

      }
    }

    for (int j = 0; j < pointsList.size(); j++)
    {
      ROS_INFO("Point in List %d: (%.10f, %.10f)",j, pointsList[j].X, pointsList[j].Y);
    }
    if(pointsList.size() == 2)
    {
      int offset = slice.directionToPoint(polygon[0]) > 0; 
      Vector2 holder = pointsList[(numSlices + offset)%2];
      pointsList[1] = pointsList[1-(numSlices + offset)%2];
      pointsList[0] = holder;
      
    }
    for (int j = 0; j < pointsList.size(); j++)
    {
      waypoints.push_back(pointsList[j]);
    }
    /*
    for (int j = 0; j < waypoints.size(); j++)
    {
      ROS_ERROR("Waypoint %d: (%.10f, %.10f)",j, waypoints[j], waypoints[j]);
    }*/      
  
   slice.increment(implementWidth);
    numSlices++;
  }
  
  /*
  for (int i = 0; i < waypoints.size()-1; i++)
  {
    ROS_ERROR("Waypoint %d: (%.10f, %.10f)",i, waypoints[i], waypoints[i]);
  }
  */
  std::stringstream ss;
  plan.waypoints.resize(0); 
  ss << "{\"coordinates\":[";
  for (int i = 0; i < waypoints.size(); i++)
  {
    ss << "{\"lat\":";
    ss << std::setprecision(18) << std::fixed << waypoints[i].Y;
    ss << ",\"lng\":";
    ss << std::setprecision(18) << std::fixed << waypoints[i].X;
    ss << "}";
    if (i < (waypoints.size()-1))
    {
      ss << ",";
    }
    grudsby_sweeping::SimpleLatLng newPoint;
    newPoint.latitude = waypoints[i].Y;
    newPoint.longitude = waypoints[i].X;
    plan.waypoints.push_back(newPoint); 
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


