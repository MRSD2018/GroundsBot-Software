// Copyright 2000 softSurfer, 2012 Dan Sunday
// This code may be freely used and modified for any purpose
// providing that this copyright notice is included with it.
// SoftSurfer makes no warranty for this code, and cannot be held
// liable for any real or imagined damage resulting from its use.
// Users of this code must verify correctness for their application.
 

#ifndef WINDING_NUM_H
#define WINDING_NUM_H
#include "ros/console.h" 
#include "ros/ros.h" 

#include "Vector2.hpp"
#include <vector>
// a Point is defined by its coordinates {int x, y;}
//===================================================================

namespace winding_num
{

// isLeft(): tests if a point is Left|On|Right of an infinite line.
//    Input:  three points P0, P1, and P2
//    Return: >0 for P2 left of the line through P0 and P1
//            =0 for P2  on the line
//            <0 for P2  right of the line
//    See: Algorithm 1 "Area of Triangles and Polygons"
double isLeft( std::vector<double> P0, std::vector<double> P1, std::vector<double> P2 )
{
    return ( (P1[0] - P0[0]) * (P2[1] - P0[1])
            - (P2[0] -  P0[0]) * (P1[1] - P0[1]) );
}
//===================================================================


// wn_PnPoly(): winding number test for a point in a polygon
//      Input:   P = a point,
//               V[] = vertex points of a polygon V[n+1] with V[n]=V[0]
//      Return:  wn = the winding number (=0 only when P is outside)
double wn_PnPoly( std::vector<double> P, std::vector<std::vector<double> > V)
{
    int    wn = 0;    // the  winding number counter
    //ROS_ERROR("Winding P Size: %d V Size %d x %d",P.size(), V.size(), V[0].size());     

    int n = V.size()-1; 
    // loop through all edges of the polygon
    for (int i=0; i<n; i++) {   // edge from V[i] to  V[i+1]
        if (V[i][1] <= P[1]) {          // start y <= P.y
            if (V[i+1][1]  > P[1])      // an upward crossing
                 if (isLeft( V[i], V[i+1], P) > 0)  // P left of  edge
                     ++wn;            // have  a valid up intersect
        }
        else {                        // start y > P.y (no test needed)
            if (V[i+1][1]  <= P[1])     // a downward crossing
                 if (isLeft( V[i], V[i+1], P) < 0)  // P right of  edge
                     --wn;            // have  a valid down intersect
        }
    }
    return wn;
}

//===================================================================

}
#endif
