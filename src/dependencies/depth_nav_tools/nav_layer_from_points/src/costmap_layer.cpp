/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2015, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     1. Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *     2. Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *     3. Neither the name of the copyright holder nor the names of its
 *        contributors may be used to endorse or promote products derived
 *        from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
/**
 * @file   main.cpp
 * @author Michal Drwiega (drwiega.michal@gmail.com)
 * @date   10.2015
 * @brief  nav_layer_from_points package
 */

#include <nav_layer_from_points/costmap_layer.h>

#include <iostream>
#include <fstream>

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

PLUGINLIB_EXPORT_CLASS(nav_layer_from_points::NavLayerFromPoints, costmap_2d::Layer)

namespace nav_layer_from_points
{
  //=================================================================================================
  void NavLayerFromPoints::onInitialize()
  {
    current_ = true;
    first_time_ = true;

    ros::NodeHandle nh("~/" + name_), g_nh;
    sub_points_ = nh.subscribe("/downstairs_detector/points", 1,
                               &NavLayerFromPoints::pointsCallback, this);

    rec_server_ = new dynamic_reconfigure::Server<NavLayerFromPointsConfig>(nh);
    f_ = boost::bind(&NavLayerFromPoints::configure, this, _1, _2);
    rec_server_->setCallback(f_);
  }

  //=================================================================================================
  void NavLayerFromPoints::configure(NavLayerFromPointsConfig &config, uint32_t level)
  {
    points_keep_time_ = ros::Duration(config.keep_time);
    enabled_ = config.enabled;

    point_radius_ = config.point_radius;
    robot_radius_ = config.robot_radius;
  }

  //=================================================================================================
  void NavLayerFromPoints::pointsCallback(const depth_nav_msgs::Point32List& points)
  {
    boost::recursive_mutex::scoped_lock lock(lock_);
    points_list_ = points;
  }

  //=================================================================================================
  void NavLayerFromPoints::clearTransformedPoints()
  {
    std::list<geometry_msgs::PointStamped>::iterator p_it;
    p_it = transformed_points_.begin();

    if (transformed_points_.size() > 10000)
      transformed_points_.clear();

    while(p_it != transformed_points_.end())
    {
      if(ros::Time::now() - (*p_it).header.stamp > points_keep_time_)
        p_it = transformed_points_.erase(p_it);
      else
        ++p_it;
    }
  }

  //=================================================================================================
  void NavLayerFromPoints::updateBounds(double origin_x, double origin_y, double origin_z,
                                    double* min_x, double* min_y, double* max_x, double* max_y)
  {
    boost::recursive_mutex::scoped_lock lock(lock_);

    std::string global_frame = layered_costmap_->getGlobalFrameID();

    // Check if there are points to remove in transformed_points list and if so, then remove it
    if(!transformed_points_.empty())
      clearTransformedPoints();

    // Add points to PointStamped list transformed_points_
    for(unsigned int i = 0; i < points_list_.points.size(); i++)
    {
      geometry_msgs::PointStamped tpt;
      geometry_msgs::PointStamped pt, out_pt;

      tpt.point = costmap_2d::toPoint(points_list_.points[i]);

      try
      {
        pt.point.x = tpt.point.x;
        pt.point.y = 0;
        pt.point.z =  tpt.point.z;
        pt.header.frame_id = points_list_.header.frame_id;

        tf_.transformPoint(global_frame, pt, out_pt);
        tpt.point.x = out_pt.point.x;
        tpt.point.y = out_pt.point.y;
        tpt.point.z = out_pt.point.z;

        tpt.header.stamp = pt.header.stamp;
        //s << " ( " << tpt.point.x << " , " << tpt.point.y << " , " << tpt.point.z << " ) ";
        transformed_points_.push_back(tpt);
      }
      catch(tf::LookupException& ex)
      {
        ROS_ERROR("No Transform available Error: %s\n", ex.what());
        continue;
      }
      catch(tf::ConnectivityException& ex)
      {
        ROS_ERROR("Connectivity Error: %s\n", ex.what());
        continue;
      }
      catch(tf::ExtrapolationException& ex)
      {
        ROS_ERROR("Extrapolation Error: %s\n", ex.what());
        continue;
      }
    }

    //ROS_INFO_STREAM_THROTTLE(2,"transformed_points = " << s.str());

    updateBoundsFromPoints(min_x, min_y, max_x, max_y);

    if(first_time_)
    {
      last_min_x_ = *min_x;
      last_min_y_ = *min_y;
      last_max_x_ = *max_x;
      last_max_y_ = *max_y;
      first_time_ = false;
    }
    else
    {
      double a = *min_x, b = *min_y, c = *max_x, d = *max_y;
      *min_x = std::min(last_min_x_, *min_x);
      *min_y = std::min(last_min_y_, *min_y);
      *max_x = std::max(last_max_x_, *max_x);
      *max_y = std::max(last_max_y_, *max_y);
      last_min_x_ = a;
      last_min_y_ = b;
      last_max_x_ = c;
      last_max_y_ = d;
    }
    std::ostringstream s;
    s << " list_size = " << transformed_points_.size() << "   ";
    s << " min_x = " << *min_x << " max_x = " << *max_x <<
         " min_y = " << *min_y << " max_y = " << *max_y << "      ";
    ROS_INFO_STREAM_THROTTLE(2,"transformed_points = " << s.str());

  }

  //=================================================================================================
  void NavLayerFromPoints::updateBoundsFromPoints(double* min_x, double* min_y, double* max_x, double* max_y)
  {
    std::list<geometry_msgs::PointStamped>::iterator p_it;

    double radius = point_radius_ + robot_radius_;

    for(p_it = transformed_points_.begin(); p_it != transformed_points_.end(); ++p_it)
    {
      geometry_msgs::PointStamped pt = *p_it;

      *min_x = std::min(*min_x, pt.point.x - radius);
      *min_y = std::min(*min_y, pt.point.y - radius);
      *max_x = std::max(*max_x, pt.point.x + radius);
      *max_y = std::max(*max_y, pt.point.y + radius);
    }
  }

  //=================================================================================================
  void NavLayerFromPoints::updateCosts(costmap_2d::Costmap2D& master_grid,
                                   int min_i, int min_j, int max_i, int max_j)
  {
    boost::recursive_mutex::scoped_lock lock(lock_);

    if(!enabled_)
      return;

    if( points_list_.points.size() == 0 )
      return;

    std::list<geometry_msgs::PointStamped>::iterator p_it;

    costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
    double res = costmap->getResolution();

    for(p_it = transformed_points_.begin(); p_it != transformed_points_.end(); ++p_it)
    {
      geometry_msgs::Point pt = (*p_it).point;

      unsigned int size = std::max(1, int( (point_radius_ + robot_radius_) / res ));
      unsigned int map_x, map_y;
      unsigned int size_x = size, size_y = size;
      unsigned int start_x, start_y, end_x, end_y;

      // Check if point is on map
      // Convert from world coordinates to map coordinates with checking for legal bounds
      if (costmap->worldToMap(pt.x, pt.y, map_x, map_y))
      {
        start_x = map_x - size_x / 2;
        start_y = map_y - size_y / 2;
        end_x = map_x + size_x / 2;
        end_y = map_y + size_y / 2;

        if (start_x < min_i)
          start_x = min_i;
        if (end_x > max_i)
          end_x = max_i;
        if (start_y < min_j)
          start_y = min_j;
        if (end_y > max_j)
          end_y = max_j;

        for(int j = start_y; j < end_y; j++)
        {
          for(int i = start_x; i < end_x; i++)
          {
            unsigned char old_cost = costmap->getCost(i, j);
//            unsigned char old_cost = costmap->getCost(map_x, map_y);

            if(old_cost == costmap_2d::NO_INFORMATION)
              continue;

            costmap->setCost(i, j, costmap_2d::LETHAL_OBSTACLE);
          }
        }

        //=============================================================================================
        /* int map_x, map_y;
      costmap->worldToMapNoBounds(pt.x, pt.y, map_x, map_y);

      int start_x = 0, start_y = 0, end_x = size, end_y = size;

      if(map_x < 0)
        start_x = -map_x;
      else if(map_x + size > costmap->getSizeInCellsX())
        end_x = std::max(0, (int)costmap->getSizeInCellsX() - map_x);

      if((int)(start_x+map_x) < min_i)
        start_x = min_i - map_x;

      if((int)(end_x+map_x) > max_i)
        end_x = max_i - map_x;

      if(map_y < 0)
        start_y = -map_y;
      else if(map_y + size > costmap->getSizeInCellsY())
        end_y = std::max(0, (int) costmap->getSizeInCellsY() - map_y);

      if((int)(start_y+map_y) < min_j)
        start_y = min_j - map_y;

      if((int)(end_y+map_y) > max_j)
        end_y = max_j - map_y;

      // Set cost for costmap poles
      for(int i = start_x; i < end_x; i++)
      {
        for(int j = start_y; j < end_y; j++)
        {
          unsigned char old_cost = costmap->getCost(i + map_x, j + map_y);

          //if(old_cost == costmap_2d::NO_INFORMATION)
            //continue;

          costmap->setCost(i + map_x, j + map_y, costmap_2d::LETHAL_OBSTACLE);
        }
      }*/
      }
    }
  }
} // end of namespace
//=================================================================================================
//=================================================================================================
