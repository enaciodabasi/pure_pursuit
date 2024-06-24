/**
 * @file pure_pursuit.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-06-24
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef PURE_PURSUIT_HPP_
#define PURE_PURSUIT_HPP_

#include "geometry_utilities/point.hpp"
#include "geometry_utilities/utils.hpp"

#include <optional>
#include <vector>

/* int findClosestPoint(const Point& current_point, const std::vector<Point>& path, const int previousClosestPointIndex = -1)
{
  bool foundClosestPoint = false;
  while(!foundClosestPoint)
  {
    for(std::vector<Point>::const_iterator pathSegmentPointIt = path.begin() + previousClosestPointIndex; pathSegmentPointIt != path.end(); pathSegmentPointIt++)
    {
            
    }
  }
} */

std::optional<int> getLookaheadPoint(
  const double lookahead_distance, 
  const int closestPointIndex,
  const std::vector<Point>& path,
  const Point& current_position  
)
{
  for(std::vector<Point>::const_iterator pointIter = path.cbegin() + closestPointIndex; pointIter != path.cend(); pointIter++)
  {
    
  }
}

#endif // PURE_PURSUIT_HPP_