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
#include <memory>

template<typename T>
int sgn(T num)
{
  return (num < 0 ? -1 : 1);
}

using Path = std::vector<Point>;
class PurePursuitController
{
  public:

  PurePursuitController() = default;

  ~PurePursuitController() = default;

  struct PurePursuitControllerParams
  {
    double lookaheadDistance;
  };

  std::pair<Path::iterator, Path::iterator> prunePath();

  bool update();

  private:

  bool init(PurePursuitControllerParams params);
  
  PurePursuitControllerParams m_ControllerParams;

  std::shared_ptr<Path> m_CurrentPath;

  std::weak_ptr<Point> m_RobotPosition;

  Path getCopyOfCurrentPath() const;

  /**
   * @brief first: iterator for the closest point
   * @brief second: calculated distance
   * 
   */
  std::pair<Path::iterator, double> m_PreviousClosestPoint;

  bool findClosestPoint(const Point& robot_position);

  double calculateCurvature(const Point& robot_position);

  std::optional<Point> circleLineSegmentIntercession(const std::pair<Path::iterator, Path::iterator>& segment_points);

};

#endif // PURE_PURSUIT_HPP_