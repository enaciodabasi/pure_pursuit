/**
 * @file pure_pursuit.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-06-24
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "pure_pursuit/pure_pursuit.hpp"
#include "geometry_utilities/point.hpp"
#include "geometry_utilities/utils.hpp"
#include <cmath>
#include <iterator>

namespace pure_pursuit
{
  std::expected<VelocityCommand, error::Error> PurePursuitController::generateVelocityCommand()
  {

    Point currentRobotPosition;

    if (auto currentRobotPositionPtr = m_RobotPosition.lock())
    {
      currentRobotPosition = *currentRobotPositionPtr;
    }
    else
    {

      return std::unexpected(error::Error::RobotPositionFetchError);
    }

    const auto currentLineSegmentBoundary = prunePath();
    if (currentLineSegmentBoundary.second == (*m_CurrentPath).end())
    {
      return std::unexpected(error::Error::LookaheadPointError);
    }

    const double requiredCurvature = this->calculateCurvature((*currentLineSegmentBoundary.second));

    return calculateVelocity(requiredCurvature);
  }

  std::pair<Path::iterator, Path::iterator> PurePursuitController::prunePath()
  {

    std::pair<Path::iterator, double> currentClosestPointInfo = m_PreviousClosestPoint;
    Path::iterator lookaheadPoint = m_CurrentPath->end();

    for (Path::iterator pathIt = std::next(currentClosestPointInfo.first); pathIt != (*m_CurrentPath).end(); pathIt++)
    {
      double distanceDiff = distanceBetweenTwoPoints(*currentClosestPointInfo.first, *pathIt);
      if (distanceDiff >= m_ControllerParams.lookaheadDistance)
      {
        lookaheadPoint = pathIt;
        break;
      }
    }

    return std::make_pair(m_PreviousClosestPoint.first, lookaheadPoint);
  }

  bool PurePursuitController::findClosestPoint(const Point &robot_position)
  {

    bool pointAtLeastLookaheadDistanceAhead = false;

    std::pair<Path::iterator, double> closestPoint(m_CurrentPath->end(), m_ControllerParams.lookaheadDistance + 1.0);
    for (Path::iterator pathIter = m_PreviousClosestPoint.first + 1; pathIter != m_CurrentPath->end(); pathIter)
    {
      double dist = distanceBetweenTwoPoints(robot_position, *(pathIter));

      if (dist > m_ControllerParams.lookaheadDistance)
      {
        break;
      }

      if (dist < closestPoint.second)
      {
        closestPoint.first = pathIter;
        closestPoint.second = dist;
      }
    }

    // If closest point is found, e.g the iterator actually points to something and the distance is not larger than the lookahead distance:
    if (closestPoint.first != m_CurrentPath->end() && closestPoint.second < m_ControllerParams.lookaheadDistance)
    {
      m_PreviousClosestPoint = closestPoint;
      return true;
    }

    return false;
  }

  double PurePursuitController::calculateCurvature(const Point &lookaheadPoint)
  {
    return 2.0 * lookaheadPoint.m_y / std::pow(m_ControllerParams.lookaheadDistance, 2);
  }

  std::optional<Point> PurePursuitController::circleLineSegmentIntercession(const std::pair<Path::iterator, Path::iterator> &segment_points)
  {

    const Point p1 = *(segment_points.first);
    const Point p2 = *(segment_points.second);

    double dx = p2.m_x - p1.m_x;
    double dy = p2.m_y - p1.m_y;
    double dr = std::sqrt(
        std::pow(dx, 2) + std::pow(dy, 2));

    double D = (p1.m_x * p2.m_y) - (p2.m_x * p1.m_y);

    double discriminant = (std::pow(m_ControllerParams.lookaheadDistance, 2) * std::pow(dr, 2)) - std::pow(D, 2);

    // no intersections found:
    if (discriminant < 0)
    {
      return std::nullopt;
    }

    double d1 = p1.m_x * p1.m_x + p1.m_y * p1.m_y;
    double d2 = p2.m_x * p2.m_x + p2.m_y * p2.m_y;
    double dd = d2 - d1;

    Point lookaheadPoint;
    lookaheadPoint.m_x = ((D * dy) + (std::copysign(1.0, dd) * dx * std::sqrt(discriminant))) / std::pow(dr, 2);
    lookaheadPoint.m_y = ((-D * dx) + (std::copysign(1.0, dd) * dy * std::sqrt(discriminant))) / std::pow(dr, 2);

    return lookaheadPoint;
  }

  VelocityCommand PurePursuitController::calculateVelocity(const double curvature)
  {
    return std::make_pair(m_ControllerParams.linearVelocity, m_ControllerParams.linearVelocity * curvature);
  }

} // End of namespace pure_pursuit