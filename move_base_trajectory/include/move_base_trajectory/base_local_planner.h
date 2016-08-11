#ifndef BASE_LOCAL_PLANNER_TRAJECTORY_H
#define BASE_LOCAL_PLANNER_TRAJECTORY_H

#include <nav_core/base_local_planner.h>
#include <moveit_msgs/RobotTrajectory.h>

namespace move_base_trajectory
{

class BaseLocalPlannerTrajectory : public nav_core::BaseLocalPlanner
{
  public:
    virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

    virtual bool setTrajectory(const moveit_msgs::RobotTrajectory & traj) = 0;
};

}

#endif

