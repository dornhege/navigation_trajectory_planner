#ifndef BASE_GLOBAL_PLANNER_TRAJECTORY_H
#define BASE_GLOBAL_PLANNER_TRAJECTORY_H

#include <nav_core/base_global_planner.h>
#include <moveit_msgs/RobotTrajectory.h>

namespace move_base_trajectory
{

class BaseGlobalPlannerTrajectory : public nav_core::BaseGlobalPlanner
{
  public:
    virtual bool makePlan(const geometry_msgs::PoseStamped& start, 
            const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

    virtual bool makeTrajectory(const geometry_msgs::PoseStamped& start, 
            const geometry_msgs::PoseStamped& goal, moveit_msgs::RobotTrajectory & traj) = 0;
};

}

#endif

