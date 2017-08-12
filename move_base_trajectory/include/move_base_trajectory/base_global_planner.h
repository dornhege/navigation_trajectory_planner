#ifndef BASE_GLOBAL_PLANNER_TRAJECTORY_H
#define BASE_GLOBAL_PLANNER_TRAJECTORY_H

#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/RobotTrajectory.h>

#include <string>

namespace move_base_trajectory
{

class BaseGlobalPlannerTrajectory
{
  public:      
      BaseGlobalPlannerTrajectory(){}

      virtual ~BaseGlobalPlannerTrajectory(){}

      virtual bool makeTrajectory(const geometry_msgs::PoseStamped& start, 
                                  const geometry_msgs::PoseStamped& goal, moveit_msgs::RobotTrajectory & traj) = 0;
      virtual void initialize(std::string name) = 0;

    protected:
};

}

#endif

