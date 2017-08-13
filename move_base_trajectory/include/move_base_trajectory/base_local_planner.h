#ifndef BASE_LOCAL_PLANNER_TRAJECTORY_H
#define BASE_LOCAL_PLANNER_TRAJECTORY_H

#include <string>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/buffer.h>
#include <moveit_msgs/RobotTrajectory.h>

namespace move_base_trajectory
{

class BaseLocalPlannerTrajectory
{
  public:
      virtual ~BaseLocalPlannerTrajectory() {}

      virtual bool computeVelocityCommands(geometry_msgs::Twist & cmd_vel) = 0;

      virtual bool isGoalReached() const = 0;

      virtual void initialize(std::string name, tf2_ros::Buffer* tf) = 0;

      virtual bool setTrajectory(const moveit_msgs::RobotTrajectory & traj) = 0;

      virtual std::string planningFrame() const = 0;
};

}

#endif

