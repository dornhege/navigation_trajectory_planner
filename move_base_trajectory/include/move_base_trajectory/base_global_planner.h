#ifndef BASE_GLOBAL_PLANNER_TRAJECTORY_H
#define BASE_GLOBAL_PLANNER_TRAJECTORY_H

#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <tf2_ros/buffer.h>

#include <string>

namespace move_base_trajectory
{

class BaseGlobalPlannerTrajectory
{
  public:      
      BaseGlobalPlannerTrajectory(){}

      virtual ~BaseGlobalPlannerTrajectory(){}

      virtual void initialize(std::string name) = 0;
      virtual bool updateForPlanRequest(geometry_msgs::PoseStamped& start, 
                                        geometry_msgs::PoseStamped& goal) = 0;
      virtual bool makeTrajectory(const geometry_msgs::PoseStamped& startPose,
                                  const geometry_msgs::PoseStamped& goalPose, 
                                  moveit_msgs::DisplayTrajectory & traj) = 0;

      virtual std::string planningFrame() const = 0;

      virtual bool foundTrajectory() const = 0;
//      virtual bool foundPrefix() const = 0;
      virtual bool getCurrentBestTrajectory(moveit_msgs::DisplayTrajectory & dtraj) const = 0;
//      virtual bool getCurrentBestPrefix(moveit_msgs::DisplayTrajectory & dtraj) const = 0;
      void setTfBuffer(tf2_ros::Buffer* tf);
    protected:
      tf2_ros::Buffer* _tf;
};

}

#endif

