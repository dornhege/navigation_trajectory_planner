#ifndef LOCAL_PLANNER_H
#define LOCAL_PLANNER_H

#include <ros/ros.h>
#include "moveit_msgs/RobotTrajectory.h"
#include <move_base_trajectory/base_local_planner.h>
#include <pluginlib/class_loader.h>

namespace move_base_trajectory
{

class LocalPlanner
{
    public:
        LocalPlanner();
        ~LocalPlanner();

        /// \returns the trajectory that is currently being executed.
        const moveit_msgs::RobotTrajectory & currentTrajectory() const;

        /// Set a new trajectory to execute.
        bool setTrajectory(const moveit_msgs::RobotTrajectory & traj);

        /// \returns true, if a command was executed.
        bool executeCurrentTrajectory();

    protected:
        bool loadLocalPlanner(const std::string & localPlannerName);

    protected:
        moveit_msgs::RobotTrajectory _currentTrajectory;

        ros::Publisher _pubVelocity;

        boost::shared_ptr<move_base_trajectory::BaseLocalPlannerTrajectory> _localPlanner;
        pluginlib::ClassLoader<move_base_trajectory::BaseLocalPlannerTrajectory> _localPlannerLoader;
};

}

#endif

