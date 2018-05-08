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
        LocalPlanner(tf2_ros::Buffer & tf);
        ~LocalPlanner();

        /// \returns the trajectory that is currently being executed.
        const moveit_msgs::RobotTrajectory & currentTrajectory() const;

        /// Set a new trajectory to execute.
        bool setTrajectory(const moveit_msgs::RobotTrajectory & traj);

        /// \returns true, if a command was executed.
        bool executeCurrentTrajectory();

        bool isGoalReached() const;

        std::string planningFrame() const;

        virtual double distanceToGoal() const;
        virtual double angleToGoal() const;
        virtual double angleAtGoal() const;
        //virtual geographic_msgs::GeoPoseStamped robotPosition() const;
        virtual void printState() const;

    protected:
        bool loadLocalPlanner(const std::string & localPlannerName, tf2_ros::Buffer & tf);

    protected:
        moveit_msgs::RobotTrajectory _currentTrajectory;

        ros::Publisher _velocityPublisher;

        boost::shared_ptr<move_base_trajectory::BaseLocalPlannerTrajectory> _localPlanner;
        pluginlib::ClassLoader<move_base_trajectory::BaseLocalPlannerTrajectory> _localPlannerLoader;
};

}

#endif

