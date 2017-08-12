#include "move_base_trajectory/global_planner_thread.h"

#include <ros/ros.h>
#include <string>

namespace move_base_trajectory
{

GlobalPlannerThread::GlobalPlannerThread() :   
    _globalPlannerLoader("move_base_trajectory", "move_base_trajectory::BaseGlobalPlannerTrajectory")
{
    ros::NodeHandle privateNh("~");
    std::string globalPlannerName;

    privateNh.param("base_global_planner", globalPlannerName, std::string(""));

    loadGlobalPlanner(globalPlannerName);
}

GlobalPlannerThread::~GlobalPlannerThread()
{
}

bool GlobalPlannerThread::getBestTrajectory(moveit_msgs::RobotTrajectory & traj) const
{

}

bool GlobalPlannerThread::isComputing() const
{

}

bool GlobalPlannerThread::computeTrajectory(const geometry_msgs::PoseStamped & goal)
{

}

bool GlobalPlannerThread::foundTrajectory() const
{

}

void GlobalPlannerThread::stopTrajectoryComputation()
{

}


bool GlobalPlannerThread::loadGlobalPlanner(const std::string & globalPlannerName)
{
    try {
        _globalPlanner = _globalPlannerLoader.createInstance(globalPlannerName);
        //ROS_INFO("Created global_planner %s", local_planner.c_str());
        _globalPlanner->initialize(_globalPlannerLoader.getName(globalPlannerName));
    } catch(pluginlib::PluginlibException & ex) {
        ROS_FATAL("%s: Failed to load global planner: %s Reason: %s",
                __func__, globalPlannerName.c_str(), ex.what());
        return false;
    }
    return true;
}

}

