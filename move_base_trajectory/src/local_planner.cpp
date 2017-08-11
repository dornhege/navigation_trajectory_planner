#include "move_base_trajectory/local_planner.h"

namespace move_base_trajectory
{

LocalPlanner::LocalPlanner() :
    _localPlannerLoader("move_base_trajectory", "move_base_trajectory::BaseLocalPlannerTrajectory")
{
    if(!loadLocalPlanner("TODO LOAD PARAM")) {

    }
}

LocalPlanner::~LocalPlanner()
{
}

const moveit_msgs::RobotTrajectory & LocalPlanner::currentTrajectory() const
{

}

bool LocalPlanner::setTrajectory(const moveit_msgs::RobotTrajectory & traj)
{

}

bool LocalPlanner::executeCurrentTrajectory()
{

}

bool LocalPlanner::loadLocalPlanner(const std::string & localPlannerName)
{
    try {
        _localPlanner = _localPlannerLoader.createInstance(localPlannerName);

        //ROS_INFO("Created local_planner %s", local_planner.c_str());
        // TODO _localPlanner->initialize(_localPlannerLoader.getName(localPlannerName),
        // TODO         &tf_);
    } catch(pluginlib::PluginlibException & ex) {
        ROS_FATAL("%s: Failed to load local planner: %s Reason: %s",
                __func__, localPlannerName.c_str(), ex.what());
        return false;
    }
    return true;
}

}

