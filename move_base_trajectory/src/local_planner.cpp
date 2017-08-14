#include "move_base_trajectory/local_planner.h"

namespace move_base_trajectory
{

LocalPlanner::LocalPlanner(tf2_ros::Buffer & tf) : 
    _localPlannerLoader("move_base_trajectory", "move_base_trajectory::BaseLocalPlannerTrajectory")
{
    ros::NodeHandle privateNh("~");
    ros::NodeHandle publishNh("local_planner"); // TODO namespaces?

    std::string localPlannerName;
    privateNh.param("base_local_planner", localPlannerName, std::string(""));

    if(!loadLocalPlanner(localPlannerName, tf)) {
        exit(1);
    }

    _velocityPublisher = publishNh.advertise<geometry_msgs::Twist>("velocity_commands", 100);
}

LocalPlanner::~LocalPlanner()
{
}

const moveit_msgs::RobotTrajectory & LocalPlanner::currentTrajectory() const
{
    return _localPlanner->currentTrajectory();
}

bool LocalPlanner::setTrajectory(const moveit_msgs::RobotTrajectory & traj)
{
    return _localPlanner->setTrajectory(traj);
}

bool LocalPlanner::executeCurrentTrajectory()
{
    geometry_msgs::Twist velocityCommand;
    bool foundValidCommand = _localPlanner->computeVelocityCommands(velocityCommand);

    _velocityPublisher.publish(velocityCommand);
    return foundValidCommand;
}

bool LocalPlanner::isGoalReached() const
{
    return _localPlanner->isGoalReached();
}

std::string LocalPlanner::planningFrame() const
{
    return _localPlanner->planningFrame();
}

bool LocalPlanner::loadLocalPlanner(const std::string & localPlannerName, tf2_ros::Buffer & tf)
{
    try {
        _localPlanner = _localPlannerLoader.createInstance(localPlannerName);
        //ROS_INFO("Created local_planner %s", local_planner.c_str());
        _localPlanner->initialize(_localPlannerLoader.getName(localPlannerName), &tf);
    } catch(pluginlib::PluginlibException & ex) {
        ROS_FATAL("%s: Failed to load local planner: %s Reason: %s",
                __func__, localPlannerName.c_str(), ex.what());
        return false;
    }
    return true;
}

}

