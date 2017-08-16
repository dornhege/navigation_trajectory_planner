#include "move_base_trajectory/global_planner_thread.h"

#include <ros/ros.h>
#include <string>
#include <boost/thread.hpp>

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

bool GlobalPlannerThread::getBestTrajectory(moveit_msgs::DisplayTrajectory & traj) const
{
    if(foundTrajectory()){
        return _globalPlanner->getCurrentBestTrajectory(traj);
    }
    return false;
}

bool GlobalPlannerThread::isComputing()
{
    return !_plannerThread.timed_join(boost::posix_time::seconds(0));
}

bool GlobalPlannerThread::computeTrajectory(const geometry_msgs::PoseStamped & start, const geometry_msgs::PoseStamped & goal)
{
    // start thread, send goal to planner
    moveit_msgs::DisplayTrajectory trajectory; // TODO do we still need this in the global planner?
    // TODO check that no thread is running
    _plannerThread = boost::thread(boost::bind(&BaseGlobalPlannerTrajectory::makeTrajectory, _globalPlanner, start, goal, trajectory));
}

bool GlobalPlannerThread::foundTrajectory() const
{
    return _globalPlanner->foundTrajectory();
}

void GlobalPlannerThread::stopTrajectoryComputation()
{
    // TODO define interruption points for the thread.
    _plannerThread.interrupt();
}

std::string GlobalPlannerThread::planningFrame() const
{
    return _globalPlanner->planningFrame();
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

