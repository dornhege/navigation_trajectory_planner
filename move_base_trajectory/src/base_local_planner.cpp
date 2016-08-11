#include "move_base_trajectory/base_local_planner.h"
#include <ros/ros.h>

namespace move_base_trajectory
{

bool BaseLocalPlannerTrajectory::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
    ROS_ERROR("%s: This function should not be called. Something is wrong.", __PRETTY_FUNCTION__);
    return false;
}

}

