#include "move_base_trajectory/base_global_planner.h"
#include <ros/ros.h>

namespace move_base_trajectory
{

bool BaseGlobalPlannerTrajectory::makePlan(const geometry_msgs::PoseStamped& start, 
        const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
{
    ROS_ERROR("%s: This function should not be called. Something is wrong.", __PRETTY_FUNCTION__);
    return false;
}

}

