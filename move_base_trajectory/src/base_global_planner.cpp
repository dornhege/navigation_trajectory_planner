#include "move_base_trajectory/base_global_planner.h"
#include <ros/ros.h>

namespace move_base_trajectory
{
void BaseGlobalPlannerTrajectory::setTfBuffer(tf2_ros::Buffer* tf)
{
    _tf = tf;
}

}

