#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "move_base_trajectory/move_base_trajectory.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_base_trajectory_node");
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfl(tfBuffer);

  move_base_trajectory::MoveBaseTrajectory moveBase(tfBuffer);

  ros::Rate rate(50.0);
  while(ros::ok()) {
      ros::spinOnce();
      rate.sleep();
  }

  return 0;
}

