#include <boost/bind.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

#include "move_base_trajectory/move_base_trajectory.h"

ros::Publisher goalPub;
move_base_trajectory::MoveBaseTrajectory* moveBase;

void goalCallback(const geometry_msgs::PoseStamped& goal)
{
    // TODO convert goal to geo pose
    bonirob_navigation_msgs::MoveBaseGeoPoseActionGoal mbGoal;
    mbGoal.header.stamp = ros::Time::now();
    mbGoal.goal.target_pose = moveBase->getGlobalPose(goal);
    goalPub.publish(mbGoal);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_base_trajectory_node");
  ros::NodeHandle nh;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfl(tfBuffer);

  moveBase = new move_base_trajectory::MoveBaseTrajectory(tfBuffer);

  goalPub = nh.advertise<bonirob_navigation_msgs::MoveBaseGeoPoseActionGoal>("move_base/goal", 1);
  ros::Subscriber goalSub = nh.subscribe("move_base_simple/goal", 3, goalCallback);

  ros::Rate rate(50.0);
  while(ros::ok()) {
      ros::spinOnce();
      rate.sleep();
  }

  return 0;
}

