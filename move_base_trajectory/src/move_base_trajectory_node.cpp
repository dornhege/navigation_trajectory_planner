#include <string>
#include <Eigen/Geometry>
#include <boost/bind.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

#include "move_base_trajectory/move_base_trajectory.h"

ros::Publisher goalPub;
move_base_trajectory::MoveBaseTrajectory* moveBase;
tf2_ros::Buffer* tfBuffer;

bool getAffineFromTf(const std::string& frame_id, const std::string& child_id,
                     const ros::Time& time,
                     Eigen::Affine3d& affine,
                     const ros::Duration& timeout=ros::Duration(0)){
    geometry_msgs::TransformStamped transformMsg;
    try{
        transformMsg = tfBuffer->lookupTransform(frame_id, child_id, time, timeout);
    }catch(tf2::TransformException ex){
        ROS_WARN("%s",ex.what());
        return false;
    }
    affine = tf2::transformToEigen(transformMsg).cast<double>();
    return true;
}


void goalCallback(const geometry_msgs::PoseStamped& goalMsg)
{
    geometry_msgs::PoseStamped transformedGoalMsg = goalMsg;
    bonirob_navigation_msgs::MoveBaseGeoPoseActionGoal mbGoal;
    mbGoal.header.stamp = ros::Time::now();
    if(goalMsg.header.frame_id != "gps_reference"){
        Eigen::Affine3d goalToGPSTransform = Eigen::Affine3d::Identity();
        if(!getAffineFromTf("gps_reference", goalMsg.header.frame_id, goalMsg.header.stamp,
                            goalToGPSTransform, ros::Duration(0.1))){
            ROS_ERROR("Could not transform goal pose to gps_reference frame. Ignoring goal!");
            return;
        }
        Eigen::Affine3d goal;
        tf::poseMsgToEigen(goalMsg.pose, goal);
        goal = goalToGPSTransform*goal;
        tf::poseEigenToMsg(goal, transformedGoalMsg.pose);
        transformedGoalMsg.header.frame_id = "gps_reference";
    }
    mbGoal.goal.target_pose = moveBase->getGlobalPose(transformedGoalMsg);
    goalPub.publish(mbGoal);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_base_trajectory_node");
  ros::NodeHandle nh;

  tfBuffer = new tf2_ros::Buffer();
  tf2_ros::TransformListener tfl(*tfBuffer);

  moveBase = new move_base_trajectory::MoveBaseTrajectory(*tfBuffer);

  goalPub = nh.advertise<bonirob_navigation_msgs::MoveBaseGeoPoseActionGoal>("move_base/goal", 1);
  ros::Subscriber goalSub = nh.subscribe("move_base_simple/goal", 3, goalCallback);

  ros::Rate rate(50.0);
  while(ros::ok()) {
      ros::spinOnce();
      rate.sleep();
  }

  return 0;
}

