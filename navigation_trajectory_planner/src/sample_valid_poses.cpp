#include "navigation_trajectory_msgs/SampleValidPoses.h"

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>

#include <sstream> 

#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sample_valid_poses");
    ros::NodeHandle nh;

    if(argc < 3) {
        printf("Usage: %s planner_name n_poses [max_tries]\n", argv[0]);
        return 1;
    }

    ros::Publisher pubPoses = nh.advertise<geometry_msgs::PoseArray>("valid_poses", 3);

    std::stringstream serviceName;
    serviceName << "/move_base_node/" << argv[1] << "/sample_valid_poses";

    navigation_trajectory_msgs::SampleValidPoses srv;
    srv.request.n = atoi(argv[2]);
    if(argc >= 4)
        srv.request.max_tries = atoi(argv[3]);
    else
        srv.request.max_tries = 1000;

    if(!ros::service::call(serviceName.str(), srv)) {
        ROS_ERROR("Could not sample");
        return 1;
    }

    while(pubPoses.getNumSubscribers() <= 0) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    pubPoses.publish(srv.response.poses);
    ros::Duration(1.0).sleep();
}

