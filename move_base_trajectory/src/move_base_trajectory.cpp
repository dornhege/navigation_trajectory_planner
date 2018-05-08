#include "move_base_trajectory/move_base_trajectory.h"

#include <angles/angles.h>
#include <ais_rosparam_tools/load_ros_parameters.h>
#include <ais_ros_msg_conversions/tf_msg_conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/tf.h>

namespace move_base_trajectory
{

MoveBaseTrajectory::MoveBaseTrajectory(tf2_ros::Buffer & tf) : _tf(tf), _actionServer(NULL), _localPlanner(tf)
{
    ros::NodeHandle nh;
    _actionServer = new MoveBaseActionServer(nh, "move_base",
                                             boost::bind(&MoveBaseTrajectory::executeCallbackGeoPose, this, _1), false);


    gps_reference_ = new gps_reference::GpsReference();
    if(!gps_reference_->gpsReferenceInitialized()){
        ROS_FATAL("Could not initialize GPS reference. Did you upload the GPS reference parameters?");
        exit(1);
    }

    ros::NodeHandle privateNh("~");
    _trajectoryPub = privateNh.advertise<moveit_msgs::DisplayTrajectory>("trajectory", 3);
    ais_rosparam_tools::checkAndLoadParameter(privateNh, "base_frame", _baseFrame, true);

    current_goal_pub_ = privateNh.advertise<geometry_msgs::PoseStamped>("global_goal", 0 );
    _actionServer->start();
}

MoveBaseTrajectory::~MoveBaseTrajectory()
{
    delete gps_reference_;
}

void MoveBaseTrajectory::executeCallbackGeoPose(const bonirob_navigation_msgs::MoveBaseGeoPoseGoalConstPtr& goal)
{
    geometry_msgs::PoseStamped target_pose = geoPoseGoalToPoseStamped(goal);
    // TODO publish goal marker
    current_goal_pub_.publish(target_pose);
    executeCallback(target_pose);
}

void MoveBaseTrajectory::executeCallback(const geometry_msgs::PoseStamped& target_pose)
{
    // check move base action is inactive
    // disable planner + local planenr
    ROS_INFO_STREAM("New move base call with goal " << target_pose.pose.position.x 
                    << " " << target_pose.pose.position.y
                    << " " << target_pose.pose.position.z
                    << " " << angles::to_degrees(tf::getYaw(target_pose.pose.orientation)) 
                    << "deg in frame " << target_pose.header.frame_id
                    << " (" << target_pose.header.stamp.toSec() << ")");

    ros::Rate loopRate(20);

    bool trigger_replan = false;  // TODO move to function for timeouts/etc. or remove if not needed
    moveit_msgs::RobotTrajectory current_trajectory;
    while(ros::ok()){
        if(_actionServer->isPreemptRequested()){
            _globalPlanner.stopTrajectoryComputation();
            _actionServer->setPreempted();
            break;
        }
        // if(not plan or trigger_replan) current_trajectory = computeGlobalTrajectory
        if(!isValidTrajectory(current_trajectory) || trigger_replan) {
            clearTrajectory(current_trajectory);
            ROS_INFO("cleared trajectory");

            //TODO use ais utility function
            //try{
            geometry_msgs::TransformStamped planningStartTf = _tf.lookupTransform(_globalPlanner.planningFrame(), _baseFrame, 
                                                                                  ros::Time(0), ros::Duration(1.5));
            //}
  
            geometry_msgs::PoseStamped planningStartPose;
            ais_ros_msg_conversions::TransformToPose(planningStartTf, planningStartPose);
            if(!startTrajectoryComputation(planningStartPose, target_pose)){
                _actionServer->setAborted(bonirob_navigation_msgs::MoveBaseGeoPoseResult(), "Trajectory computation could not be started. Maybe the start or goal state are in collision.");
                ROS_ERROR("Starting the trajectory computation failed.");
                break;
            }
        }

        enum GlobalTrajectoryComputationResult res = updateGlobalTrajectory(current_trajectory);
        switch(res) {
        case GTCR_SUCCESS:
            // nothing to do here, we got what we wanted...
            //ROS_INFO_STREAM("Found a trajectory of length " << current_trajectory.multi_dof_joint_trajectory.points.size());
            break;
        case GTCR_INCONSISTENT:
            ROS_ERROR("Got inconsistent behavior from the planner.");
        case GTCR_NO_TRAJECTORY:
            ROS_ERROR("No plan could be found.");
            _actionServer->setAborted(bonirob_navigation_msgs::MoveBaseGeoPoseResult(), "Planner could not find a plan at all");
            goto after_loop;
            break;
        case GTCR_PREEMPTED:
            goto after_loop;
            break;
        }
        trigger_replan = false;

        if(!isSameTrajectory(current_trajectory, _localPlanner.currentTrajectory())) {
            ROS_INFO("Attempting to set trajectory in local planner");
            if(!updateLocalPlannerTrajectory(current_trajectory)){
                ROS_ERROR("Trajectory for local planner could not be set.");
                _actionServer->setAborted(bonirob_navigation_msgs::MoveBaseGeoPoseResult(), "Trajectory for local planner could not be set.");
                break;
            }
        }

        if(!executeTrajectory()) {
            ROS_WARN("Could not find a valid trajectory. Triggering replan.");
            trigger_replan = true;
        }else{
            ROS_INFO_THROTTLE(5., "Executing velicity commands");
        }

        bonirob_navigation_msgs::MoveBaseGeoPoseFeedback feedback;
        feedback.base_position;
        feedback.dist_to_goal;
        feedback.angle_to_goal;
        feedback.angle_at_goal;
        _actionServer->publishFeedback(feedback);

        if(_localPlanner.isGoalReached()){
            _actionServer->setSucceeded(bonirob_navigation_msgs::MoveBaseGeoPoseResult(), "Goal reached!");
            break;
        }
        loopRate.sleep();
    }

after_loop:
    _globalPlanner.stopTrajectoryComputation();
}

bool MoveBaseTrajectory::startTrajectoryComputation(const geometry_msgs::PoseStamped & start, const geometry_msgs::PoseStamped & goal)
{
    ROS_INFO("starting trajectory computation in MoveBaseTrajectory");
    return _globalPlanner.computeTrajectory(start, goal);
}

MoveBaseTrajectory::GlobalTrajectoryComputationResult MoveBaseTrajectory::updateGlobalTrajectory(moveit_msgs::RobotTrajectory & traj)
{
    enum GlobalTrajectoryComputationResult result = GTCR_NO_TRAJECTORY;

    if(!_globalPlanner.isComputing() && !_globalPlanner.foundTrajectory()){
        //_actionServer->setPreempted();
        //_actionServer->setAborted(bonirob_navigation_msgs::MoveBaseGeoPoseResult(), "Trajectory for local planner could not be set.");
        return result;
    }

    // setup planner with new goal
    while(_globalPlanner.isComputing()) {   // TODO true = not preempt, timeout, whatever
        if(_actionServer->isPreemptRequested()){
            ROS_INFO("The planner was asked to stop.");
            _globalPlanner.stopTrajectoryComputation();
            _actionServer->setPreempted();
            return GTCR_PREEMPTED;
        }
        result = updateTrajectoryFromPlanner(traj);
        if(result == GTCR_SUCCESS || result == GTCR_INCONSISTENT){
            break;
        }
        usleep(10*1000);
    }
    
    return updateTrajectoryFromPlanner(traj);
}

MoveBaseTrajectory::GlobalTrajectoryComputationResult MoveBaseTrajectory::updateTrajectoryFromPlanner(moveit_msgs::RobotTrajectory& traj)
{
    enum GlobalTrajectoryComputationResult result = GTCR_NO_TRAJECTORY;
    if(_globalPlanner.foundTrajectory()) {
        moveit_msgs::DisplayTrajectory dtraj;
        if(_globalPlanner.getBestTrajectory(dtraj)) {
            //ROS_INFO("got a trajectory from global planner.");
            _trajectoryPub.publish(dtraj);
            traj = dtraj.trajectory[0];
            result = GTCR_SUCCESS;
        } else {
            ROS_INFO("got inconsistent behavior.");
            result = GTCR_INCONSISTENT;
        }
    }else if(_globalPlanner.foundPrefix()){
        moveit_msgs::DisplayTrajectory dtraj;
        if(_globalPlanner.getBestPrefix(dtraj)) {
            //ROS_INFO("got prefix from global planner");
            traj = dtraj.trajectory[0];
            result = GTCR_SUCCESS;
        }       
    }

    return result;
}

bool MoveBaseTrajectory::updateLocalPlannerTrajectory(const moveit_msgs::RobotTrajectory & traj)
{
    // setup local planner with plan (new!)
    return _localPlanner.setTrajectory(traj);
}

bool MoveBaseTrajectory::executeTrajectory()
{
    // execute plan
    // handle goal reached/failures
    return _localPlanner.executeCurrentTrajectory();
}


bool MoveBaseTrajectory::isSameTrajectory(const moveit_msgs::RobotTrajectory & traj1,
        const moveit_msgs::RobotTrajectory & traj2)
{
    ROS_DEBUG_STREAM(__func__ << " stamps: " << traj1.multi_dof_joint_trajectory.header.stamp << " " << traj2.multi_dof_joint_trajectory.header.stamp
                    << ", number of points: " << traj1.multi_dof_joint_trajectory.points.size()
                    << ", " << traj2.multi_dof_joint_trajectory.points.size());
    return (traj1.multi_dof_joint_trajectory.header.stamp == traj2.multi_dof_joint_trajectory.header.stamp
            && traj1.multi_dof_joint_trajectory.points.size() == traj2.multi_dof_joint_trajectory.points.size());
}

bool MoveBaseTrajectory::isValidTrajectory(const moveit_msgs::RobotTrajectory & traj)
{
    return (!traj.multi_dof_joint_trajectory.points.empty());
}

void MoveBaseTrajectory::clearTrajectory(moveit_msgs::RobotTrajectory & traj)
{
    traj = moveit_msgs::RobotTrajectory();
}

geometry_msgs::PoseStamped MoveBaseTrajectory::geoPoseGoalToPoseStamped(const bonirob_navigation_msgs::MoveBaseGeoPoseGoalConstPtr& goal)
{
    return gps_reference_->getGpsReferencedPose(goal->target_pose);
}

geographic_msgs::GeoPoseStamped MoveBaseTrajectory::getGlobalPose(const geometry_msgs::PoseStamped& goal)
{
    return gps_reference_->getGlobalPose(goal);
}
}

