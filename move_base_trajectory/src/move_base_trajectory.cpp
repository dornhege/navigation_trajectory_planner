#include "move_base_trajectory/move_base_trajectory.h"

#include <ais_3dtools_ros_utility/ros_utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace move_base_trajectory
{

MoveBaseTrajectory::MoveBaseTrajectory(tf2_ros::Buffer & tf) : _tf(tf), _actionServer(NULL), _localPlanner(tf)
{
    ros::NodeHandle nh;
    _actionServer = new MoveBaseActionServer(nh, "move_base",
            boost::bind(&MoveBaseTrajectory::executeCallback, this, _1), false);
    _actionServer->start();
}

MoveBaseTrajectory::~MoveBaseTrajectory()
{
}

void MoveBaseTrajectory::executeCallback(const move_base_msgs::MoveBaseGoalConstPtr & goal)
{
    // check move base aciton is inactive
    // disable planner + local planenr


    // handle final goal/fail

    bool trigger_replan = false;  // TODO move to function for timeouts/etc. or remove if not needed
    moveit_msgs::DisplayTrajectory current_trajectory;
    while(true /* !atGoal or permanant failrure or preempt,etc. */) {
        // if(not plan or trigger_replan) current_trajectory = computeGlobalTrajectory
        if(!isValidTrajectory(current_trajectory) || trigger_replan) {
            clearTrajectory(current_trajectory);
            
            geometry_msgs::TransformStamped planningStartTf = _tf.lookupTransform(_globalPlanner.planningFrame(), _localPlanner.planningFrame(), 
                                                                                     ros::Time(0), ros::Duration(1.5));
  
            geometry_msgs::PoseStamped planningStartPose;
            ais_3dtools_ros_utility::TransformToPose(planningStartTf, planningStartPose);

            //tf2::convert<geometry_msgs::TransformStamped, geometry_msgs::PoseStamped>(planningStartTf, planningStartPose);
            enum GlobalTrajectoryComputationResult res = computeGlobalTrajectory(
                planningStartPose, goal->target_pose, current_trajectory);
            switch(res) {
                case GTCR_SUCCESS:
                    // nothing to do here, we got what we wanted...
                    break;
                case GTCR_NO_TRAJECTORY:
                    // action fail and exit loop
                    break;
                case GTCR_PREEMPTED:
                    // exit loop
                    break;
            }
            trigger_replan = false;
        }

        // improve trajectory
        // if(planner.current_trajectory.ts != plan.ts)
        //  plan = planner.current_trajectory

        ROS_ASSERT(current_trajectory.trajectory.size() > 0);
        if(!isSameTrajectory(current_trajectory.trajectory[0], _localPlanner.currentTrajectory())) {
            updateLocalPlannerTrajectory(current_trajectory.trajectory[0]);
        }
        if(!executeTrajectory()) {
            trigger_replan = true;
        }
    }
}

MoveBaseTrajectory::GlobalTrajectoryComputationResult MoveBaseTrajectory::computeGlobalTrajectory(
    const geometry_msgs::PoseStamped & start, const geometry_msgs::PoseStamped & goal, 
    moveit_msgs::DisplayTrajectory & dtraj)
{
    enum GlobalTrajectoryComputationResult result = GTCR_NO_TRAJECTORY;
    _globalPlanner.computeTrajectory(start, goal);
    // setup planner with new goal
    while(_globalPlanner.isComputing()) {   // true = not preempt, timeout, whatever
        if(_globalPlanner.foundTrajectory()) {
            if(_globalPlanner.getBestTrajectory(dtraj)) {
                result = GTCR_SUCCESS;
                break;
            } else {
                result = GTCR_NO_TRAJECTORY;
                break;
            }
        }
    }
    if(_globalPlanner.foundTrajectory()) {
        if(_globalPlanner.getBestTrajectory(dtraj)) {
            result = GTCR_SUCCESS;
        } else {
            result = GTCR_NO_TRAJECTORY;
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
    return traj1.multi_dof_joint_trajectory.header.stamp == traj2.multi_dof_joint_trajectory.header.stamp;
}

bool MoveBaseTrajectory::isValidTrajectory(const moveit_msgs::DisplayTrajectory & dtraj)
{
    return (!dtraj.trajectory.empty() && !dtraj.trajectory[0].multi_dof_joint_trajectory.points.empty());
}

void MoveBaseTrajectory::clearTrajectory(moveit_msgs::DisplayTrajectory & dtraj)
{
    //dtraj = moveit_msgs::RobotTrajectory();
    dtraj.trajectory.clear();
}

}

