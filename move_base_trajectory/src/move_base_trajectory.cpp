#include "move_base_trajectory/move_base_trajectory.h"

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
    moveit_msgs::RobotTrajectory current_trajectory;
    while(true /* !atGoal or permanant failrure or preempt,etc. */) {
        // if(not plan or trigger_replan) current_trajectory = computeGlobalTrajectory
        if(!isValidTrajectory(current_trajectory) || trigger_replan) {
            clearTrajectory(current_trajectory);
            enum GlobalTrajectoryComputationResult res = computeGlobalTrajectory(
                    goal->target_pose, current_trajectory);
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

        if(!isSameTrajectory(current_trajectory, _localPlanner.currentTrajectory())) {
            updateLocalPlannerTrajectory(current_trajectory);
        }
        if(!executeTrajectory()) {
            trigger_replan = true;
        }
    }
}

MoveBaseTrajectory::GlobalTrajectoryComputationResult MoveBaseTrajectory::computeGlobalTrajectory(
        const geometry_msgs::PoseStamped & goal, moveit_msgs::RobotTrajectory & traj)
{
    enum GlobalTrajectoryComputationResult result = GTCR_NO_TRAJECTORY;
    _globalPlanner.computeTrajectory(goal);
    // setup planner with new goal
    while(_globalPlanner.isComputing()) {   // true = not preempt, timeout, whatever
        if(_globalPlanner.foundTrajectory()) {
            if(_globalPlanner.getBestTrajectory(traj)) {
                result = GTCR_SUCCESS;
                break;
            } else {
                result = GTCR_NO_TRAJECTORY;
                break;
            }
        }
    }
    if(_globalPlanner.foundTrajectory()) {
        if(_globalPlanner.getBestTrajectory(traj)) {
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

bool MoveBaseTrajectory::isValidTrajectory(const moveit_msgs::RobotTrajectory & traj)
{
    return !traj.multi_dof_joint_trajectory.points.empty();
}

void MoveBaseTrajectory::clearTrajectory(moveit_msgs::RobotTrajectory & traj)
{
    traj = moveit_msgs::RobotTrajectory();
}

}

