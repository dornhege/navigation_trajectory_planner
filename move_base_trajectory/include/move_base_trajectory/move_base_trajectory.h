#ifndef MOVE_BASE_TRAJECTORY_H
#define MOVE_BASE_TRAJECTORY_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_trajectory/base_local_planner.h>
#include <move_base_trajectory/base_global_planner.h>
#include "move_base_trajectory/local_planner.h"
#include "move_base_trajectory/global_planner_thread.h"
#include <moveit_msgs/DisplayTrajectory.h>

namespace move_base_trajectory
{

typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> MoveBaseActionServer;

class MoveBaseTrajectory
{
    public:
        MoveBaseTrajectory(tf2_ros::Buffer & tf);
        ~MoveBaseTrajectory();

        /// Handles MoveBase action requests
        void executeCallback(const move_base_msgs::MoveBaseGoalConstPtr & goal);

    protected:
        enum GlobalTrajectoryComputationResult {
            GTCR_SUCCESS,
            GTCR_NO_TRAJECTORY,
            GTCR_INCONSISTENT,
            GTCR_PREEMPTED,
        };

    protected:
        /// Start computation of a global trajectory using the global planner thread
        bool startTrajectoryComputation(const geometry_msgs::PoseStamped & start, const geometry_msgs::PoseStamped & goal);

        /**
         * \param [out] traj the computed trajectory (if GTCR_SUCCESS is returned).
         * \returns a GlobalTrajectoryComputationResult. If GTCR_PREEMPTED,
         * the action server has already been preempted. Otherwise control
         * should continue depending on the result.
         */
        GlobalTrajectoryComputationResult updateGlobalTrajectory(moveit_msgs::RobotTrajectory & traj);


        MoveBaseTrajectory::GlobalTrajectoryComputationResult updateTrajectoryFromPlanner(moveit_msgs::RobotTrajectory& traj);

        /// Update the local planner with a new trajectory.
        /**
         *  \returns true, if the trajectory can be executed.
         */
        bool updateLocalPlannerTrajectory(const moveit_msgs::RobotTrajectory & traj);

        /// Execute the current trajectory by the local planner.
        /**
         *  \returns true, if a command to execute could be computed.
         */
        bool executeTrajectory();

        /// \returns true, if traj1 and traj2 are considered the same.
        /**
         *  Currently based on the time stamps, assuming these are unique among different trajectories.
         */
        bool isSameTrajectory(const moveit_msgs::RobotTrajectory & traj1,
                const moveit_msgs::RobotTrajectory & traj2);

        /// \returns true, if traj is a valid trajectory (with content) that could be executed.
        bool isValidTrajectory(const moveit_msgs::RobotTrajectory & dtraj);

        /// Empties a trajectory.
        void clearTrajectory(moveit_msgs::RobotTrajectory & dtraj);

    protected:
        std::string _baseFrame;
        ros::Publisher _trajectoryPub;

        tf2_ros::Buffer & _tf;
        MoveBaseActionServer* _actionServer;

        GlobalPlannerThread _globalPlanner;
        LocalPlanner _localPlanner;
};

}

#endif

