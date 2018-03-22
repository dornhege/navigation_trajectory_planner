#ifndef GLOBAL_PLANNER_THREAD_H
#define GLOBAL_PLANNER_THREAD_H

#include <move_base_trajectory/base_global_planner.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <pluginlib/class_loader.h>

#include <string>

namespace move_base_trajectory
{

class GlobalPlannerThread
{
    public:
        GlobalPlannerThread();
        ~GlobalPlannerThread();

        /// Sets traj to the best trajectory.
        /**
         * \returns true, if a best trajectory exists.
         */
        bool getBestTrajectory(moveit_msgs::DisplayTrajectory & dtraj) const;

        bool getBestPrefix(moveit_msgs::DisplayTrajectory & dtraj) const;

        /// \returns true, if the trajectory computation is still running.
        bool isComputing() const;

        /// Start computation of a trajectory to a new goal.
        /**
         * The function does not block, but starts the computation in the background.
         *
         * \returns true, if the goal is valid and a computation could be started.
         */
        bool computeTrajectory(const geometry_msgs::PoseStamped & start, const geometry_msgs::PoseStamped & goal);

        /// \returns true, if there is a trajectory found by the planner.
        bool foundTrajectory() const;

        bool foundPrefix() const;

        /// Blocks until the current computation is halted (or none is running).
        void stopTrajectoryComputation();

        std::string planningFrame() const;

    protected:
        bool loadGlobalPlanner(const std::string & globalPlannerName);

    protected: 
        boost::shared_ptr<move_base_trajectory::BaseGlobalPlannerTrajectory> _globalPlanner;
        pluginlib::ClassLoader<move_base_trajectory::BaseGlobalPlannerTrajectory> _globalPlannerLoader;
        mutable boost::thread _plannerThread;

};

}

#endif

