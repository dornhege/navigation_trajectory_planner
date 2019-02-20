#ifndef NAVIGATION_TRAJECTORY_PLANNER_H
#define NAVIGATION_TRAJECTORY_PLANNER_H

#include "navigation_trajectory_msgs/SampleValidPoses.h"
#include "navigation_trajectory_planner/environment_navxythetalat_generic.h"
#include "move_base_trajectory/base_global_planner.h"

#include <sbpl/headers.h>
#include <nav_core/base_global_planner.h>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <iostream>
#include <vector>

namespace navigation_trajectory_planner
{

class XYThetaStateChangeQuery : public StateChangeQuery
{
public:
    XYThetaStateChangeQuery(EnvironmentNavXYThetaLatGeneric* env, const std::vector<nav2dcell_t> & changedcells);

    virtual const std::vector<int> * getPredecessors() const;
    virtual const std::vector<int> * getSuccessors() const;

public:
    EnvironmentNavXYThetaLatGeneric* env_;
    std::vector<nav2dcell_t> changedcells_;
    mutable std::vector<int> predsOfChangedCells_;
    mutable std::vector<int> succsOfChangedCells_;
};

class NavigationTrajectoryPlanner : public move_base_trajectory::BaseGlobalPlannerTrajectory
{
public:
    NavigationTrajectoryPlanner();
    virtual ~NavigationTrajectoryPlanner() {
        delete private_nh_;
    }

    virtual void initialize(std::string name);

    /// Main query from move_base
    virtual bool makeTrajectory(const geometry_msgs::PoseStamped& startPose,
                                const geometry_msgs::PoseStamped& goalPose, 
                                moveit_msgs::DisplayTrajectory & dtraj);

    /// Returns the frame that all planning is happening in.
    /**
     * This is usually the planning frame of the environment. All data has to be either in this
     * frame or has to be transformed to this.
     */
    virtual std::string planningFrame() const;

    /// Read in parameters that can be re-set for each makePlan call.
    virtual void readDynamicParameters();

    virtual bool foundTrajectory() const;
//    virtual bool foundPrefix() const;

    virtual bool getCurrentBestTrajectory(moveit_msgs::DisplayTrajectory & dtraj) const;

    bool transformPoseToPlanningFrame(geometry_msgs::PoseStamped & poseMsg);
//    virtual bool getCurrentBestPrefix(moveit_msgs::DisplayTrajectory & dtraj) const;
protected:
    //virtual bool sampleValidPoses(navigation_trajectory_msgs::SampleValidPoses::Request & req, navigation_trajectory_msgs::SampleValidPoses::Response & resp);

    /// Make sure that a ready to use environment exists.
    virtual bool createAndInitializeEnvironment();

    /// Make sure there is a planner instance.
    virtual bool createPlanner();

    /// Create a custom environment for this planner.
    virtual EnvironmentNavXYThetaLatGeneric* createEnvironment(ros::NodeHandle & nhPriv) = 0;
    virtual bool initializeEnvironment(
            double trans_vel, double timeToTurn45Degs, const std::string & motion_primitive_filename) = 0;

    /// Update internal representation of the planner for a plan request.
    virtual bool updateForPlanRequest(geometry_msgs::PoseStamped& start, geometry_msgs::PoseStamped& goal);

    virtual void publishStats(int solution_cost, int solution_size, const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal);
    virtual void publish_expansions();
    virtual void publish_expansion_map();

    nav_msgs::Path trajectoryToGuiPath(const moveit_msgs::RobotTrajectory & traj) const;

    virtual void fillGrid(nav_msgs::OccupancyGrid & grid, const std::vector< std::set<int> > & gridDirections, int maxDirections);
    void rememberDisplayTrajectoryFromStateIdPath(const std::vector<int> & path, const double cost);
//    void handleNewExpandedStatePath(const std::vector<int> & path);

    virtual bool isComputing() const;

protected:
    void setIsComputing(bool isComputing);

    bool initialized_;
    ros::NodeHandle* private_nh_;

    ARAPlanner* planner_;
    EnvironmentNavXYThetaLatGeneric* env_;

    bool isComputing_;
    double allocated_time_;
    double initial_epsilon_;
    int force_scratch_limit_;   ///< if the number of changed cells is >= this, planning from scratch will happen
/*    double prefix_dist_;
    int min_prefix_entries_;
    double used_prefix_portion_;
    bool found_prefix_;*/

    moveit_msgs::DisplayTrajectory current_best_trajectory_;
//    moveit_msgs::DisplayTrajectory current_best_prefix_;
    double current_best_cost_;
    mutable boost::mutex trajectory_mutex_; // locks current_best_trajectory_ and current_best_cost_
    mutable boost::mutex is_computing_mutex_;
//    mutable boost::mutex prefix_mutex_; // locks current_best_prefix_

    ros::Publisher plan_pub_;
    ros::Publisher traj_pub_;
    ros::Publisher stats_publisher_;
    ros::Publisher expansions_publisher_;

    std::string expansion_color_scheme_;    ///< occupancy or costmap
    ros::Publisher pub_expansion_map_;
    ros::Publisher pub_generation_map_;
    ros::Publisher pub_expansion_first_map_;
    ros::Publisher pub_generation_first_map_;
//    ros::Publisher pub_expansion_prefix_;
//    ros::Publisher pub_chosen_prefix_;

//    ros::ServiceServer srv_sample_poses_;
};

}

#endif

