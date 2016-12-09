#include "navigation_trajectory_planner/environment_navxythetalat_generic.h"
#include "freespace_mechanism_heuristic/compute_heuristic.h"
#include <geometry_msgs/PoseArray.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <sbpl/utils/mdp.h>
#include <sbpl/utils/key.h>
#include <sbpl/planners/planner.h>
#include <sbpl/utils/2Dgridsearch.h>
#include <tf/transform_datatypes.h>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include "timing/timing.h"
#include <boost/make_shared.hpp>
#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH

EnvironmentNavXYThetaLatGeneric::EnvironmentNavXYThetaLatGeneric(ros::NodeHandle & nhPriv) : nhPriv_(nhPriv)
{
    timeFreespace = new Timing("freespace_heuristic", true, Timing::SP_STATS, false);
    timeHeuristic = new Timing("heuristic", true, Timing::SP_STATS, false);
}

EnvironmentNavXYThetaLatGeneric::~EnvironmentNavXYThetaLatGeneric()
{
    delete timeFreespace;
    delete timeHeuristic;
}

bool EnvironmentNavXYThetaLatGeneric::InitializeEnv(int width, int height, const unsigned char* mapdata,
        double startx, double starty, double starttheta,
        double goalx, double goaly, double goaltheta,
        double goaltol_x, double goaltol_y, double goaltol_theta,
        const std::vector<sbpl_2Dpt_t>& perimeterptsV, double cellsize_m,
        double nominalvel_mpersecs, double timetoturn45degsinplace_secs,
        unsigned char obsthresh, const char* sMotPrimFile)
{
    std::string freespace_heuristic_costmap_file;
    nhPriv_.getParam("freespace_heuristic_costmap", freespace_heuristic_costmap_file);
	if (! freespace_heuristic_costmap_file.empty())
	{
		useFreespaceHeuristic_ = true;
		bool heuristic_map_valid = true;
		ROS_INFO("Loading freespace costmap from %s and enabling useFreespaceHeuristic.", freespace_heuristic_costmap_file.c_str());
		// see HeuristicCostMap::getOutOfMapBehaviorFromString for values
		std::string freespace_costmap_out_of_map_behavior;
		nhPriv_.param("freespace_costmap_out_of_map_behavior", freespace_costmap_out_of_map_behavior, std::string("euclidean_append"));
		freespace_mechanism_heuristic::HeuristicCostMap::OutOfMapBehavior behavior = freespace_mechanism_heuristic::HeuristicCostMap::getOutOfMapBehaviorFromString(freespace_costmap_out_of_map_behavior);
		freespace_heuristic_costmap = boost::make_shared<freespace_mechanism_heuristic::HeuristicCostMap>(freespace_heuristic_costmap_file, behavior);
		if (freespace_heuristic_costmap->getNumThetaDirs() == 0)
		{
			ROS_ERROR("freespace heuristic costmap failed to load.");
			heuristic_map_valid = false;
		}

		double linear_velocity = freespace_heuristic_costmap->getTransVelCellsPerSec() * cellsize_m;
		if (heuristic_map_valid & fabs(linear_velocity - nominalvel_mpersecs) > 0.0001)
		{
			ROS_ERROR("InitializeEnv: linear velocity mismatch: "
					"freespace_heuristic_costmap: %f -- environment: %f",	linear_velocity, nominalvel_mpersecs);
			heuristic_map_valid = false;
		}
		double angular_velocity = M_PI_4 / timetoturn45degsinplace_secs;
		if (heuristic_map_valid & fabs(freespace_heuristic_costmap->getRotVel() - angular_velocity) > 0.0001)
		{
			ROS_ERROR("InitializeEnv: angular velocity mismatch: "
					"freespace_heuristic_costmap: %f -- environment: %f", freespace_heuristic_costmap->getRotVel(), angular_velocity);
			heuristic_map_valid = false;
		}

		if (! heuristic_map_valid)
		{
			ROS_WARN("freespace heuristic costmap INVALID. Re-computing heuristic map...");
			int err_code = generateHeuristicMap(freespace_heuristic_costmap, sMotPrimFile, cellsize_m, nominalvel_mpersecs, angular_velocity);
			if (err_code != 0)
			{
				ROS_ERROR("freespace heuristic costmap generation failed. Not using freespace heuristic");
				freespace_heuristic_costmap.reset();
				useFreespaceHeuristic_ = false;
			}
			else
			{
				ROS_INFO_STREAM("freespace heuristic costmap generation succeeded. Saving costmap to file "<<freespace_heuristic_costmap_file);
				bool success = freespace_heuristic_costmap->saveCostMap(freespace_heuristic_costmap_file);
				if (! success)
				{
					ROS_WARN_STREAM("map save failed. path: "<<freespace_heuristic_costmap_file);
				}
			}
		}
    }
    return EnvironmentNAVXYTHETALAT::InitializeEnv(width, height, mapdata,
            startx, starty, starttheta, goalx, goaly, goaltheta,
            goaltol_x, goaltol_y, goaltol_theta, perimeterptsV,
            cellsize_m, nominalvel_mpersecs, timetoturn45degsinplace_secs,
            obsthresh, sMotPrimFile);
}

bool EnvironmentNavXYThetaLatGeneric::ReadMotionPrimitives(FILE* fMotPrim)
{
	char buf[1024];
	std::vector<std::string> lines;
	while (fgets(buf,1000, fMotPrim)!=NULL)
	{
		lines.push_back(buf);
	}

	PrimitivesFileIO primitivesReader;
	primitivesReader.readPrimitives(lines, EnvNAVXYTHETALATCfg);

	return true;
}

bool EnvironmentNavXYThetaLatGeneric::useFreespaceHeuristic(bool on)
{
    useFreespaceHeuristic_ = on;
    if(useFreespaceHeuristic_ && freespace_heuristic_costmap == NULL) {
        ROS_ERROR("useFreespaceHeuristic requested on, but no freespace_heuristic_costmap loaded.");
        useFreespaceHeuristic_ = false;
    }
}

void EnvironmentNavXYThetaLatGeneric::updateForPlanRequest()
{
    nhPriv_.getParam("use_freespace_heuristic", useFreespaceHeuristic_);
    useFreespaceHeuristic(useFreespaceHeuristic_);

    if(freespace_heuristic_costmap) {
        std::string freespace_costmap_out_of_map_behavior;
        if(nhPriv_.getParam("freespace_costmap_out_of_map_behavior", freespace_costmap_out_of_map_behavior)) {
            ROS_INFO("Switching freespace_costmap_out_of_map_behavior to %s",
                    freespace_costmap_out_of_map_behavior.c_str());
            freespace_heuristic_costmap->setOutOfMapBehavior(
                    freespace_mechanism_heuristic::HeuristicCostMap::getOutOfMapBehaviorFromString(
                        freespace_costmap_out_of_map_behavior));
        }
    }
}

int EnvironmentNavXYThetaLatGeneric::GetFromToHeuristic(int FromStateID, int ToStateID)
{
    timeHeuristic->start();
    ScopeExit se(boost::bind(&Timing::end, timeHeuristic));

    int heur = EnvironmentNAVXYTHETALAT::GetFromToHeuristic(FromStateID, ToStateID);

    int fromX, fromY, fromTheta;
    int toX, toY, toTheta;
    GetCoordFromState(FromStateID, fromX, fromY, fromTheta);
    GetCoordFromState(ToStateID, toX, toY, toTheta);

    int dx = toX - fromX;
    int dy = toY - fromY;

    int hfs = getFreespaceCost(dx, dy, fromTheta, toTheta);
    if(hfs > heur)
        return hfs;
    return heur;
}

int EnvironmentNavXYThetaLatGeneric::GetStartHeuristic(int stateID)
{
    timeHeuristic->start();
    ScopeExit se(boost::bind(&Timing::end, timeHeuristic));

    int heur = EnvironmentNAVXYTHETALAT::GetStartHeuristic(stateID);

    int x, y, theta;
    GetCoordFromState(stateID, x, y, theta);
    int dx = EnvNAVXYTHETALATCfg.StartX_c - x;
    int dy = EnvNAVXYTHETALATCfg.StartY_c - y;
    dx = -dx;   // FIXME this is supposed to be FROM start TO stateID, not vice versa
    dy = -dy;
    int endTh = theta;

    int hfs = getFreespaceCost(dx, dy, EnvNAVXYTHETALATCfg.StartTheta, endTh);
    if(hfs > heur)
        return hfs;
    return heur;
}

int EnvironmentNavXYThetaLatGeneric::GetGoalHeuristic(int stateID)
{
    timeHeuristic->start();
    ScopeExit se(boost::bind(&Timing::end, timeHeuristic));

#if USE_HEUR==0
    return 0;
#endif

#if DEBUG
    if (stateID >= (int)StateID2CoordTable.size()) {
        SBPL_ERROR("ERROR in EnvironmentNavXYThetaLatGeneric GetGoalHeuristic function: stateID illegal\n");
        throw new SBPL_Exception();
    }
#endif

    int x, y, theta;
    GetCoordFromState(stateID, x, y, theta);
    //computes distances from start state that is grid2D, so it is EndX_c EndY_c
    int h2D = grid2Dsearchfromgoal->getlowerboundoncostfromstart_inmm(x, y);
    int hEuclid = (int)(NAVXYTHETALAT_COSTMULT_MTOMM * EuclideanDistance_m(x, y,
                EnvNAVXYTHETALATCfg.EndX_c,
                EnvNAVXYTHETALATCfg.EndY_c));
    //define this function if it is used in the planner (heuristic backward search would use it)
    int heur = (int)(((double)__max(h2D, hEuclid)) / EnvNAVXYTHETALATCfg.nominalvel_mpersecs);

    int dx = EnvNAVXYTHETALATCfg.EndX_c - x;
    int dy = EnvNAVXYTHETALATCfg.EndY_c - y;
    int startTh = theta;

    int hfs = getFreespaceCost(dx, dy, startTh, EnvNAVXYTHETALATCfg.EndTheta);
    if(hfs > heur)
        return hfs;
    return heur;
}

int EnvironmentNavXYThetaLatGeneric::getFreespaceCost(int deltaX, int deltaY, int theta_start, int theta_end)
{
    timeFreespace->start();
    ScopeExit se(boost::bind(&Timing::end, timeFreespace));

    if(useFreespaceHeuristic_ && freespace_heuristic_costmap) {
        return freespace_heuristic_costmap->getCost(deltaX, deltaY, theta_start, theta_end);
    }

    return 0;
}

void EnvironmentNavXYThetaLatGeneric::resetTimingStats()
{
    timeFreespace->getStats().reset();
    timeHeuristic->getStats().reset();
}

void EnvironmentNavXYThetaLatGeneric::printTimingStats()
{
    timeFreespace->printStats(true);
    timeHeuristic->printStats(true);
}

moveit_msgs::DisplayTrajectory EnvironmentNavXYThetaLatGeneric::stateIDPathToDisplayTrajectory(const std::vector<int> & stateIDPath)
{
    moveit_msgs::DisplayTrajectory dtraj;
    dtraj.model_id = "robot";

    std::vector<sbpl_xy_theta_pt_t> path;

    if(stateIDPath.empty()) {
        // make move_base happy
        geometry_msgs::Pose start = poseFromStateID(EnvNAVXYTHETALAT.startstateid);
        double theta_start = tf::getYaw(start.orientation);
        sbpl_xy_theta_pt_t s(start.position.x, start.position.y, theta_start);
        path.push_back(s);
    } else {
        try {
            ConvertStateIDPathintoXYThetaPath(&stateIDPath, &path);
        } catch (SBPL_Exception& e) {
            ROS_ERROR("SBPL encountered a fatal exception while reconstructing the path");
            return dtraj;
        }
    }

    moveit::core::robotStateToRobotStateMsg(getPlanningScene()->getCurrentState(), dtraj.trajectory_start);
    moveit_msgs::RobotTrajectory traj;
    traj.multi_dof_joint_trajectory.header.frame_id = getPlanningFrame();
    traj.multi_dof_joint_trajectory.joint_names.push_back("world_joint");
    tf::Transform lastPose;
    for(unsigned int i = 0; i < path.size(); ++i) {
        trajectory_msgs::MultiDOFJointTrajectoryPoint pt;
        geometry_msgs::Transform tf;
        tf.translation.x = path[i].x;
        tf.translation.y = path[i].y;
        tf.translation.z = 0;
        tf.rotation = tf::createQuaternionMsgFromYaw(path[i].theta);

        if(i > 0) {
            tf::Transform curPose;
            tf::transformMsgToTF(tf, curPose);
            tf::Transform delta = lastPose.inverseTimes(curPose);
            if(hypot(delta.getOrigin().x(), delta.getOrigin().y()) < 0.05 &&
                    fabs(tf::getYaw(delta.getRotation())) < 0.2)
                continue;
        }

        pt.transforms.push_back(tf);
        tf::transformMsgToTF(tf, lastPose);

        traj.multi_dof_joint_trajectory.points.push_back(pt);
    }

    dtraj.trajectory.push_back(traj);
    return dtraj;
}

