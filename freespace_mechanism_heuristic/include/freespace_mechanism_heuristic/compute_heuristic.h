/*
 * compute_heuristic.h
 *
 *  Created on: Jul 27, 2016
 *      Author: andreas
 */

#ifndef INCLUDE_FREESPACE_MECHANISM_HEURISTIC_COMPUTE_HEURISTIC_H_
#define INCLUDE_FREESPACE_MECHANISM_HEURISTIC_COMPUTE_HEURISTIC_H_

#include <sbpl/discrete_space_information/environment_navxythetalat.h>
#include <sbpl/utils/key.h>
#include <navigation_trajectory_utils/primitives_file_io.h>
#include "freespace_mechanism_heuristic/freespace_mechanism_heuristic.h"

using namespace freespace_mechanism_heuristic;

bool g_WriteImages = false;

// This is basically just to get to the motion primitives functionalities
class EnvironmentMotionPrims : public EnvironmentNAVXYTHETALATTICE
{
    public:

    EnvironmentMotionPrims(int width, int height,
            double nominalvel_mpersecs, double timetoturn45degsinplace_secs) {
        EnvNAVXYTHETALATCfg.EnvWidth_c = width;
        EnvNAVXYTHETALATCfg.EnvHeight_c = height;
        EnvNAVXYTHETALATCfg.nominalvel_mpersecs = nominalvel_mpersecs;
        EnvNAVXYTHETALATCfg.timetoturn45degsinplace_secs = timetoturn45degsinplace_secs;
    }

    const EnvNAVXYTHETALATConfig_t getConfig() const {
        return EnvNAVXYTHETALATCfg;
    }

    bool readPrimitives(const std::string & fname, double resolution);

    private:
    virtual int SizeofCreatedEnv(){ return -1;}
    virtual void PrintState(int, bool, FILE*) { }
    virtual int GetFromToHeuristic(int FromStateID, int ToStateID) { return -1;}
    virtual int GetGoalHeuristic(int stateID) { return -1;}
    virtual int GetStartHeuristic(int stateID) { return -1;}
    virtual void SetAllActionsandAllOutcomes(CMDPSTATE* state) { }
    virtual void GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV) { }
    virtual void GetPredsofChangedEdges(std::vector<nav2dcell_t> const * changedcellsV, std::vector<int> *preds_of_changededgesIDV) { }
    virtual void GetSuccsofChangedEdges(std::vector<nav2dcell_t> const * changedcellsV, std::vector<int> *succs_of_changededgesIDV) { }
    virtual void InitializeEnvironment() { }
    virtual void GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<EnvNAVXYTHETALATAction_t*>* actionindV ) { }
    virtual void GetLazySuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<bool>* isTrueCost, std::vector<EnvNAVXYTHETALATAction_t*>* actionindV ) { }
    virtual void GetSuccsWithUniqueIds(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<EnvNAVXYTHETALATAction_t*>* actionindV ) { }
    virtual void GetLazySuccsWithUniqueIds(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<bool>* isTrueCost, std::vector<EnvNAVXYTHETALATAction_t*>* actionindV ) { }
    virtual int GetTrueCost(int parentID, int childID) { return -1;}
    virtual bool isGoal(int id) { return false;}
};

struct SearchNode {
    int x;
    int y;
    int theta;
    int g;

    bool operator<(const SearchNode & rhs) const {
        return g > rhs.g;
    }

    SearchNode(int x, int y, int theta, int g) : x(x), y(y), theta(theta), g(g) {}
};

namespace freespace_mechanism_heuristic
{

void computeCosts(const EnvNAVXYTHETALATConfig_t & cfg, int theta, HeuristicCostMapPtr & costmaps);

/** Generate a new Heuristic cost map of given size.
 *  \param [out] costmap The resulting costmap.
 *  \param [in] mprims_path Path to the motion primitive file.
 *  \param [in] linear_velocity The nominal linear velocity of the robot in meter per second.
 *  \param [in] angular_velocity The nominal angular velocity of the robot in radians per second.
 *  \param [in] size_x Width of the generated map in cells.
 *  \param [in] size_y Height of the generated map in cells.
 *  \return 0 if successful, 1 if failed to read motion primitives
 */
int generateHeuristicMap(
		HeuristicCostMapPtr& costmap,
		const std::string& mprims_path,
		double resolution,
		double linear_velocity = 0.8,
		double angular_velocity = 1.308997,
		int size_x = 250,
		int size_y = 250);

}

#endif /* INCLUDE_FREESPACE_MECHANISM_HEURISTIC_COMPUTE_HEURISTIC_H_ */
