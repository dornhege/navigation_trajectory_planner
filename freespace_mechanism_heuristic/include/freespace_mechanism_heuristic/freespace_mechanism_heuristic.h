#ifndef FREESPACE_MECHANISM_HEURISTIC_H
#define FREESPACE_MECHANISM_HEURISTIC_H

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <sbpl/discrete_space_information/environment_navxythetalat.h>

namespace freespace_mechanism_heuristic
{

class HeuristicCostMap;
typedef boost::shared_ptr<HeuristicCostMap> HeuristicCostMapPtr;
class HeuristicCostMap
{
    friend void computeCosts(const EnvNAVXYTHETALATConfig_t & cfg, int theta, HeuristicCostMapPtr & costmaps);

    public:
        /// How are queries outside the costmap answered
        enum OutOfMapBehavior
        {
            OutOfMapMaxCost,        ///< use max value of map
            OutOfMapInfiniteCost,   ///< use INFINITECOST
            OutOfMapAssert,         ///< Assert, i.e., should not happen
            OutOfMapZero,           ///< if out of map, return 0
            // The following options minimize over theta, but not over all cells in the grid (for performance).
            // Thus these are not admissable any more.
            OutOfMapExpandEuclideanPrepend,    ///< compute the out of map part by prepending euclidean distance query
            OutOfMapExpandEuclideanAppend,     ///< compute the out of map part by appending euclidean distance query
            // Warning: with 16 theta dirs, performance is in 16^depth, where depth is in the order of
            // request_size/(heuristic_costmap_size/2)
            OutOfMapRecursiveQuery, ///< Split the request in multiple recursive requests along the query
        };

        /**
         * Construct empty HeuristicCostMap.
         *
         * \param [in] transVel translational velocity in cells/s
         * \param [in] rotVel rotational velocity in rad/s
         */
        HeuristicCostMap(unsigned int height, unsigned int width, unsigned int numThetaDirs, 
                double transVel, double rotVel,
                bool allocateMaps, enum OutOfMapBehavior outOfMapBehaviour);

        HeuristicCostMap(const std::string & mapfile, enum OutOfMapBehavior outOfMapBehaviour);
        ~HeuristicCostMap();

        void setOutOfMapBehavior(enum OutOfMapBehavior oomb) { outOfMapBehaviour_ = oomb; }

        /// Get the OutOfMapBehavior setting from a string.
        /**
         * Values are: max_cost, infinite_cost, assert, zero, euclidean_prepend, euclidean_append, recursive_query.
         */
        static enum OutOfMapBehavior getOutOfMapBehaviorFromString(const std::string & behavior_name);

        unsigned int getCost(int dx, int dy, int startTheta, int endTheta) const;

        bool loadCostMap(const std::string & mapfile);
        bool saveCostMap(const std::string & mapfile) const;

        /// Save costmap as PPM.
        /**
         * \param [in] endTheta end pose theta for costmap image. If < 0, chooses min cost over all end thetas
         * \param [in] expansion expand the costmap image to write and query beyond the borders
         */
        bool saveCostMapImage(const std::string & ppmfile, int startTheta, int endTheta, int expansion = 0) const;

        void updateMaxCost();

        unsigned int getHeight() const { return height_; }
        unsigned int getWidth() const { return width_; }
        unsigned int getNumThetaDirs() const { return numThetaDirs_; }
        double getTransVelCellsPerSec() const { return tv_; }
        double getRotVel() const { return rv_; }

        /// Compute the intersecting border coordinate for a (dx, dy) query that goes outside of the map
        void computeBorderCell(int dx, int dy, int & border_x, int & border_y) const;

        /// Compute the euclidean cost for a query.
        /**
         * Computed as the cost for a SBPL motion primitve as 1000 * max time for linear or angular movement.
         * Linear movement between dx and dy, and angular movement between startTheta and endTheta are
         * considered independent.
         */
        unsigned int getEuclideanCost(int dx, int dy, int startTheta, int endTheta) const;

    protected:
        void allocateMaps();
        void deallocateMaps();

        /// Raw access for writing
        unsigned int*** getCostMap(unsigned int theta) { return costmaps_[theta]; }

        unsigned int getCostRecurs(int dx, int dy, int startTheta, int endTheta, int depth) const;

    protected:
        int height_;
        int width_;
        unsigned int numThetaDirs_;
        double tv_;     ///< tv in cells/s
        double rv_;     ///< rv in rad/s

        enum OutOfMapBehavior outOfMapBehaviour_;

        /// One costmap for each theta in numThetaDirs.
        /**
         * Each costmap is array of size height_ x width_ x numThetaDirs for
         * the cost of deltax, deltay, endtheta, where
         * deltax, deltay = 0, 0 is assumed to be at height_/2, width_/2.
         *
         * Each costmap contains the costs to get from (0, 0, start theta) -> each cells (dx, dy, end theta).
         */
        std::vector<unsigned int ***> costmaps_;

        unsigned int maxCost_;
};

}

#endif

