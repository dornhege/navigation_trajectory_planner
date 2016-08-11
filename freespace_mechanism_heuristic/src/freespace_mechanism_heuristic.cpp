#include "freespace_mechanism_heuristic/freespace_mechanism_heuristic.h"
#include <sbpl/utils/key.h>
#include <sbpl/utils/utils.h>
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <fstream>
#include <ros/ros.h>

namespace freespace_mechanism_heuristic
{

HeuristicCostMap::HeuristicCostMap(unsigned int height, unsigned int width, unsigned int numThetaDirs,
        double transVel, double rotVel,
        bool allocate, enum OutOfMapBehavior outOfMapBehaviour) :
    height_(height), width_(width), numThetaDirs_(numThetaDirs), tv_(transVel), rv_(rotVel),
    outOfMapBehaviour_(outOfMapBehaviour)
{
    if(allocate) {
        allocateMaps(); 
    }
}

HeuristicCostMap::HeuristicCostMap(const std::string & mapfile, enum OutOfMapBehavior outOfMapBehaviour) :
    outOfMapBehaviour_(outOfMapBehaviour)
{
    if(!loadCostMap(mapfile)) {
        fprintf(stderr, "Loading map from \"%s\" failed.", mapfile.c_str());
        width_ = height_ = numThetaDirs_ = 0;
    }
}

HeuristicCostMap::~HeuristicCostMap()
{
    deallocateMaps();
}

enum HeuristicCostMap::OutOfMapBehavior HeuristicCostMap::getOutOfMapBehaviorFromString(
        const std::string & behavior_name)
{
    enum OutOfMapBehavior out_of_map_behavior;
    if(behavior_name == "max_cost") {
        out_of_map_behavior = HeuristicCostMap::OutOfMapMaxCost;
    } else if(behavior_name == "infinite_cost") {
        out_of_map_behavior = HeuristicCostMap::OutOfMapInfiniteCost;
    } else if(behavior_name == "assert") {
        out_of_map_behavior = HeuristicCostMap::OutOfMapAssert;
    } else if(behavior_name == "zero") {
        out_of_map_behavior = HeuristicCostMap::OutOfMapZero;
    } else if(behavior_name == "euclidean_prepend") {
        out_of_map_behavior = HeuristicCostMap::OutOfMapExpandEuclideanPrepend;
    } else if(behavior_name == "euclidean_append") {
        out_of_map_behavior = HeuristicCostMap::OutOfMapExpandEuclideanAppend;
    } else if(behavior_name == "recursive_query") {
        out_of_map_behavior = HeuristicCostMap::OutOfMapRecursiveQuery;
    } else {
        ROS_ERROR("Unknown OutOfMapBehavior type: %s - using euclidean_append",
                behavior_name.c_str());
        out_of_map_behavior = HeuristicCostMap::OutOfMapExpandEuclideanAppend;
    }
    return out_of_map_behavior;
}

void HeuristicCostMap::allocateMaps()
{
    assert(costmaps_.empty());
    for(unsigned int i = 0; i < numThetaDirs_; ++i) {
        unsigned int*** cost_map = new unsigned int**[width_];
        for(unsigned int x = 0; x < width_; ++x) {
            cost_map[x] = new unsigned int*[height_];
            for(unsigned int y = 0; y < height_; ++y) {
                cost_map[x][y] = new unsigned int[numThetaDirs_];
                for(unsigned int th = 0; th < numThetaDirs_; ++th) {
                    cost_map[x][y][th] = INFINITECOST;
                }
            }
        }
        costmaps_.push_back(cost_map);
    }
}

void HeuristicCostMap::deallocateMaps()
{
    for(unsigned int i = 0; i < costmaps_.size(); ++i) {
        unsigned int*** cost_map = costmaps_.at(i);
        if(cost_map) {
            for(unsigned int x = 0; x < width_; ++x) {
                for(unsigned int y = 0; y < height_; ++y) {
                    delete [] cost_map[x][y];
                }
                delete [] cost_map[x];
            }
            delete [] cost_map;
        }
    }
    costmaps_.clear();
}


bool HeuristicCostMap::loadCostMap(const std::string & mapfile)
{
    std::ifstream f(mapfile.c_str(), std::ios_base::binary);
    if(!f.good()) {
        return false;
    }

    // header infos: sx, sy, num thetas, tv, rv
    f.read(reinterpret_cast<char*>(&width_), sizeof(int));
    f.read(reinterpret_cast<char*>(&height_), sizeof(int));
    f.read(reinterpret_cast<char*>(&numThetaDirs_), sizeof(int));
    f.read(reinterpret_cast<char*>(&tv_), sizeof(double));
    f.read(reinterpret_cast<char*>(&rv_), sizeof(double));
    // delete/allocate
    deallocateMaps();
    allocateMaps();
    // Then one costmap per start theta
    assert(costmaps_.size() == numThetaDirs_);
    for(unsigned int i = 0; i < costmaps_.size(); ++i) {
        // costmap is costs as int in x, y, endtheta
        for(unsigned int x = 0; x < width_; ++x) {
            for(unsigned int y = 0; y < height_; ++y) {
                for(unsigned int th = 0; th < numThetaDirs_; ++th) {
                    int cost;
                    f.read(reinterpret_cast<char*>(&cost), sizeof(int));
                    costmaps_[i][x][y][th] = cost;
                }
            }
        }
    }
    f.close();

    updateMaxCost();
    return true;
}

bool HeuristicCostMap::saveCostMap(const std::string & mapfile) const
{
    std::ofstream f(mapfile.c_str(), std::ios_base::binary);
    if(!f.good()) {
        return false;
    }

    // header infos: sx, sy, num thetas, tv, rv
    f.write(reinterpret_cast<const char*>(&width_), sizeof(int));
    f.write(reinterpret_cast<const char*>(&height_), sizeof(int));
    f.write(reinterpret_cast<const char*>(&numThetaDirs_), sizeof(int));
    f.write(reinterpret_cast<const char*>(&tv_), sizeof(double));
    f.write(reinterpret_cast<const char*>(&rv_), sizeof(double));
    // Then one costmap per start theta
    assert(costmaps_.size() == numThetaDirs_);
    for(unsigned int i = 0; i < costmaps_.size(); ++i) {
        // costmap is costs as int in x, y, endtheta
        for(unsigned int x = 0; x < width_; ++x) {
            for(unsigned int y = 0; y < height_; ++y) {
                for(unsigned int th = 0; th < numThetaDirs_; ++th) {
                    int cost = costmaps_[i][x][y][th];
                    f.write(reinterpret_cast<const char*>(&cost), sizeof(int));
                }
            }
        }
    }
    f.close();
    return true;
}

bool HeuristicCostMap::saveCostMapImage(const std::string & ppmfile, int startTheta, int endTheta, int expansion) const
{
    std::ofstream f(ppmfile.c_str());
    if(!f.good())
        return false;

    f << "P3 " << width_ + 2*expansion << " " << height_ + 2 * expansion << " 255" << std::endl;

    // image save, i.e.: iterate lines by height and start with "top line", i.e. the last/max index
    int maxCostExtra = 0;
    if(expansion > 0) {
        maxCostExtra = getEuclideanCost(expansion, expansion, 0, numThetaDirs_/2);
    }

    for(int dy = -height_/2 + height_ - 1 + expansion; dy >= -height_/2 - expansion; --dy) {
        for(int dx = -width_/2 - expansion; dx < -width_/2 + width_ + expansion; ++dx) {
            unsigned int cost = 255;
            if(endTheta < 0) {
                cost = getCost(dx, dy, startTheta, 0);
                for(unsigned int th = 1; th < numThetaDirs_; ++th) {
                    if(getCost(dx, dy, startTheta, th) < cost)
                        cost = getCost(dx, dy, startTheta, th);
                }
            } else {
                cost = getCost(dx, dy, startTheta, endTheta);
            }
            if(cost == INFINITECOST) {
                f << "255 0 255 ";
                continue;
            }
            int c = int(double(cost)/(maxCost_ + maxCostExtra) * 240.0);
            //printf("Writing cost: %d as %d for th %d (max %d)\n", cost, c, theta, g_maxCost);
            f << c << " " << c << " " << c << " ";
        }
        f << std::endl;
    }

    f.close();
    return true;
}

void HeuristicCostMap::updateMaxCost()
{
    maxCost_ = 0; 
    for(unsigned int i = 0; i < costmaps_.size(); ++i) {
        for(unsigned int x = 0; x < width_; ++x) {
            for(unsigned int y = 0; y < height_; ++y) {
                for(unsigned int th = 0; th < numThetaDirs_; ++th) {
                    if(costmaps_[i][x][y][th] == INFINITECOST)
                        continue;
                    if(costmaps_[i][x][y][th] > maxCost_)
                        maxCost_ = costmaps_[i][x][y][th];
                }
            }
        }
    }
    if(maxCost_ == 0)
        maxCost_ = 1;
}

void HeuristicCostMap::computeBorderCell(int dx, int dy, int & border_x, int & border_y) const
{
    int ind_x = dx + width_/2;
    int ind_y = dy + height_/2;

    // clamp the indices
    if(ind_x < 0)
        ind_x = 0;
    if(ind_x >= width_)
        ind_x = width_ - 1;
    if(ind_y < 0)
        ind_y = 0;
    if(ind_y >= height_)
        ind_y = height_ - 1;

    // compute back the max delta from these
    // These are signed clamping values, representing the max extends, not absolutes!
    int clamped_dx = ind_x - width_/2;
    int clamped_dy = ind_y - height_/2;

    // y coordinate corresponding to clamped_dx in direction of dx, dy, and vice versa
    int yForClamped_dx = dx==0? clamped_dy : (clamped_dx*dy)/dx;
    int xForClamped_dy = dy==0? clamped_dx : (clamped_dy*dx)/dy;

    // project either to x or y border and take the one that ends up in map
    int dxInMap = clamped_dx;
    int dyInMap = yForClamped_dx;
    // dyInMap == 0 -> keep this
    if((dyInMap > 0 && dyInMap > clamped_dy) ||
            (dyInMap < 0 && dyInMap < clamped_dy)) {
        dxInMap = xForClamped_dy;
        dyInMap = clamped_dy;
        assert((dxInMap >= 0 && dxInMap <= clamped_dx) || (dxInMap <= 0 && dxInMap >= clamped_dx));
    }
    border_x = dxInMap;
    border_y = dyInMap;
}

unsigned int HeuristicCostMap::getEuclideanCost(int dx, int dy, int startTheta, int endTheta) const
{
    // See EnvironmentNAVXYTHETALATTICE::PrecomputeActionswithCompleteMotionPrimitive
    double linear_time = hypot(dx, dy)/tv_;
    double angular_distance =
        fabs(computeMinUnsignedAngleDiff(DiscTheta2Cont(endTheta, numThetaDirs_),
                    DiscTheta2Cont(startTheta, numThetaDirs_)));
    double angular_time = angular_time/rv_;
    return (int)ceil(1000.0 * std::max(linear_time, angular_time));
}

unsigned int HeuristicCostMap::getCost(int dx, int dy, int startTheta, int endTheta) const
{
    int ind_x = dx + width_/2;
    int ind_y = dy + height_/2;
    if(ind_x < 0 || ind_x >= width_ || ind_y < 0 || ind_y >= height_
            || startTheta < 0 || startTheta >= numThetaDirs_ || endTheta < 0 || endTheta >= numThetaDirs_) {
        switch(outOfMapBehaviour_) {
            case OutOfMapMaxCost:
                return maxCost_;
            case OutOfMapInfiniteCost:
                return INFINITECOST;
            case OutOfMapAssert:
                assert(ind_x >= 0 && ind_x < width_ && ind_y >= 0 && ind_y < height_);
                assert(startTheta >= 0 && startTheta < numThetaDirs_ && endTheta >= 0 && endTheta < numThetaDirs_);
                return 0;
            case OutOfMapExpandEuclideanPrepend:
                {
                // compute border cell coordinates
                int bx, by;
                computeBorderCell(dx, dy, bx, by);
                int ind_bx = bx + width_/2;
                int ind_by = by + height_/2;
                assert(ind_bx >= 0 && ind_bx < width_ && ind_by >= 0 && ind_by < height_);
                // compute cost of border cell to goal delta
                // Nevermind if prepend or append, we'll always take the largest possible translation
                // part in the costmap
                int minCost = costmaps_[0][ind_bx][ind_by][endTheta] +
                    getEuclideanCost(dx - bx, dy - by, startTheta, 0);
                for(int i = 1; i < numThetaDirs_; ++i) {
                    int cost = costmaps_[i][ind_bx][ind_by][endTheta] +
                        getEuclideanCost(dx - bx, dy - by, startTheta, i);
                    if(cost < minCost)
                        minCost = cost;
                }
                return minCost;
                }
            case OutOfMapExpandEuclideanAppend:
                {
                // compute border cell coordinates
                int bx, by;
                computeBorderCell(dx, dy, bx, by);
                int ind_bx = bx + width_/2;
                int ind_by = by + height_/2;
                assert(ind_bx >= 0 && ind_bx < width_ && ind_by >= 0 && ind_by < height_);
                // compute cost of border cell to goal delta
                int minCost = costmaps_[startTheta][ind_bx][ind_by][0] +
                    getEuclideanCost(dx - bx, dy - by, 0, endTheta);
                for(int i = 1; i < numThetaDirs_; ++i) {
                    int cost = costmaps_[startTheta][ind_bx][ind_by][i] +
                        getEuclideanCost(dx - bx, dy - by, i, endTheta);
                    if(cost < minCost)
                        minCost = cost;
                }
                return minCost;
                }
            case OutOfMapRecursiveQuery:
                return getCostRecurs(dx, dy, startTheta, endTheta, 0);
            case OutOfMapZero:
                return 0;
        }
    }

    assert(costmaps_.size() == numThetaDirs_);
    return costmaps_[startTheta][ind_x][ind_y][endTheta];
}

unsigned int HeuristicCostMap::getCostRecurs(int dx, int dy, int startTheta, int endTheta, int depth) const
{
    if(depth >= 6) {
        ROS_WARN_THROTTLE(1.0, "Freespace Heuristic depth: %d", depth);
    }

    // compute border cell coordinates
    int bx, by;
    computeBorderCell(dx, dy, bx, by);
    if(dx == bx && dy == by) {
        return getCost(dx, dy, startTheta, endTheta);
    }

    int ind_bx = bx + width_/2;
    int ind_by = by + height_/2;
    assert(ind_bx >= 0 && ind_bx < width_ && ind_by >= 0 && ind_by < height_);

    int hdelta = getCost(bx, by, startTheta, 0)
        + getCostRecurs(dx - bx, dy - by, 0, endTheta, depth + 1);
    for(size_t i = 1; i < numThetaDirs_; ++i){
        int hnewdelta = getCost(bx, by, startTheta, i)
            + getCostRecurs(dx - bx, dy - by, i, endTheta, depth + 1);
        if(hnewdelta < hdelta){
            hdelta = hnewdelta;
        }
    }
    return hdelta;
}

}

