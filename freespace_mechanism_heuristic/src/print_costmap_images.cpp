#include "freespace_mechanism_heuristic/freespace_mechanism_heuristic.h"
#include <stdio.h>
#include <sstream>
#include <iomanip>
#include <stdlib.h>

int main(int argc, char** argv)
{
    if(argc < 2) {
        fprintf(stderr, "No costmaps filename given.\n");
        fprintf(stderr, "Usage: print_costmap_images costmap_file [costmap expansion]\n");
        fprintf(stderr, "Note: Expansion will be done with Euclidean append. If a negative value is given Euclidean prepend is used.\n");
        return 1;
    }

    std::string fname = argv[1];
    int expansion = 0;
    if(argc >= 3)
        expansion = atoi(argv[2]);
    enum freespace_mechanism_heuristic::HeuristicCostMap::OutOfMapBehavior mode =
        freespace_mechanism_heuristic::HeuristicCostMap::OutOfMapExpandEuclideanAppend;
    if(expansion < 0) {
        mode = freespace_mechanism_heuristic::HeuristicCostMap::OutOfMapExpandEuclideanPrepend;
        expansion = -expansion;
    }


    freespace_mechanism_heuristic::HeuristicCostMap costmap(fname, mode);

    for(int sth = 0; sth < costmap.getNumThetaDirs(); ++sth) {
        for(int eth = 0; eth < costmap.getNumThetaDirs(); ++eth) {
            std::stringstream ss;
            ss << "costmap_" << std::setfill('0') << std::setw(2) << sth << "_" << std::setw(2) << eth << ".ppm";
            costmap.saveCostMapImage(ss.str(), sth, eth, expansion);
        }
        std::stringstream ss;
        ss << "costmap_" << std::setfill('0') << std::setw(2) << sth << "_bestTh.ppm";
        costmap.saveCostMapImage(ss.str(), sth, -1, expansion);
    }

    return 0;
}

