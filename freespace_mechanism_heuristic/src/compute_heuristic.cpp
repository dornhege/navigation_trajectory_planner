#include <string>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <queue>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <boost/make_shared.hpp>
#include <navigation_trajectory_utils/primitives_file_io.h>
#include "freespace_mechanism_heuristic/compute_heuristic.h"

bool EnvironmentMotionPrims::readPrimitives(const std::string & fname, double resolution)
{
	EnvNAVXYTHETALATCfg.cellsize_m = resolution;
	FILE* fMotPrims = fopen(fname.c_str(), "r");
	if (! fMotPrims)
		return false;

	char buf[1024];
	std::vector<std::string> lines;
	while (fgets(buf, 1000, fMotPrims)!=NULL)
	{
		lines.push_back(buf);
	}
	fclose(fMotPrims);

	PrimitivesFileIO primitivesReader;
	primitivesReader.readPrimitives(lines, EnvNAVXYTHETALATCfg);

	// manually trigger here to get computed actions
	InitializeEnvConfig(&EnvNAVXYTHETALATCfg.mprimV);

	return true;
}

namespace freespace_mechanism_heuristic
{

void computeCosts(const EnvNAVXYTHETALATConfig_t & cfg, int theta,
		HeuristicCostMapPtr & costmaps)
{
	costmaps->maxCost_ = INFINITECOST;
	unsigned int*** costmap = costmaps->getCostMap(theta);

	std::priority_queue<SearchNode> queue;
	queue.push(SearchNode(cfg.EnvWidth_c / 2, cfg.EnvHeight_c / 2, theta, 0));

	unsigned int grid_size = cfg.EnvWidth_c * cfg.EnvHeight_c
			* cfg.NumThetaDirs;

	const bool debug = false;

	int count = 0;
	unsigned int expanded = 0;
	while (!queue.empty())
	{
		unsigned int debugInterval = 1000;
		if (debug && expanded % (debugInterval) == 0)
		{
			printf("Queue: %zu, Expanded: %d Grid: %d (%.2f x)\n", queue.size(),
					expanded,
					cfg.EnvWidth_c * cfg.EnvHeight_c * cfg.NumThetaDirs,
					(double) expanded
							/ (cfg.EnvWidth_c * cfg.EnvHeight_c
									* cfg.NumThetaDirs) * 1.0);
			unsigned int filled = 0;
			for (unsigned int x = 0; x < cfg.EnvWidth_c; ++x)
			{
				for (unsigned int y = 0; y < cfg.EnvHeight_c; ++y)
				{
					for (unsigned int th = 0; th < cfg.NumThetaDirs; ++th)
					{
						if (costmap[x][y][th] < INFINITECOST)
							filled++;
					}
				}
			}
			printf("Filled: %d/%d (%.2f %%)\n", filled, grid_size,
					double(filled) / grid_size * 100.0);
			if (filled > 0)
			{
				costmaps->updateMaxCost();
				printf("Max Cost is: %d\n", costmaps->maxCost_);
				std::stringstream ss;
				for (int th = 0; th < cfg.NumThetaDirs; th++)
				{
					ss << std::setfill('0') << std::setw(6) << count;
					std::string countStr = ss.str();
					ss.str("");

					ss << std::setfill('0') << std::setw(2) << th;
					std::string endTh = ss.str();
					ss.str("");

					ss << std::setfill('0') << "costmap_" << std::setw(2)
							<< theta << "_" << countStr << "_" << endTh
							<< ".pgm";
					if (g_WriteImages)
						costmaps->saveCostMapImage(ss.str(), theta, th);
					ss.str("");
				}
				ss << std::setfill('0') << std::setw(6) << count;
				std::string countStr = ss.str();
				ss.str("");
				ss << std::setfill('0') << "costmap_" << std::setw(2) << theta
						<< "_bestTh" << "_" << countStr << ".pgm";
				if (g_WriteImages)
					costmaps->saveCostMapImage(ss.str(), theta, -1);

				count++;
				if (count > 3)
				{
					printf("Stopping at count %d\n", count);
					break;
				}
			}
		}
		SearchNode current = queue.top();
		queue.pop();

		if (current.g >= costmap[current.x][current.y][current.theta])
			continue;
		costmap[current.x][current.y][current.theta] = current.g;

		expanded++;
		for (unsigned int i = 0; i < cfg.actionwidth; ++i)
		{
			const EnvNAVXYTHETALATAction_t & act =
					cfg.ActionsV[current.theta][i];
			SearchNode succ(current.x + act.dX, current.y + act.dY,
					act.endtheta, current.g + act.cost);
			if (succ.x < 0 || succ.x >= cfg.EnvWidth_c || succ.y < 0
					|| succ.y >= cfg.EnvHeight_c)
				continue;
			if (succ.g < costmap[succ.x][succ.y][succ.theta])
				queue.push(succ);
		}
	}
	printf("Total expanded: %d\n", expanded);

	costmaps->updateMaxCost();
	printf("Max Cost is: %d\n", costmaps->maxCost_);
	std::stringstream ss;
	for (int th = 0; th < cfg.NumThetaDirs; th++)
	{
		std::string countStr = "final";

		ss << std::setfill('0') << std::setw(2) << th;
		std::string endTh = ss.str();
		ss.str("");

		ss << std::setfill('0') << "costmap_" << std::setw(2) << theta << "_"
				<< countStr << "_" << endTh << ".pgm";
		if (g_WriteImages)
			costmaps->saveCostMapImage(ss.str(), theta, th);
		ss.str("");
	}
	std::string countStr = "final";
	ss << std::setfill('0') << "costmap_" << std::setw(2) << theta << "_bestTh"
			<< "_" << countStr << ".pgm";
	if (g_WriteImages)
		costmaps->saveCostMapImage(ss.str(), theta, -1);
}

int generateHeuristicMap(HeuristicCostMapPtr& costmap,
		const std::string& mprims_file,
		double resolution,
		double linear_velocity, double angular_velocity,
		int size_x, int size_y)
{
	double timetoturn45degsinplace_secs = M_PI_4 / angular_velocity;
	EnvironmentMotionPrims env(size_x, size_y, linear_velocity,
			timetoturn45degsinplace_secs);

	if (!env.readPrimitives(mprims_file, resolution))
	{
		fprintf(stderr, "Failed to read prims.\n");
		return 1;
	}
	const EnvNAVXYTHETALATConfig_t& cfg = env.getConfig();

	//double rv = M_PI / 4.0 / timetoturn45degsinplace_secs;
	double tv = linear_velocity / cfg.cellsize_m; // euclidean queries will come in cells, not m

	costmap = boost::make_shared<HeuristicCostMap>(cfg.EnvHeight_c,
			cfg.EnvWidth_c, cfg.NumThetaDirs, tv, angular_velocity, true,
			HeuristicCostMap::OutOfMapAssert);

	for (unsigned int th = 0; th < cfg.NumThetaDirs; ++th)
	{
		printf("Computing costs for th: %d\n", th);
		computeCosts(cfg, th, costmap);
	}
	return 0;
}

}
