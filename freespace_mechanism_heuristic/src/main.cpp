/*
 * main.cpp
 *
 *  Created on: Jul 27, 2016
 *      Author: andreas
 */

#include <getopt.h>
#include <stdlib.h>
#include "freespace_mechanism_heuristic/compute_heuristic.h"

void printUsage() {
	printf(
			"Usage: compute_heuristic --sx SIZE_X --sy SIZE_Y --mprims MPRIM_FILE --tv LINEAR_VEL_M_PER_S --rv ANGULAR_VEL_RAD_SECS -i (enable write debug images)\n");
}

int main(int argc, char** argv) {
	std::string mprims_path;
	int size_x, size_y;
	double linear_velocity;
	double angular_velocity;

	int c;
	static struct option long_options[] =
	{
			{ "sx", required_argument, 0, 'x' },
			{ "sy", required_argument, 0, 'y' },
			{ "mprims", required_argument, 0, 'f' },
			{ "tv", required_argument, 0, 't' },
			{ "rv", required_argument, 0, 'r' },
			{ 0, 0, 0, 0 }
	};
	while (true) {
		int option_index = 0;
		c = getopt_long(argc, argv, "hix:y:f:t:r:", long_options,
				&option_index);
		if (c == -1)
			break;
		switch (c) {
		case 'h':
			printUsage();
			return 1;
		case 'i':
			g_WriteImages = true;
			break;
		case 'x':
			size_x = atoi(optarg);
			break;
		case 'y':
			size_y = atoi(optarg);
			break;
		case 'f':
			mprims_path = optarg;
			break;
		case 't':
			linear_velocity = atof(optarg);
			break;
		case 'r':
			angular_velocity = atof(optarg);
			break;
		}
	}

	printf("Grid (%d x %d), tv: %.2f rv: %.2f\n", size_x, size_y,
			linear_velocity, angular_velocity);
	printf("Reading prims from %s\n", mprims_path.c_str());
	freespace_mechanism_heuristic::HeuristicCostMapPtr costmap;
	int err_code = generateHeuristicMap(costmap, mprims_path,
			linear_velocity, angular_velocity,
			size_x, size_y);
	if (err_code == 0)
		costmap->saveCostMap(mprims_path + "_costmap.dat");
	return err_code;
}
