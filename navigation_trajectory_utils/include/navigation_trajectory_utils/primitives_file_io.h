/*
 * MotionPrimitiveGenerator.h
 *
 *  Created on: Jul 28, 2016
 *      Author: andreas
 */

#ifndef SRC_MOTION_PRIMITIVE_GENERATOR_H_
#define SRC_MOTION_PRIMITIVE_GENERATOR_H_

#include <ros/ros.h>
#include <boost/exception/all.hpp>
#include <sbpl/discrete_space_information/environment_navxythetalat.h>
//#include <sbpl/utils/key.h>

typedef boost::tuple<std::string, std::string> expected_found_mismatch;
typedef boost::error_info<struct key_mismatch_string, expected_found_mismatch> key_mismatch_info;
struct key_mismatch_error: virtual boost::exception {};
typedef boost::error_info<struct syntax_error_string, expected_found_mismatch> syntax_error_info;
struct primitives_file_syntax_error: virtual boost::exception {};

class PrimitivesFileIO
{
public:

	PrimitivesFileIO() : primitives_scaling_factor(1)
	{
	}

	void writePrimitives(const std::string& filepath, EnvNAVXYTHETALATConfig_t& config);
	void readPrimitives(std::vector<std::string>& lines, EnvNAVXYTHETALATConfig_t& config);

protected:
	typedef boost::iterator_range<std::vector<std::string>::const_iterator> StrListIt;
	void fillMotionPrimitive(StrListIt& it, SBPL_xytheta_mprimitive& primitive);
	void fillDiscretePose(StrListIt& it, const std::string& key, sbpl_xy_theta_cell_t& pose);
	void fillPose(StrListIt& it, sbpl_xy_theta_pt_t& pose);
	int readIntValue(StrListIt& it, const std::string& key);
	double readDoubleValue(StrListIt& it, const std::string& key);

	double primitives_scaling_factor;
};

#endif /* SRC_MOTION_PRIMITIVE_GENERATOR_H_ */
