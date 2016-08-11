/*
 * MotionPrimitiveGenerator.cpp
 *
 *  Created on: Jul 28, 2016
 *      Author: andreas
 */

#include <iostream>
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH
#include <navigation_trajectory_utils/primitives_file_io.h>

using namespace std;
using namespace boost;
using namespace boost::algorithm;

void PrimitivesFileIO::writePrimitives(const std::string & filepath, EnvNAVXYTHETALATConfig_t& config)
{
	ofstream primitives_file(filepath.c_str());
	/// SBPL motion primitive header
	//	resolution_m: 0.050000
	//	numberofangles: 16
	//	totalnumberofprimitives: 144
	primitives_file << "resolution_m: " << config.cellsize_m << endl;
	primitives_file << "numberofangles: " << config.NumThetaDirs << endl;
	primitives_file << "totalnumberofprimitives: " << config.mprimV.size() << endl;
	forEach(const SBPL_xytheta_mprimitive& primitive, config.mprimV)
	{
		/// SBPL motion primitive
		//	primID: 0
		//	startangle_c: 0
		//	endpose_c: 1 0 0
		//	additionalactioncostmult: 1
		//	intermediateposes: 10
		//	0.0000 0.0000 0.0000
		primitives_file << "primID: " << primitive.motprimID << endl;
		primitives_file << "startangle_c: " << primitive.starttheta_c << endl;
		primitives_file << "endpose_c: " << primitive.endcell.x << " " << primitive.endcell.y << " " << primitive.endcell.theta << endl;
		primitives_file << "additionalactioncostmult: " << primitive.additionalactioncostmult << endl;
		primitives_file << "intermediateposes: " << primitive.intermptV.size() << endl;
		forEach(const sbpl_xy_theta_pt_t& pose, primitive.intermptV)
		{
			primitives_file << pose.x << " " << pose.y << " " << pose.theta << endl;
		}
	}
	primitives_file.close();
}

void PrimitivesFileIO::readPrimitives(std::vector<std::string>& lines, EnvNAVXYTHETALATConfig_t& config)
{
	StrListIt it(lines.begin(), lines.end());
	try
	{
		/// SBPL motion primitive header
		//	resolution_m: 0.050000
		//	numberofangles: 16
		//	totalnumberofprimitives: 144

		primitives_scaling_factor = config.cellsize_m / readDoubleValue(it, "resolution_m");
		config.NumThetaDirs = readIntValue(it, "numberofangles");
		int primitives_count = readIntValue(it, "totalnumberofprimitives");

		vector<SBPL_xytheta_mprimitive>& primitives = config.mprimV;
		primitives.resize(primitives_count);
		forEach (SBPL_xytheta_mprimitive& primitive, primitives)
		{
			fillMotionPrimitive(it, primitive);
		}
	}
	catch(boost::exception& e)
	{
		int line_number = it.begin() - lines.begin();
		e << boost::errinfo_at_line(line_number);
		ROS_FATAL_STREAM("Error while parsing primitives file at line "<< lexical_cast<string>(line_number)<<":"<<endl<<it.front());
		throw;
	}
}

void PrimitivesFileIO::fillMotionPrimitive(StrListIt& it, SBPL_xytheta_mprimitive& primitive)
{
	/// SBPL motion primitive
	//	primID: 0
	//	startangle_c: 0
	//	endpose_c: 1 0 0
	//	additionalactioncostmult: 1
	//	intermediateposes: 10
	primitive.motprimID = readIntValue(it, "primID");
	primitive.starttheta_c = readIntValue(it, "startangle_c");
	fillDiscretePose(it, "endpose_c", primitive.endcell);
	primitive.additionalactioncostmult = readDoubleValue(it, "additionalactioncostmult");
	int poses_count = readIntValue(it, "intermediateposes");
	primitive.intermptV.resize(poses_count);
	forEach (sbpl_xy_theta_pt_t& pose, primitive.intermptV)
	{
		fillPose(it, pose);
	}
}

void PrimitivesFileIO::fillDiscretePose(StrListIt& it, const std::string& key, sbpl_xy_theta_cell_t& pose)
{
	/// SBPL discrete pose
	//	endpose_c: 1 0 0
	if (it.empty())
		throw primitives_file_syntax_error() << syntax_error_info(expected_found_mismatch(key+": <int> <int> <int>", "EOF"));
	string line = it.front();
	vector<string> key_value;
	split(key_value, line, is_any_of(": \n"), token_compress_on);
	if (key_value.size() < 4)
		throw primitives_file_syntax_error() << syntax_error_info(expected_found_mismatch(key+": <int> <int> <int>", line));
	if (key != key_value[0])
		throw key_mismatch_error() << key_mismatch_info(expected_found_mismatch(key, key_value[0]));
	pose.x = boost::lexical_cast<int>(key_value[1]);
	pose.y = boost::lexical_cast<int>(key_value[2]);
	pose.theta = boost::lexical_cast<int>(key_value[3]);
	it.pop_front();
}

void PrimitivesFileIO::fillPose(StrListIt& it, sbpl_xy_theta_pt_t& pose)
{
	/// SBPL pose
	/// [m]    [m]    [rad]
	//	0.0000 0.0000 0.0000
	if (it.empty())
		throw primitives_file_syntax_error() << syntax_error_info(expected_found_mismatch("<double> <double> <double>", "EOF"));
	string line = it.front();
	vector<string> numbers;
	split(numbers, line, is_any_of(" \n"), token_compress_on);
	if (numbers.size() < 3)
		throw primitives_file_syntax_error() << syntax_error_info(expected_found_mismatch("<double> <double> <double>", line));
	pose.x = lexical_cast<double>(numbers[0]) * primitives_scaling_factor;
	pose.y = lexical_cast<double>(numbers[1]) * primitives_scaling_factor;
	pose.theta = lexical_cast<double>(numbers[2]);
	it.pop_front();
}

int PrimitivesFileIO::readIntValue(StrListIt& it, const std::string& key)
{
	if (it.empty())
		throw primitives_file_syntax_error() << syntax_error_info(expected_found_mismatch(key+": <int>", "EOF"));
	string line = it.front();
	vector<string> key_value;
	split(key_value, line, is_any_of(": \n"), token_compress_on);
	if (key_value.size() < 2)
		throw primitives_file_syntax_error() << syntax_error_info(expected_found_mismatch(key+": <int>", line));
	if (key != key_value[0])
		throw key_mismatch_error() << key_mismatch_info(expected_found_mismatch(key, key_value[0]));
	int value = lexical_cast<int>(key_value[1]);
	it.pop_front();
	return value;
}

double PrimitivesFileIO::readDoubleValue(StrListIt& it, const std::string& key)
{
	if (it.empty())
		throw primitives_file_syntax_error() << syntax_error_info(expected_found_mismatch(key+": <double>", "EOF"));
	string line = it.front();
	vector<string> key_value;
	split(key_value, line, is_any_of(": \n"), token_compress_on);
	if (key_value.size() < 2)
		throw primitives_file_syntax_error() << syntax_error_info(expected_found_mismatch(key+": <double>", line));
	if (key != key_value[0])
		throw key_mismatch_error() << key_mismatch_info(expected_found_mismatch(key, key_value[0]));
	double value = lexical_cast<double>(key_value[1]);
	it.pop_front();
	return value;
}
