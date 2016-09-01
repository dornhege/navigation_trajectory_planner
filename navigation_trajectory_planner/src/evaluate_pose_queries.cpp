#include "navigation_trajectory_msgs/PlannerStats.h"

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GetPlan.h>
#include <yaml-cpp/emitter.h>
#include <sstream>
#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH

std::vector< std::pair<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> > poseQueries;
std::vector<navigation_trajectory_msgs::PlannerStats> g_Stats;
bool g_StatsReceived;
double minDist, maxDist;
bool g_ReverseQueries = true;
int startAtQuery = 0;

void poseArrayCallback(const geometry_msgs::PoseArray & pa)
{
  std::cout << "got called back" << std::endl;
  ROS_INFO("Got PA size: %zu", pa.poses.size());

  geometry_msgs::PoseStamped psStart;
  geometry_msgs::PoseStamped psGoal;
  psStart.header = pa.header;
  psGoal.header = pa.header;
  for(int i = 0; i < pa.poses.size(); ++i) {
    // no reverse queries?
    int j = i + 1;
    if(g_ReverseQueries)
      j = 0;
    for(; j < pa.poses.size(); ++j) {
      if(i == j)
        continue;
      psStart.pose = pa.poses[i];
      psGoal.pose = pa.poses[j];
      if(hypot(psStart.pose.position.x - psGoal.pose.position.x,
               psStart.pose.position.y - psGoal.pose.position.y) < minDist
         || hypot(psStart.pose.position.x - psGoal.pose.position.x,
                  psStart.pose.position.y - psGoal.pose.position.y) > maxDist){
        std::cout << "Poses too close together or too far apart. Skipping." << std::endl;
        continue;
      }
      std::cout << "adding pose query." << std::endl;
      poseQueries.push_back(std::make_pair(psStart, psGoal));
    }
  }
}

void statsCallback(const navigation_trajectory_msgs::PlannerStats & stats)
{
  g_Stats.push_back(stats);
  g_StatsReceived = true;
}

void collectData()
{
  ROS_INFO("Got %zu queries, starting at query %d", poseQueries.size(), startAtQuery);
  g_Stats.clear();
  for(int i = startAtQuery; i < poseQueries.size(); ++i) {
    if(!ros::ok())
      break;
    g_StatsReceived = false;
    nav_msgs::GetPlan srv;
    srv.request.start = poseQueries[i].first;
    srv.request.goal = poseQueries[i].second;
    bool err = false;
    ROS_INFO("Calling plan for query: %d", i);
    if(!ros::service::call("/move_base_node/make_plan", srv) || srv.response.plan.poses.empty()) {
      ROS_ERROR("Could not plan for %d", i);
      // FIXME also no plan found
      err = true;
    }
    int count = 0;
    while(!g_StatsReceived) {
      ros::spinOnce();
      ros::Duration(0.1).sleep();
      count++;
      if(count > 10) {
        if(err) {
          ROS_INFO("Inserting empty stats");
          g_Stats.push_back(navigation_trajectory_msgs::PlannerStats());
          break;
        }
      }
    }
  }
}

template <typename T>
void writeKeyVal(YAML::Emitter & em, const std::string & key, const T& val)
{
  em << YAML::Key;
  em << key;
  em << YAML::Value;
  em << val;
}

void yamlPose(YAML::Emitter & emitter, const geometry_msgs::PoseStamped & ps)
{
  emitter << YAML::BeginMap;
  writeKeyVal(emitter, "frame_id", ps.header.frame_id);
  writeKeyVal(emitter, "x", ps.pose.position.x);
  writeKeyVal(emitter, "y", ps.pose.position.y);
  writeKeyVal(emitter, "z", ps.pose.position.z);
  writeKeyVal(emitter, "qx", ps.pose.orientation.x);
  writeKeyVal(emitter, "qy", ps.pose.orientation.y);
  writeKeyVal(emitter, "qz", ps.pose.orientation.z);
  writeKeyVal(emitter, "qw", ps.pose.orientation.w);
  emitter << YAML::EndMap;
}

class ParameterRun
{
public:
  std::vector<std::string> paramNames;
  std::vector<XmlRpc::XmlRpcValue> parameters;

  std::vector<navigation_trajectory_msgs::PlannerStats> planner_stats;

  static std::vector<ParameterRun> setupRuns(const std::vector<std::string> & paramNames,
                                             const std::vector<std::vector<std::string> > & paramValues) {
    std::vector<ParameterRun> runs;

    std::vector<XmlRpc::XmlRpcValue> params(paramNames.size());
    for(size_t i = 0; i < paramNames.size(); ++i){
      if(!ros::param::get(paramNames[i], params[i])) {
        ROS_ERROR("Could not get parameter %s to setup runs.", paramNames[i].c_str());
        return runs;
      }
    }

    ParameterRun run;
    run.paramNames = paramNames;
    run.parameters.resize(paramNames.size());
    int numRuns = 1;
    for(size_t i = 0; i < paramNames.size(); ++i){
      numRuns *= paramValues[i].size();
    }
    runs.resize(numRuns, run);

    std::vector<std::vector<std::string>::const_iterator> it(paramNames.size());
    for(size_t i = 0; i < paramNames.size(); ++i){
      it[i] = paramValues[i].begin();
    }

    std::vector<std::vector<std::string> > paramCombinations(numRuns);
    int count = 0;
    while(count < numRuns){
      for(size_t p = 0; p < paramNames.size(); ++p){
        paramCombinations[count].push_back(*(it[p]));
      }
      count++;

      int currentParam = 0;
      while(currentParam < paramValues.size()){
        //for(int currentParam = 0; currentParam != paramValues.size()-1; ){
        ++it[currentParam];
        if(it[currentParam] == paramValues[currentParam].end()){
          if(currentParam == paramValues.size()-1){
            goto useCombinations;
          }
          it[currentParam] = paramValues[currentParam].begin();
          currentParam++;
        }else{
          break;
        }
      }
    }

  useCombinations:
    for(size_t r = 0; r < numRuns; ++r){
      for(size_t i = 0; i < paramNames.size(); ++i){
        const std::string & paramStr = paramCombinations[r][i];
        if(params[i].getType() == XmlRpc::XmlRpcValue::TypeBoolean){
          bool val = (paramStr == "true"? true: false);
          runs[r].parameters[i] = XmlRpc::XmlRpcValue(val);
        }else if(params[i].getType() == XmlRpc::XmlRpcValue::TypeInt){
          std::stringstream ss(paramStr);
          int val;
          ss >> val;
          runs[r].parameters[i] = XmlRpc::XmlRpcValue(val);
        }else if(params[i].getType() == XmlRpc::XmlRpcValue::TypeDouble){
          std::stringstream ss(paramStr);
          double val;
          ss >> val;
          runs[r].parameters[i] = XmlRpc::XmlRpcValue(val);
        }else if(params[i].getType() == XmlRpc::XmlRpcValue::TypeString) {
          runs[r].parameters[i] = XmlRpc::XmlRpcValue(paramStr);
        }
      }
    }
    std::cout << "got " << numRuns << " runs" << std::endl;
    return runs;
  }
};


  int main(int argc, char** argv)
  {
    if(argc < 6){
      std::cout << "Usage: " << argv[0] << " <BaseGlobalPlanner name> <number of parameters> <parametername1> <parametername2> ... <minDist> <maxDist> <start at query> [numParam1Values param1Value1 param1Value2 ...] [numParam2Values param2Value1 param2Value2 ...] ..." << std::endl;
      std::cout << "Note: parameter values are converted to the type of the ROS parameter that is set on the server. Thus it is necessary that the respective value is already on the parameter server." << std::endl;
      return 1;
    }

    ros::init(argc, argv, "evaluate_pose_queries");
    ros::NodeHandle nh;

    std::string plannerName(argv[1]);
    int numParams = atoi(argv[2]);
    if(argc < 5+numParams){
      std::cout << "given number of parameters does not match the number of given parameters." << std::endl;
      std::cout << "Usage: " << argv[0] << " <BaseGlobalPlanner name> <number of parameters> <parametername1> <parametername2> ... <minDist> <maxDist> <start at query> [numParam1Values param1Value1 param1Value2 ...] [numParam2Values param2Value1 param2Value2 ...] ..." << std::endl;
      std::cout << "Note: parameter values are converted to the type of the ROS parameter that is set on the server. Thus it is necessary that the respective value is already on the parameter server." << std::endl;
      return 1;
    }

    std::vector<std::string> paramNames(numParams);
    for(int i = 0; i < numParams; ++i){
      paramNames[i] = std::string(argv[3+i]);
    }
    minDist = atof(argv[3+numParams]);
    maxDist = atof(argv[4+numParams]);
    startAtQuery = atoi(argv[5+numParams]);
    std::vector<std::vector<std::string> > paramValues(numParams);
    //std::vector<std::string> paramValues;
    bool newParam = true;
    int currentParam = 0;
    int numVals = 0;
    std::cout << "num params = " << numParams << " num args = " << argc << std::endl;
    for(int pInd = 6+numParams; pInd < argc; pInd++) {
      if(newParam){
        numVals = atoi(argv[pInd]);
        newParam = false;
      }else{
        paramValues[currentParam].push_back(argv[pInd]);
        numVals--;
        if(numVals == 0){
          newParam = true;
          currentParam++;
        }
      }
    }

    for(size_t i = 0; i < numParams; ++i){
      std::cout << "testing with parameter " << paramNames[i] << " for values ";
      for(size_t j = 0; j < paramValues[i].size(); ++j){
        std::cout << paramValues[i][j] << " ";
      }    
      std::cout << std::endl;
    }

    std::vector<std::string> parameterRosNames;
    for(size_t i = 0; i < numParams; ++i){
      std::stringstream parameterRosName;
      parameterRosName << "/move_base_node/" << plannerName << "/" << paramNames[i];
      parameterRosNames.push_back(parameterRosName.str());
    }
      
    std::vector<ParameterRun> param_runs = ParameterRun::setupRuns(parameterRosNames, paramValues);
    std::cout << "got runs" << std::endl;

    ros::Subscriber subPoses = nh.subscribe("/valid_poses", 3, poseArrayCallback);
    std::cout << "subscribed to valid poses" << std::endl;
    std::stringstream statsTopicName;
    statsTopicName << "/move_base_node/" << plannerName << "/planner_stats";
    ros::Subscriber subStats = nh.subscribe(statsTopicName.str(), 10, statsCallback);

    ros::Rate rate(10.0);
    while(ros::ok() && poseQueries.empty()) {
      ros::spinOnce();
      rate.sleep();
    }

    // force a replan every time, our queries shouldn't depend on each other, which might happen for efficiency.
    ros::param::set(std::string("/move_base_node/") + plannerName + "/force_scratch_limit", -1);
    std::cout << "setting force scratch limit" << std::endl;

    forEach(ParameterRun & run, param_runs) {
      std::cout << "running ";
      for(size_t i = 0; i < run.paramNames.size(); ++i){
        ros::param::set(run.paramNames[i], run.parameters[i]);
        std::cout << run.parameters[i] << " ";
      }
      std::cout << " collecting data...";
      collectData();
      std::cout << "done." << std::endl;
      run.planner_stats = g_Stats;
    }

    YAML::Emitter emitter;
    emitter << YAML::BeginMap;
    emitter << YAML::Key;
    for(size_t i = 0; i < paramNames.size()-1; ++i){
      emitter << parameterRosNames[i] << ",";
    }
    emitter << parameterRosNames[parameterRosNames.size()-1];
    emitter << YAML::Value;

    emitter << YAML::BeginMap;
    forEach(ParameterRun & run, param_runs) {
      if(poseQueries.size() != run.planner_stats.size()) {
        // TODO fix parameter name output
        ROS_ERROR("poseQueries size %zu != %s planner_stats size %zu",
                  poseQueries.size(), paramNames[0].c_str(), run.planner_stats.size());
      }

      emitter << YAML::Key;
      for(size_t i = 0; i < paramNames.size(); ++i){
        emitter << paramNames[i] << "_";
        if(run.parameters[i].getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
          bool isOn = run.parameters[i];
          emitter << (isOn ? "true" : "false");  // be a bit nicer with bools, otherwise 1, 0
        } else {
          std::stringstream paramValueStr;
          run.parameters[i].write(paramValueStr);
          emitter << paramValueStr.str();
        }
        if(i < paramNames.size()-1){
          emitter << "_";
        }
      }

      emitter << YAML::Value;
      emitter << YAML::BeginSeq;
      int stat_index = startAtQuery;
      forEach(const navigation_trajectory_msgs::PlannerStats & ps, run.planner_stats) {
        emitter << YAML::BeginMap;
        if(stat_index < poseQueries.size()) {
          emitter << YAML::Key;
          emitter << "start_pose";
          emitter << YAML::Value;
          yamlPose(emitter, poseQueries[stat_index].first);
          emitter << YAML::Key;
          emitter << "goal_pose";
          emitter << YAML::Value;
          yamlPose(emitter, poseQueries[stat_index].second);
        }
        emitter << YAML::Key;
        emitter << "planner_stats";
        emitter << YAML::Value;
        emitter << YAML::BeginSeq;
        forEach(const navigation_trajectory_msgs::PlannerStat & pss, ps.stats) {
          emitter << YAML::BeginMap;
          writeKeyVal(emitter, "eps", pss.eps);
          writeKeyVal(emitter, "suboptimality", pss.suboptimality);
          writeKeyVal(emitter, "g", pss.g);
          writeKeyVal(emitter, "cost", pss.cost);
          writeKeyVal(emitter, "time", pss.time);
          writeKeyVal(emitter, "expands", pss.expands);
          emitter << YAML::EndMap;
        }
        emitter << YAML::EndSeq;
        emitter << YAML::EndMap;
        stat_index++;
      }
      emitter << YAML::EndSeq;
    }
    emitter << YAML::EndMap;

    emitter << YAML::EndMap;

    FILE* f = fopen("evaluate_pose_queries.yaml", "w");
    if(!f)
      return 1;
    fprintf(f, "%s", emitter.c_str());
    fclose(f);
  }

