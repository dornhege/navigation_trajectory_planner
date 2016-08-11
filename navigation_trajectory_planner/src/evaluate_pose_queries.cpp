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
        std::string paramName;
        XmlRpc::XmlRpcValue parameter;

        std::vector<navigation_trajectory_msgs::PlannerStats> planner_stats;

        static std::vector<ParameterRun> setupRuns(const std::string & paramName,
                const std::vector<std::string> & paramValues) {
            std::vector<ParameterRun> runs;

            XmlRpc::XmlRpcValue param;
            if(!ros::param::get(paramName, param)) {
                ROS_ERROR("Could not get parameter %s to setup runs.", paramName.c_str());
                return runs;
            }

            ParameterRun run;
            run.paramName = paramName;

            if(param.getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
                if(!paramValues.empty()) {
                  forEach(const std::string & paramStr, paramValues) {
                    bool val = (paramStr == "true"? true: false);
                    run.parameter = XmlRpc::XmlRpcValue(val);
                    runs.push_back(run);
                  }
                }else{
                  run.parameter = XmlRpc::XmlRpcValue(true);
                  runs.push_back(run);
                  run.parameter = XmlRpc::XmlRpcValue(false);
                  runs.push_back(run);
                }
            } else if(param.getType() == XmlRpc::XmlRpcValue::TypeInt) {
                forEach(const std::string & paramStr, paramValues) {
                    std::stringstream ss(paramStr);
                    int val;
                    ss >> val;
                    run.parameter = XmlRpc::XmlRpcValue(val);
                    runs.push_back(run);
                }
            } else if(param.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                forEach(const std::string & paramStr, paramValues) {
                    std::stringstream ss(paramStr);
                    double val;
                    ss >> val;
                    run.parameter = XmlRpc::XmlRpcValue(val);
                    runs.push_back(run);
                }
            } else if(param.getType() == XmlRpc::XmlRpcValue::TypeString) {
                forEach(const std::string & paramStr, paramValues) {
                    run.parameter = XmlRpc::XmlRpcValue(paramStr);
                    runs.push_back(run);
                }
            }
        }
};


int main(int argc, char** argv)
{
  if(argc < 5){
    std::cout << "Usage: " << argv[0] << " <BaseGlobalPlanner name> <parametername> <minDist> <maxDist> [paramValue1 paramValue2 ...]" << std::endl;
    std::cout << "Note: parameter values are ignored for Boolean params and otherwise converted to the type of the ROS parameter that is set on the server. Thus for non-Boolean parameters it is necessary that the respective value is already on the parameter server." << std::endl;
    return 1;
  }

  ros::init(argc, argv, "evaluate_pose_queries");
  ros::NodeHandle nh;

  std::string plannerName(argv[1]);
  std::string parameterName(argv[2]);
  minDist = atof(argv[3]);
  maxDist = atof(argv[4]);
  startAtQuery = atoi(argv[5]);
  std::vector<std::string> paramValues;
  for(int pInd = 6; pInd < argc; pInd++) {
      paramValues.push_back(argv[pInd]);
  }

  std::stringstream parameterRosName;
  parameterRosName << "/move_base_node/" << plannerName << "/" << parameterName;
  std::vector<ParameterRun> param_runs = ParameterRun::setupRuns(parameterRosName.str(), paramValues);

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

  forEach(ParameterRun & run, param_runs) {
      ros::param::set(run.paramName, run.parameter);
      collectData();
      run.planner_stats = g_Stats;
  }

  YAML::Emitter emitter;
  emitter << YAML::BeginMap;
  emitter << YAML::Key;
  emitter << parameterRosName.str();
  emitter << YAML::Value;

  emitter << YAML::BeginMap;
  forEach(ParameterRun & run, param_runs) {
      if(poseQueries.size() != run.planner_stats.size()) {
          ROS_ERROR("poseQueries size %zu != %s planner_stats size %zu",
                  poseQueries.size(), parameterName.c_str(), run.planner_stats.size());
      }

      emitter << YAML::Key;
      if(run.parameter.getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
          bool isOn = run.parameter;
          emitter << (isOn ? "true" : "false");  // be a bit nicer with bools, otherwise 1, 0
      } else {
          std::stringstream paramValueStr;
          run.parameter.write(paramValueStr);
          emitter << paramValueStr.str();
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

