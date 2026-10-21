// Standalone library benchmark: deliberately no rclcpp init, node, or executor.
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <moveit/trajectory_processing/ruckig_traj_smoothing.hpp>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

std::string read(const char* path) {
  std::ifstream f(path); if (!f) throw std::runtime_error("Cannot read input");
  return {std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>()};
}
void dump(const robot_trajectory::RobotTrajectory& tr, const std::vector<std::string>& names) {
  std::cout << '[';
  for (size_t i=0;i<tr.getWayPointCount();++i) {
    if(i) std::cout << ',';
    const auto& s=tr.getWayPoint(i);
    std::cout << "{\"time\":" << tr.getWayPointDurationFromStart(i);
    for (const std::string field : {"positions","velocities","accelerations"}) {
      std::cout << ",\"" << field << "\":[";
      for (size_t j=0;j<names.size();++j) {
        if(j) std::cout << ',';
        std::cout << (field=="positions" ? s.getVariablePosition(names[j]) :
                     field=="velocities" ? s.getVariableVelocity(names[j]) : s.getVariableAcceleration(names[j]));
      }
      std::cout << ']';
    }
    std::cout << '}';
  }
  std::cout << ']';
}
int main(int argc,char** argv) {
  try {
    if(argc!=7) throw std::runtime_error("usage: moveit_timing model.urdf model.srdf path.txt velocity acceleration jerk");
    auto u=urdf::parseURDF(read(argv[1])); if(!u) throw std::runtime_error("Invalid URDF");
    auto semantic=std::make_shared<srdf::Model>();
    if(!semantic->initString(*u,read(argv[2]))) throw std::runtime_error("Invalid SRDF");
    auto model=std::make_shared<moveit::core::RobotModel>(u,semantic);
    auto group=model->getJointModelGroup("manipulator");
    if(!group || group->getVariableCount()!=5) throw std::runtime_error("Expected five-joint manipulator");
    std::ifstream input(argv[3]); size_t n,d; input>>n>>d;
    if(n<2 || n>1000 || d!=5) throw std::runtime_error("Invalid path dimensions");
    std::vector<std::string> names(d); for(auto& name:names) input>>name;
    std::unordered_map<std::string,double> v,a,j;
    double vl=std::stod(argv[4]),al=std::stod(argv[5]),jl=std::stod(argv[6]);
    if(!(vl>0 && al>0 && jl>0)) throw std::runtime_error("Positive limits required");
    for(const auto& name:names) {
      if(!group->hasJointModel(name)) throw std::runtime_error("Unknown joint");
      v[name]=vl; a[name]=al; j[name]=jl;
    }
    robot_trajectory::RobotTrajectory tr(model,"manipulator");
    for(size_t i=0;i<n;++i) {
      moveit::core::RobotState state(model); state.setToDefaultValues();
      for(const auto& name:names) {double q; if(!(input>>q)|| !std::isfinite(q)) throw std::runtime_error("Invalid position"); state.setVariablePosition(name,q);}
      tr.addSuffixWayPoint(state,0.0);
    }
    const auto begin=std::chrono::steady_clock::now();
    trajectory_processing::TimeOptimalTrajectoryGeneration totg(0.001,0.1,0.001);
    if(!totg.computeTimeStamps(tr,v,a)) throw std::runtime_error("TOTG failed");
    const auto timed=std::chrono::steady_clock::now();
    robot_trajectory::RobotTrajectory original(tr, true);
    if(!trajectory_processing::RuckigSmoothing::applySmoothing(tr,v,a,j,1.0,1.0,true,0.001))
      throw std::runtime_error("MoveIt Ruckig smoothing failed");
    const auto end=std::chrono::steady_clock::now();
    std::cout << std::setprecision(17) << "{\"totg_wall_ms\":" << std::chrono::duration<double,std::milli>(timed-begin).count()
      << ",\"ruckig_wall_ms\":" << std::chrono::duration<double,std::milli>(end-timed).count() << ",\"totg\":";
    dump(original,names); std::cout << ",\"ruckig\":"; dump(tr,names); std::cout << "}\n";
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
