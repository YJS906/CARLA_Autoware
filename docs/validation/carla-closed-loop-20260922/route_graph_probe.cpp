#include "lanelet2_plugins/default_planner.hpp"
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <rclcpp/rclcpp.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <execinfo.h>
#include <signal.h>
#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <memory>
#include <vector>

struct Planner : autoware::mission_planner_universe::lanelet2::DefaultPlanner {
  void set_map(const lanelet::LaneletMapPtr &map) { route_handler_.setMap(map); }
};

geometry_msgs::msg::Pose pose(double x,double y,double yaw) {
  geometry_msgs::msg::Pose p;
  p.position.x=x; p.position.y=y;
  p.orientation=autoware_utils::create_quaternion_from_yaw(yaw);
  return p;
}

int main(int argc,char **argv) {
  signal(SIGSEGV, [](int){void *frames[64];auto count=backtrace(frames,64);backtrace_symbols_fd(frames,count,2);_exit(139);});
  std::cerr<<std::setprecision(15);
  rclcpp::init(argc,argv);
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args","--params-file","/opt/autoware/sample_vehicle_description/share/sample_vehicle_description/config/vehicle_info.param.yaml","--params-file","/work/src/autoware_mission_planner_universe/config/mission_planner.param.yaml"});
  auto node=std::make_shared<rclcpp::Node>("mission_repro",options);
  Planner planner;
  planner.initialize(node.get());
  lanelet::ErrorMessages errors;
  lanelet::projection::UtmProjector projector(lanelet::Origin({0.0,0.0}));
  const char *map_path=std::getenv("MISSION_MAP_PATH");
  lanelet::LaneletMapPtr map=lanelet::load(map_path ? map_path : "/work/map/lanelet2_map.osm",projector,&errors);
  for(auto point: map->pointLayer) {
    if(point.hasAttribute("local_x")) point.x()=point.attribute("local_x").asDouble().value();
    if(point.hasAttribute("local_y")) point.y()=point.attribute("local_y").asDouble().value();
  }
  lanelet::utils::overwriteLaneletsCenterline(map,5.0,true);
  for(const auto &lane: map->laneletLayer) {
    bool valid=true;
    for(const auto &point:lane.centerline()) if(!std::isfinite(point.x()) || !std::isfinite(point.y()) || !std::isfinite(point.z())) valid=false;
    if(!valid) {
      std::cerr<<"NONFINITE lane="<<lane.id()<<" left="<<lane.leftBound().size()<<" right="<<lane.rightBound().size()<<" center="<<lane.centerline().size()<<std::endl;
    }
  }
  std::cerr<<"Loaded "<<map->laneletLayer.size()<<" lanelets, errors="<<errors.size()<<std::endl;
  planner.set_map(map);
  std::vector<geometry_msgs::msg::Pose> points{pose(-144.1007537841797,.9974544048309326,M_PI),pose(-268.808,-45.1585,-1.52857)};
  const char *scenario=std::getenv("MISSION_CASE");
  if(scenario && std::string(scenario)=="center_goal") {
    const auto line=map->laneletLayer.get(6183).centerline();
    const auto i=line.size()/2;
    points[1]=pose(line[i].x(),line[i].y(),std::atan2(line[i+1].y()-line[i].y(),line[i+1].x()-line[i].x()));
    std::cerr<<"CENTER_GOAL="<<points[1].position.x<<","<<points[1].position.y<<", yaw="<<2*std::atan2(points[1].orientation.z,points[1].orientation.w)<<std::endl;
  } else if(scenario && std::string(scenario)=="fixed_goal") {
    points[1]=pose(-264.260,-80.0957,1.581);
  } else if(scenario && std::string(scenario)=="fixed_corrected") {
    points[1]=pose(-264.260,-80.0957,2.17607);
  } else if(scenario && std::string(scenario)=="straight25") {
    points[1]=pose(-170.498947,.535680,-179.860535*M_PI/180);
  } else if(scenario && std::string(scenario)=="east140") {
    points[0]=pose(210.324,-75.500,1.559196262876784);
    points[1]=pose(210.0991668701172,66.0926742553711,1.582474288174894);
  } else if(scenario && std::string(scenario)=="outside") {
    points[1]=pose(10000,10000,0);
  }
  lanelet::ConstLanelets lane_path;
  lanelet::ConstLaneletOrAreas area_path;
  auto lane_ok=planner.getRouteHandler().planPathLaneletsBetweenCheckpoints(points[0],points[1],&lane_path,false);
  auto area_ok=planner.getRouteHandler().planPathLaneletsBetweenCheckpoints(points[0],points[1],&area_path,false);
  std::cerr<<"LANE_OK="<<lane_ok<<" size="<<lane_path.size()<<" AREA_OK="<<area_ok<<" size="<<area_path.size()<<std::endl;
  const auto result=planner.plan(points);
  std::cerr<<"ROUTE_SEGMENTS="<<result.segments.size()<<std::endl;
  if(scenario && std::string(scenario)=="east140") {
    for(const auto &seg:result.segments) {std::cerr<<"CORRIDOR="; for(const auto &prim:seg.primitives) std::cerr<<prim.id<<","; std::cerr<<std::endl;}
    std::cerr<<"CHANGEABLE="; for(const auto &lane:planner.getRouteHandler().getLaneChangeableNeighbors(map->laneletLayer.get(18052))) std::cerr<<lane.id()<<","; std::cerr<<std::endl;
  }
  rclcpp::shutdown();
  return 0;
}
