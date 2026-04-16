#include "auto_construct/convert_map.hpp"

#include <fstream>
#include <iomanip>
#include <filesystem>
#include <stdexcept>
#include <algorithm>
#include <chrono>

namespace auto_construct {

// ═══════════════════════════════════════════════════════
// GmlGenerator
// ═══════════════════════════════════════════════════════
bool GmlGenerator::generate(
  const std::string & output_path,
  const std::vector<geometry_msgs::msg::Point32> & outer_boundary,
  const std::vector<PillarContour> & pillars,
  const std::string & field_name)
{
  if (outer_boundary.size() < 3) return false;
  std::ofstream file(output_path);
  if (!file.is_open()) return false;

  auto writeCoords = [&](const std::vector<geometry_msgs::msg::Point32> & pts, bool close = true) {
    for (size_t i = 0; i < pts.size(); ++i) {
      file << std::fixed << std::setprecision(6) << pts[i].x << "," << pts[i].y;
      if (i + 1 < pts.size()) file << " ";
    }
    if (close && !pts.empty()) { file << " " << pts[0].x << "," << pts[0].y; }
  };

  file << "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n"
       << "<GAOS_parcel xmlns:gml=\"http://www.opengis.net/gml\" id=\"coverage-field\">\n"
       << "  <Field area=\"0\" date=\"2024-01-01\" id=\"" << field_name << "\" lineage=\"Unknown\">\n"
       << "    <geometry>\n"
       << "      <gml:Polygon srsName=\"undefined\">\n"
       << "        <gml:outerBoundaryIs>\n"
       << "          <gml:LinearRing>\n"
       << "            <gml:coordinates>";
  writeCoords(outer_boundary);
  file << "</gml:coordinates>\n          </gml:LinearRing>\n        </gml:outerBoundaryIs>\n";

  for (auto & pillar : pillars) {
    if (pillar.points.size() < 3) continue;
    file << "        <gml:innerBoundaryIs>\n"
         << "          <gml:LinearRing>\n"
         << "            <gml:coordinates>";
    writeCoords(pillar.points);
    file << "</gml:coordinates>\n          </gml:LinearRing>\n        </gml:innerBoundaryIs>\n";
  }
  file << "      </gml:Polygon>\n    </geometry>\n  </Field>\n</GAOS_parcel>\n";
  return true;
}

GmlGenerator::PillarContour GmlGenerator::fromPillarMsg(const auto_construct::msg::Pillar & pillar)
{
  float hx = static_cast<float>(pillar.width / 2.0);
  float hy = static_cast<float>(pillar.height / 2.0);
  float cx = static_cast<float>(pillar.cx);
  float cy = static_cast<float>(pillar.cy);
  auto pt = [](float x, float y) { geometry_msgs::msg::Point32 p; p.x = x; p.y = y; p.z = 0; return p; };
  return PillarContour{{ pt(cx-hx, cy-hy), pt(cx-hx, cy+hy), pt(cx+hx, cy+hy), pt(cx+hx, cy-hy) }};
}

// ═══════════════════════════════════════════════════════
// CoverageManagerNode
// ═══════════════════════════════════════════════════════
CoverageManagerNode::CoverageManagerNode(const rclcpp::NodeOptions & options)
: Node("coverage_manager", options)
{
  declare_parameter("gml_output_dir", "/tmp/coverage_gml");
  declare_parameter("robot_width",    0.5);
  declare_parameter("headland_width", 0.5);
  declare_parameter("route_type",     std::string("BOUSTROPHEDON"));
  declare_parameter("curve_type",     std::string("DUBIN"));
  declare_parameter("swath_angle",    -1.0);
  declare_parameter("map_base_dir",   "/home/nic/ROS/ROS/map/maps");

  gml_dir_ = get_parameter("gml_output_dir").as_string();
  std::filesystem::create_directories(gml_dir_);

  srv_cbg_    = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  action_cbg_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = action_cbg_;
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", rclcpp::QoS(1).transient_local(),
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
      map_info_ = msg->info; map_received_ = true;
    }, sub_opts);

  path_pub_   = create_publisher<nav_msgs::msg::Path>("/coverage_result_path", rclcpp::QoS(1).transient_local());
  map_pub_    = create_publisher<nav_msgs::msg::OccupancyGrid>("coverage_map/pcd2pgm_map", rclcpp::QoS(1).transient_local());
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/coverage/detected_pillars", rclcpp::QoS(1).transient_local());

  set_region_srv_ = create_service<auto_construct::srv::SetRegion>(
    "/coverage/set_region", std::bind(&CoverageManagerNode::onSetRegion, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, srv_cbg_);

  confirm_region_srv_ = create_service<auto_construct::srv::ConfirmRegion>(
    "/coverage/confirm_region", std::bind(&CoverageManagerNode::onConfirmRegion, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, srv_cbg_);

  update_params_srv_ = create_service<auto_construct::srv::UpdateParams>(
    "/coverage/update_params", std::bind(&CoverageManagerNode::onUpdateParams, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, srv_cbg_);

  load_map_srv_ = create_service<nav2_msgs::srv::LoadMap>(
    "/coverage/load_map", std::bind(&CoverageManagerNode::onLoadMap, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, srv_cbg_);

  get_map_list_srv_ = create_service<auto_construct::srv::GetMapList>(
    "/coverage/get_map_list", std::bind(&CoverageManagerNode::onGetMapList, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, srv_cbg_);

  action_client_ = rclcpp_action::create_client<ComputeCoverage>(this, "compute_coverage_path", action_cbg_);
  RCLCPP_INFO(get_logger(), "🚀 CoverageManager(纯 API 版) 已启动");
}

// 接收手机端发送的完整区域数据并覆盖本地
void CoverageManagerNode::onSetRegion(
  const auto_construct::srv::SetRegion::Request::SharedPtr  req,
  const auto_construct::srv::SetRegion::Response::SharedPtr res)
{
  if (!map_received_) {
    res->success = false; res->message = "地图尚未接收"; return;
  }

  // 完全信任手机端，直接覆盖（无状态设计核心）
  current_boundary_.assign(req->region.outer_boundary.begin(), req->region.outer_boundary.end());
  current_pillars_.assign(req->region.pillars.begin(), req->region.pillars.end());
  
  publishPillarMarkers(); // 更新可视化

  res->success = true;
  res->message = "数据更新成功，共 " + std::to_string(current_pillars_.size()) + " 个障碍物";
}

// 触发路径规划
void CoverageManagerNode::onConfirmRegion(
  const auto_construct::srv::ConfirmRegion::Request::SharedPtr  /*req*/,
  const auto_construct::srv::ConfirmRegion::Response::SharedPtr res)
{
  if (current_boundary_.empty()) {
    res->success = false; res->message = "未收到有效边界数据"; return;
  }

  current_gml_path_ = current_map_dir_ + "/field.gml";
  if (!writeGml(current_gml_path_)) {
    res->success = false; res->message = "GML 生成失败"; return;
  }

  try {
    nav_msgs::msg::Path path = triggerPlanningSync();
    res->success = true; res->message = "规划完成"; res->path = path;
  } catch (const std::exception & e) {
    res->success = false; res->message = std::string("规划失败: ") + e.what();
  }
}

// 更新参数并重新触发规划
void CoverageManagerNode::onUpdateParams(
  const auto_construct::srv::UpdateParams::Request::SharedPtr  req,
  const auto_construct::srv::UpdateParams::Response::SharedPtr res)
{
  if (req->robot_width > 0)    set_parameter({"robot_width",    req->robot_width});
  if (req->headland_width > 0) set_parameter({"headland_width", req->headland_width});
  if (!req->route_type.empty())set_parameter({"route_type",     req->route_type});
  if (!req->curve_type.empty())set_parameter({"curve_type",     req->curve_type});
  if (req->swath_angle >= 0)   set_parameter({"swath_angle",    req->swath_angle});

  if (current_boundary_.empty()) {
    res->success = true; res->message = "参数已保存"; return; // 如果还没传区域，光存参数即可
  }

  try {
    nav_msgs::msg::Path path = triggerPlanningSync();
    res->success = true; res->message = "重新规划完成"; res->path = path;
  } catch (const std::exception & e) {
    res->success = false; res->message = std::string("重新规划失败: ") + e.what();
  }
}

// 同步触发规划
nav_msgs::msg::Path CoverageManagerNode::triggerPlanningSync()
{
  if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
    throw std::runtime_error("Action Server 未就绪");
  }

  auto goal = ComputeCoverage::Goal();
  goal.use_gml_file = true; goal.gml_field = current_gml_path_;
  goal.generate_headland = true; goal.generate_route = true; goal.generate_path = true;

  auto promise_ptr = std::make_shared<std::promise<nav_msgs::msg::Path>>();
  auto future = promise_ptr->get_future();

  auto opts = rclcpp_action::Client<ComputeCoverage>::SendGoalOptions();
  opts.result_callback =
    [this, promise_ptr](const rclcpp_action::ClientGoalHandle<ComputeCoverage>::WrappedResult & result) {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        path_pub_->publish(result.result->nav_path);
        saveCachedPath(result.result->nav_path);
        promise_ptr->set_value(result.result->nav_path);
      } else {
        promise_ptr->set_exception(std::make_exception_ptr(std::runtime_error("规划失败")));
      }
    };

  action_client_->async_send_goal(goal, opts);
  if (future.wait_for(std::chrono::seconds(30)) != std::future_status::ready) {
    throw std::runtime_error("规划超时"); 
  }
  return future.get();
}

bool CoverageManagerNode::writeGml(const std::string & path)
{
  std::vector<GmlGenerator::PillarContour> contours;
  for (auto & p : current_pillars_) contours.push_back(GmlGenerator::fromPillarMsg(p));
  return GmlGenerator::generate(path, current_boundary_, contours);
}

void CoverageManagerNode::publishPillarMarkers()
{
  visualization_msgs::msg::MarkerArray arr;
  visualization_msgs::msg::Marker del;
  del.action = visualization_msgs::msg::Marker::DELETEALL;
  arr.markers.push_back(del);

  int id = 0;
  for (auto & p : current_pillars_) {
    visualization_msgs::msg::Marker m;
    m.header.stamp = now(); m.header.frame_id = "map"; m.ns = "pillars"; m.id = id++;
    m.type = visualization_msgs::msg::Marker::CUBE; m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = p.cx; m.pose.position.y = p.cy; m.pose.position.z = 0.0; m.pose.orientation.w = 1.0;
    m.scale.x = p.width > 0 ? p.width : 0.3; m.scale.y = p.height > 0 ? p.height : 0.3; m.scale.z = 0.1;
    m.color.r = 1.0f; m.color.g = 0.4f; m.color.b = 0.0f; m.color.a = 0.8f;
    arr.markers.push_back(m);
  }

  if (!current_boundary_.empty()) {
    visualization_msgs::msg::Marker boundary;
    boundary.header.stamp = now(); boundary.header.frame_id = "map"; boundary.ns = "boundary"; boundary.id = 9999;
    boundary.type = visualization_msgs::msg::Marker::LINE_STRIP; boundary.action = visualization_msgs::msg::Marker::ADD;
    boundary.scale.x = 0.05; boundary.color.g = 0.8f; boundary.color.b = 1.0f; boundary.color.a = 1.0f;
    boundary.pose.orientation.w = 1.0;
    for (auto & p : current_boundary_) {
      geometry_msgs::msg::Point pt; pt.x = p.x; pt.y = p.y; pt.z = 0.05; boundary.points.push_back(pt);
    }
    geometry_msgs::msg::Point first; first.x = current_boundary_[0].x; first.y = current_boundary_[0].y; first.z = 0.05;
    boundary.points.push_back(first);
    arr.markers.push_back(boundary);
  }
  marker_pub_->publish(arr);
}

// 保持原有的 LoadMap 和 GetMapList 逻辑...
void CoverageManagerNode::onLoadMap(
  const nav2_msgs::srv::LoadMap::Request::SharedPtr  req,
  const nav2_msgs::srv::LoadMap::Response::SharedPtr res)
{

  RCLCPP_INFO(this->get_logger(), " Load map service");


  // 读取 YAML & PGM
  const std::string & yaml_path = req->map_url;
  if (yaml_path.empty() || !std::filesystem::exists(yaml_path)) {
    res->result = nav2_msgs::srv::LoadMap::Response::RESULT_MAP_DOES_NOT_EXIST; return;
  }
  YAML::Node kv;
  try { kv = YAML::LoadFile(yaml_path); } 
  catch (...) { res->result = nav2_msgs::srv::LoadMap::Response::RESULT_INVALID_MAP_METADATA; return; }

  const double resolution  = kv["resolution"] ? kv["resolution"].as<double>() : 0.05;
  const int    negate      = kv["negate"] ? kv["negate"].as<int>() : 0;
  const double occ_thresh  = kv["occupied_thresh"] ? kv["occupied_thresh"].as<double>() : 0.65;
  const double free_thresh = kv["free_thresh"] ? kv["free_thresh"].as<double>() : 0.196;

  double origin_x = 0.0, origin_y = 0.0;
  if (kv["origin"] && kv["origin"].IsSequence() && kv["origin"].size() >= 2) {
    origin_x = kv["origin"][0].as<double>(); origin_y = kv["origin"][1].as<double>();
  }

  const std::string pgm_path = std::filesystem::path(yaml_path).replace_extension(".pgm").string();
  cv::Mat img = cv::imread(pgm_path, cv::IMREAD_GRAYSCALE);
  if (img.empty()) { res->result = nav2_msgs::srv::LoadMap::Response::RESULT_INVALID_MAP_DATA; return; }

  const int W = img.cols, H = img.rows;
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.stamp = now(); grid.header.frame_id = "map";
  grid.info.resolution = static_cast<float>(resolution); grid.info.width = W; grid.info.height = H;
  grid.info.origin.position.x = origin_x; grid.info.origin.position.y = origin_y; grid.info.origin.orientation.w = 1.0;
  grid.data.resize(static_cast<size_t>(W * H));

  for (int row = 0; row < H; ++row) {
    for (int col = 0; col < W; ++col) {
      const uint8_t pixel = img.at<uint8_t>(row, col);
      const double  prob  = negate ? (pixel / 255.0) : ((255 - pixel) / 255.0);
      grid.data[static_cast<size_t>((H - 1 - row) * W + col)] = (prob > occ_thresh) ? 100 : (prob < free_thresh ? 0 : -1);
    }
  }

  map_pub_->publish(grid);
  map_info_ = grid.info; map_received_ = true;
  res->map = grid; 
  res->result = nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS;
  
  RCLCPP_INFO(this->get_logger(), " Load map service success");

  current_boundary_.clear(); current_pillars_.clear(); current_gml_path_ = "";
  current_map_dir_ = std::filesystem::path(req->map_url).parent_path().string();

  if (hasCachedPath()) { try { loadCachedPath(); } catch (...) {} }
}

void CoverageManagerNode::onGetMapList(
  const auto_construct::srv::GetMapList::Request::SharedPtr,
  const auto_construct::srv::GetMapList::Response::SharedPtr res)
{

  RCLCPP_INFO(this->get_logger(), " get map list service");


  std::string base_dir = get_parameter("map_base_dir").as_string();
  
  if (!std::filesystem::exists(base_dir)) { res->success = false; return; }
  std::vector<std::string> folders;
  try {
    for (const auto & entry : std::filesystem::directory_iterator(base_dir)) {
      if (entry.is_directory() && entry.path().filename().string().find("map_") == 0) {
        folders.push_back(entry.path().filename().string());
      }
    }
    std::sort(folders.rbegin(), folders.rend());
    res->success = true; res->map_folders = folders;
  } catch (...) { res->success = false; }
}

// 缓存相关保持不变
void CoverageManagerNode::saveCachedPath(const nav_msgs::msg::Path & path) {
  if (current_map_dir_.empty()) return;
  YAML::Emitter out;
  out << YAML::BeginMap << YAML::Key << "boundary" << YAML::Value << YAML::BeginSeq;
  for (auto & p : current_boundary_) out << YAML::BeginMap << YAML::Key << "x" << YAML::Value << p.x << YAML::Key << "y" << YAML::Value << p.y << YAML::EndMap;
  out << YAML::EndSeq << YAML::Key << "pillars" << YAML::Value << YAML::BeginSeq;
  for (auto & p : current_pillars_) out << YAML::BeginMap << YAML::Key << "id" << YAML::Value << p.id << YAML::Key << "cx" << YAML::Value << p.cx << YAML::Key << "cy" << YAML::Value << p.cy << YAML::Key << "width" << YAML::Value << p.width << YAML::Key << "height" << YAML::Value << p.height << YAML::EndMap;
  out << YAML::EndSeq << YAML::Key << "path" << YAML::Value << YAML::BeginMap << YAML::Key << "frame_id" << YAML::Value << path.header.frame_id << YAML::Key << "poses" << YAML::Value << YAML::BeginSeq;
  for (auto & ps : path.poses) out << YAML::BeginMap << YAML::Key << "position" << YAML::Value << YAML::BeginMap << YAML::Key << "x" << YAML::Value << ps.pose.position.x << YAML::Key << "y" << YAML::Value << ps.pose.position.y << YAML::Key << "z" << YAML::Value << ps.pose.position.z << YAML::EndMap << YAML::Key << "orientation" << YAML::Value << YAML::BeginMap << YAML::Key << "x" << YAML::Value << ps.pose.orientation.x << YAML::Key << "y" << YAML::Value << ps.pose.orientation.y << YAML::Key << "z" << YAML::Value << ps.pose.orientation.z << YAML::Key << "w" << YAML::Value << ps.pose.orientation.w << YAML::EndMap << YAML::EndMap;
  out << YAML::EndSeq << YAML::EndMap << YAML::EndMap;
  std::ofstream(current_map_dir_ + "/coverage_path_cache.yaml") << out.c_str();
}

bool CoverageManagerNode::hasCachedPath() const {
  return !current_map_dir_.empty() && std::filesystem::exists(current_map_dir_ + "/coverage_path_cache.yaml");
}

nav_msgs::msg::Path CoverageManagerNode::loadCachedPath() {
  YAML::Node root = YAML::LoadFile(current_map_dir_ + "/coverage_path_cache.yaml");
  current_boundary_.clear();
  for (const auto & n : root["boundary"]) {
      geometry_msgs::msg::Point32 p;
      p.x = n["x"].as<float>();
      p.y = n["y"].as<float>();
      p.z = 0.0f;
      current_boundary_.push_back(p);
  }
  current_pillars_.clear();
  for (const auto & n : root["pillars"]) {
    auto_construct::msg::Pillar p; p.id = n["id"].as<std::string>(); p.cx = n["cx"].as<float>(); p.cy = n["cy"].as<float>(); p.width = n["width"].as<float>(); p.height = n["height"].as<float>();
    current_pillars_.push_back(p);
  }
  nav_msgs::msg::Path path; path.header.frame_id = root["path"]["frame_id"].as<std::string>("map"); path.header.stamp = now();
  for (const auto & n : root["path"]["poses"]) {
    geometry_msgs::msg::PoseStamped ps; ps.header = path.header;
    ps.pose.position.x = n["position"]["x"].as<double>(); ps.pose.position.y = n["position"]["y"].as<double>(); ps.pose.position.z = n["position"]["z"].as<double>();
    ps.pose.orientation.x = n["orientation"]["x"].as<double>(); ps.pose.orientation.y = n["orientation"]["y"].as<double>(); ps.pose.orientation.z = n["orientation"]["z"].as<double>(); ps.pose.orientation.w = n["orientation"]["w"].as<double>();
    path.poses.push_back(ps);
  }
  current_gml_path_ = current_map_dir_ + "/field.gml";
  path_pub_->publish(path); publishPillarMarkers();
  return path;
}

} // namespace auto_construct

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<auto_construct::CoverageManagerNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node); executor.spin();
  rclcpp::shutdown(); return 0;
}