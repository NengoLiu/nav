#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <opencv2/opencv.hpp> // 仅用于 onLoadMap 读取 pgm
#include <opennav_coverage_msgs/action/compute_coverage_path.hpp>
#include "auto_construct/msg/pillar.hpp"
#include "auto_construct/msg/coverage_region.hpp"
#include "auto_construct/srv/set_region.hpp"
#include "auto_construct/srv/confirm_region.hpp"
#include "auto_construct/srv/update_params.hpp"
#include "auto_construct/srv/get_map_list.hpp"
#include <nav2_msgs/srv/load_map.hpp>
#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include <future>
#include <memory>

namespace auto_construct {

// ═══════════════════════════════════════════════════════
// GmlGenerator  坐标 → GML 文件
// ═══════════════════════════════════════════════════════
class GmlGenerator {
public:
  struct PillarContour {
    std::vector<geometry_msgs::msg::Point32> points;
  };

  static bool generate(
    const std::string & output_path,
    const std::vector<geometry_msgs::msg::Point32> & outer_boundary,
    const std::vector<PillarContour> & pillars,
    const std::string & field_name = "coverage_field");

  static PillarContour fromPillarMsg(
    const auto_construct::msg::Pillar & pillar);
};

// ═══════════════════════════════════════════════════════
// CoverageManagerNode  无状态 ROS2 节点
// ═══════════════════════════════════════════════════════
class CoverageManagerNode : public rclcpp::Node {
public:
  explicit CoverageManagerNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using ComputeCoverage = opennav_coverage_msgs::action::ComputeCoveragePath;

  // ── 服务回调 ─────────────────────────────────────────
  void onSetRegion(
    const auto_construct::srv::SetRegion::Request::SharedPtr  req,
    const auto_construct::srv::SetRegion::Response::SharedPtr res);

  void onConfirmRegion(
    const auto_construct::srv::ConfirmRegion::Request::SharedPtr  req,
    const auto_construct::srv::ConfirmRegion::Response::SharedPtr res);

  void onUpdateParams(
    const auto_construct::srv::UpdateParams::Request::SharedPtr  req,
    const auto_construct::srv::UpdateParams::Response::SharedPtr res);

  void onLoadMap(
    const nav2_msgs::srv::LoadMap::Request::SharedPtr  req,
    const nav2_msgs::srv::LoadMap::Response::SharedPtr res);

  void onGetMapList(
    const auto_construct::srv::GetMapList::Request::SharedPtr req,
    const auto_construct::srv::GetMapList::Response::SharedPtr res);

  // ── 内部工具 ─────────────────────────────────────────
  bool writeGml(const std::string & path);
  nav_msgs::msg::Path triggerPlanningSync();
  void publishPillarMarkers();
  void saveCachedPath(const nav_msgs::msg::Path & path);
  nav_msgs::msg::Path loadCachedPath();
  bool hasCachedPath() const;

  // ── 状态数据（仅作为当前作业缓存）──────────────────────
  bool map_received_ {false};
  nav_msgs::msg::MapMetaData               map_info_;
  std::string                              current_gml_path_;
  std::string                              current_map_dir_;
  std::string                              gml_dir_;
  std::vector<geometry_msgs::msg::Point32> current_boundary_;
  std::vector<auto_construct::msg::Pillar> current_pillars_;

  // ── ROS 接口 ─────────────────────────────────────────
  rclcpp::CallbackGroup::SharedPtr srv_cbg_;
  rclcpp::CallbackGroup::SharedPtr action_cbg_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr      map_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                  path_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr         map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  rclcpp::Service<auto_construct::srv::SetRegion>::SharedPtr         set_region_srv_;
  rclcpp::Service<auto_construct::srv::ConfirmRegion>::SharedPtr     confirm_region_srv_;
  rclcpp::Service<auto_construct::srv::UpdateParams>::SharedPtr      update_params_srv_;
  rclcpp::Service<nav2_msgs::srv::LoadMap>::SharedPtr                load_map_srv_;
  rclcpp::Service<auto_construct::srv::GetMapList>::SharedPtr        get_map_list_srv_;

  rclcpp_action::Client<ComputeCoverage>::SharedPtr action_client_;
};

} // namespace auto_construct