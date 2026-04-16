#ifndef MAPPING_MANAGER_HPP_
#define MAPPING_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <auto_construct/srv/set_region.hpp>
#include <auto_construct/srv/confirm_region.hpp>
#include <auto_construct/srv/update_params.hpp>
#include <auto_construct/srv/get_map_list.hpp>
#include <auto_construct/srv/set_path_and_start.hpp>

// ─────────────────────────────────────────────────────────────────────────────
// MappingManager
//
// all_in_one.launch.py 启动后，所有子系统节点（FastLIO2、PGO、Octomap、
// Nav2、opennav_coverage、CoveragePath）已常驻运行，各功能相互独立。
//
// 本节点仅作为统一的 /sys/* service 入口，将调用无状态地转发给对应的子系统节点，
// 不做任何模式校验或互斥控制。
// ─────────────────────────────────────────────────────────────────────────────
class MappingManager : public rclcpp::Node
{
public:
  MappingManager();

private:
  // ── 建图生命周期（标记语义，不管理子进程）───────────────────────────────
  void handle_start_mapping(
    std::shared_ptr<std_srvs::srv::Trigger::Request>  req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  void handle_finish_mapping(
    std::shared_ptr<std_srvs::srv::Trigger::Request>  req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  // ── 导航生命周期（标记语义，不管理子进程）───────────────────────────────
  void handle_start_navigation(
    std::shared_ptr<std_srvs::srv::Trigger::Request>  req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  void handle_stop_all(
    std::shared_ptr<std_srvs::srv::Trigger::Request>  req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  // ── 覆盖路径规划（转发至 /coverage/* 子系统）────────────────────────────
  void handle_set_region(
    std::shared_ptr<auto_construct::srv::SetRegion::Request>  req,
    std::shared_ptr<auto_construct::srv::SetRegion::Response> res);

  void handle_confirm_region(
    std::shared_ptr<auto_construct::srv::ConfirmRegion::Request>  req,
    std::shared_ptr<auto_construct::srv::ConfirmRegion::Response> res);

  void handle_update_params(
    std::shared_ptr<auto_construct::srv::UpdateParams::Request>  req,
    std::shared_ptr<auto_construct::srv::UpdateParams::Response> res);

  void handle_get_map_list(
    std::shared_ptr<auto_construct::srv::GetMapList::Request>  req,
    std::shared_ptr<auto_construct::srv::GetMapList::Response> res);

  void handle_set_path_and_start(
    std::shared_ptr<auto_construct::srv::SetPathAndStart::Request>  req,
    std::shared_ptr<auto_construct::srv::SetPathAndStart::Response> res);

  // ── 公共等待+转发模板 ────────────────────────────────────────────────────
  template<typename SrvT>
  bool wait_for_service(
    typename rclcpp::Client<SrvT>::SharedPtr & client,
    const std::string & name,
    std::chrono::seconds timeout = std::chrono::seconds(2));

  // ── ROS 对象 ─────────────────────────────────────────────────────────────
  rclcpp::CallbackGroup::SharedPtr cb_group_;

  // 服务端（对外）
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_start_mapping_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_finish_mapping_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_start_navigation_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_stop_all_;
  rclcpp::Service<auto_construct::srv::SetRegion>::SharedPtr        srv_set_region_;
  rclcpp::Service<auto_construct::srv::ConfirmRegion>::SharedPtr    srv_confirm_region_;
  rclcpp::Service<auto_construct::srv::UpdateParams>::SharedPtr     srv_update_params_;
  rclcpp::Service<auto_construct::srv::GetMapList>::SharedPtr       srv_get_map_list_;
  rclcpp::Service<auto_construct::srv::SetPathAndStart>::SharedPtr  srv_set_path_and_start_;

  // 客户端（对内）
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr                client_save_map_;
  rclcpp::Client<auto_construct::srv::SetRegion>::SharedPtr        client_set_region_;
  rclcpp::Client<auto_construct::srv::ConfirmRegion>::SharedPtr    client_confirm_region_;
  rclcpp::Client<auto_construct::srv::UpdateParams>::SharedPtr     client_update_params_;
  rclcpp::Client<auto_construct::srv::GetMapList>::SharedPtr       client_get_map_list_;
  rclcpp::Client<auto_construct::srv::SetPathAndStart>::SharedPtr  client_set_path_and_start_;
};

#endif  // MAPPING_MANAGER_HPP_
