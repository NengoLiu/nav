#ifndef MAPPING_MANAGER_HPP_
#define MAPPING_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <auto_construct/srv/set_region.hpp>
#include <auto_construct/srv/confirm_region.hpp>
#include <auto_construct/srv/update_params.hpp>
#include <auto_construct/srv/get_map_list.hpp>
#include <auto_construct/srv/set_path_and_start.hpp>
#include <nav2_msgs/srv/load_map.hpp>

#include <atomic>
#include <mutex>
#include <sys/types.h>
#include <unistd.h>

// ─────────────────────────────────────────────────────────────────────────────
enum class RobotMode { IDLE, MAPPING, NAVIGATION, TRANSITIONING };

inline const char * mode_str(RobotMode m)
{
  switch (m) {
    case RobotMode::IDLE:          return "IDLE";
    case RobotMode::MAPPING:       return "MAPPING";
    case RobotMode::NAVIGATION:    return "NAVIGATION";
    case RobotMode::TRANSITIONING: return "TRANSITIONING";
  }
  return "UNKNOWN";
}

// ─────────────────────────────────────────────────────────────────────────────
class MappingManager : public rclcpp::Node
{
public:
  MappingManager();
  ~MappingManager() override;

private:
  // ── Service 回调 ─────────────────────────────────────────────────────────
  void handle_start_mapping(
    std::shared_ptr<std_srvs::srv::Trigger::Request>  req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  void handle_finish_mapping(
    std::shared_ptr<std_srvs::srv::Trigger::Request>  req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  void handle_start_navigation(
    std::shared_ptr<std_srvs::srv::Trigger::Request>  req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  void handle_stop_all(
    std::shared_ptr<std_srvs::srv::Trigger::Request>  req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  // ── 覆盖路径规划服务回调 ────────────────────────────────────────────────
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

  // ── 进程管理 ─────────────────────────────────────────────────────────────
  bool start_launch_process(const std::string & launch_file);
  void stop_current_process();

  // ── 状态 ─────────────────────────────────────────────────────────────────
  // ⚠️ 原版三个裸变量在 Reentrant + MultiThreadedExecutor 下存在数据竞争
  //    用 mutex 统一保护，并增加 TRANSITIONING 防止并发切换
  std::mutex          state_mtx_;
  RobotMode           current_mode_  {RobotMode::IDLE};
  bool                has_saved_map_ {false};
  pid_t               current_pid_   {-1};
  pid_t               coverage_pid_  {-1};

  // ── ROS 对象 ─────────────────────────────────────────────────────────────
  rclcpp::CallbackGroup::SharedPtr cb_group_;

  // 系统模式管理服务
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_start_mapping_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_finish_mapping_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_start_navigation_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_stop_all_;

  // 覆盖路径规划服务
  rclcpp::Service<auto_construct::srv::SetRegion>::SharedPtr srv_set_region_;
  rclcpp::Service<auto_construct::srv::ConfirmRegion>::SharedPtr srv_confirm_region_;
  rclcpp::Service<auto_construct::srv::UpdateParams>::SharedPtr srv_update_params_;
  rclcpp::Service<auto_construct::srv::GetMapList>::SharedPtr srv_get_map_list_;
  rclcpp::Service<auto_construct::srv::SetPathAndStart>::SharedPtr srv_set_path_and_start_;

  // 客户端
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr  client_save_map_;
  rclcpp::Client<auto_construct::srv::SetRegion>::SharedPtr client_set_region_;
  rclcpp::Client<auto_construct::srv::ConfirmRegion>::SharedPtr client_confirm_region_;
  rclcpp::Client<auto_construct::srv::UpdateParams>::SharedPtr client_update_params_;
  rclcpp::Client<auto_construct::srv::GetMapList>::SharedPtr client_get_map_list_;
  rclcpp::Client<auto_construct::srv::SetPathAndStart>::SharedPtr client_set_path_and_start_;
};

#endif  // MAPPING_MANAGER_HPP_