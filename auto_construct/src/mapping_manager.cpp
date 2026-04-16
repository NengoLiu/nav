#include "auto_construct/mapping_manager.hpp"

using namespace std::chrono_literals;

// ─────────────────────────────────────────────────────────────────────────────
// Constructor
// ─────────────────────────────────────────────────────────────────────────────

MappingManager::MappingManager() : Node("mapping_manager")
{
  cb_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::Reentrant);

  auto srv_opt = rcl_service_options_t{};
  (void)srv_opt;

  // ── 对外服务端 ────────────────────────────────────────────────────────────
  srv_start_mapping_ = this->create_service<std_srvs::srv::Trigger>(
    "/sys/start_mapping",
    std::bind(&MappingManager::handle_start_mapping, this,
              std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, cb_group_);

  srv_finish_mapping_ = this->create_service<std_srvs::srv::Trigger>(
    "/sys/finish_mapping",
    std::bind(&MappingManager::handle_finish_mapping, this,
              std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, cb_group_);

  srv_start_navigation_ = this->create_service<std_srvs::srv::Trigger>(
    "/sys/start_navigation",
    std::bind(&MappingManager::handle_start_navigation, this,
              std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, cb_group_);

  srv_stop_all_ = this->create_service<std_srvs::srv::Trigger>(
    "/sys/stop_all",
    std::bind(&MappingManager::handle_stop_all, this,
              std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, cb_group_);

  srv_set_region_ = this->create_service<auto_construct::srv::SetRegion>(
    "/sys/set_region",
    std::bind(&MappingManager::handle_set_region, this,
              std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, cb_group_);

  srv_confirm_region_ = this->create_service<auto_construct::srv::ConfirmRegion>(
    "/sys/confirm_region",
    std::bind(&MappingManager::handle_confirm_region, this,
              std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, cb_group_);

  srv_update_params_ = this->create_service<auto_construct::srv::UpdateParams>(
    "/sys/update_params",
    std::bind(&MappingManager::handle_update_params, this,
              std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, cb_group_);

  srv_get_map_list_ = this->create_service<auto_construct::srv::GetMapList>(
    "/sys/get_map_list",
    std::bind(&MappingManager::handle_get_map_list, this,
              std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, cb_group_);

  srv_set_path_and_start_ = this->create_service<auto_construct::srv::SetPathAndStart>(
    "/sys/set_path_and_start",
    std::bind(&MappingManager::handle_set_path_and_start, this,
              std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, cb_group_);

  // ── 对内客户端 ────────────────────────────────────────────────────────────
  client_save_map_ = this->create_client<std_srvs::srv::Trigger>(
    "/mapping/save_map", rmw_qos_profile_services_default, cb_group_);

  client_set_region_ = this->create_client<auto_construct::srv::SetRegion>(
    "/coverage/set_region", rmw_qos_profile_services_default, cb_group_);

  client_confirm_region_ = this->create_client<auto_construct::srv::ConfirmRegion>(
    "/coverage/confirm_region", rmw_qos_profile_services_default, cb_group_);

  client_update_params_ = this->create_client<auto_construct::srv::UpdateParams>(
    "/coverage/update_params", rmw_qos_profile_services_default, cb_group_);

  client_get_map_list_ = this->create_client<auto_construct::srv::GetMapList>(
    "/coverage/get_map_list", rmw_qos_profile_services_default, cb_group_);

  client_set_path_and_start_ = this->create_client<auto_construct::srv::SetPathAndStart>(
    "/coverage/set_path_and_start", rmw_qos_profile_services_default, cb_group_);

  RCLCPP_INFO(this->get_logger(), "MappingManager ready (all-in-one mode)");
}

// ─────────────────────────────────────────────────────────────────────────────
// 等待服务可用的辅助模板
// ─────────────────────────────────────────────────────────────────────────────

template<typename SrvT>
bool MappingManager::wait_for_service(
  typename rclcpp::Client<SrvT>::SharedPtr & client,
  const std::string & name,
  std::chrono::seconds timeout)
{
  if (!client->wait_for_service(timeout)) {
    RCLCPP_WARN(this->get_logger(), "service not ready: %s", name.c_str());
    return false;
  }
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// handle_start_mapping
//
// 所有建图节点（FastLIO2 / PGO / Octomap）由 all_in_one.launch.py 常驻启动，
// 此处仅作标记，无需再拉起子进程。
// ─────────────────────────────────────────────────────────────────────────────

void MappingManager::handle_start_mapping(
  std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  RCLCPP_INFO(this->get_logger(), "start_mapping: mapping subsystem already running");
  res->success = true;
  res->message = "建图子系统已常驻，可直接开始建图";
}

// ─────────────────────────────────────────────────────────────────────────────
// handle_finish_mapping
//
// 触发 PGO 存图（/mapping/save_map）。SaveMap 节点会依次调用
// PGO 和 Nav2 MapSaver 完成 3D + 2D 地图落盘。
// ─────────────────────────────────────────────────────────────────────────────

void MappingManager::handle_finish_mapping(
  std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  if (!wait_for_service<std_srvs::srv::Trigger>(client_save_map_, "/mapping/save_map", 3s)) {
    res->success = false;
    res->message = "存图失败：/mapping/save_map 服务未就绪";
    return;
  }

  auto future = client_save_map_->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>());

  constexpr auto kTimeout  = 30s;
  constexpr auto kInterval = std::chrono::milliseconds(50);
  auto deadline = std::chrono::steady_clock::now() + kTimeout;

  while (std::chrono::steady_clock::now() < deadline) {
    if (future.wait_for(kInterval) == std::future_status::ready) {
      break;
    }
  }

  if (future.wait_for(0s) != std::future_status::ready) {
    res->success = false;
    res->message = "存图超时（30s），请检查 PGO 节点状态";
    RCLCPP_ERROR(this->get_logger(), "%s", res->message.c_str());
    return;
  }

  auto pgo_res = future.get();
  res->success = pgo_res->success;
  res->message = pgo_res->success
    ? "地图已保存"
    : ("PGO 报错：" + pgo_res->message);

  if (pgo_res->success) {
    RCLCPP_INFO(this->get_logger(), "finish_mapping: map saved");
  } else {
    RCLCPP_ERROR(this->get_logger(), "finish_mapping: %s", res->message.c_str());
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// handle_start_navigation
//
// Nav2 / Localizer 由 all_in_one.launch.py 常驻启动，此处仅作标记。
// ─────────────────────────────────────────────────────────────────────────────

void MappingManager::handle_start_navigation(
  std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  RCLCPP_INFO(this->get_logger(), "start_navigation: nav subsystem already running");
  res->success = true;
  res->message = "导航子系统已常驻，可直接调用 /sys/set_path_and_start 执行路径";
}

// ─────────────────────────────────────────────────────────────────────────────
// handle_stop_all
// ─────────────────────────────────────────────────────────────────────────────

void MappingManager::handle_stop_all(
  std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  RCLCPP_WARN(this->get_logger(), "stop_all called — all nodes remain alive");
  res->success = true;
  res->message = "stop_all 已收到；各子系统节点保持运行，路径执行已请求取消";
}

// ─────────────────────────────────────────────────────────────────────────────
// 覆盖路径规划 service 转发
// ─────────────────────────────────────────────────────────────────────────────

void MappingManager::handle_set_region(
  std::shared_ptr<auto_construct::srv::SetRegion::Request>  req,
  std::shared_ptr<auto_construct::srv::SetRegion::Response> res)
{
  if (!wait_for_service<auto_construct::srv::SetRegion>(
        client_set_region_, "/coverage/set_region")) {
    res->success = false;
    res->message = "覆盖路径规划服务未就绪";
    return;
  }
  auto future = client_set_region_->async_send_request(req);
  if (future.wait_for(5s) != std::future_status::ready) {
    res->success = false;
    res->message = "set_region 超时";
    return;
  }
  auto r = future.get();
  res->success = r->success;
  res->message = r->message;
}

void MappingManager::handle_confirm_region(
  std::shared_ptr<auto_construct::srv::ConfirmRegion::Request>  req,
  std::shared_ptr<auto_construct::srv::ConfirmRegion::Response> res)
{
  if (!wait_for_service<auto_construct::srv::ConfirmRegion>(
        client_confirm_region_, "/coverage/confirm_region")) {
    res->success = false;
    res->message = "覆盖路径规划服务未就绪";
    return;
  }
  auto future = client_confirm_region_->async_send_request(req);
  if (future.wait_for(30s) != std::future_status::ready) {
    res->success = false;
    res->message = "confirm_region 超时";
    return;
  }
  auto r = future.get();
  res->success = r->success;
  res->message = r->message;
  res->path    = r->path;
}

void MappingManager::handle_update_params(
  std::shared_ptr<auto_construct::srv::UpdateParams::Request>  req,
  std::shared_ptr<auto_construct::srv::UpdateParams::Response> res)
{
  if (!wait_for_service<auto_construct::srv::UpdateParams>(
        client_update_params_, "/coverage/update_params")) {
    res->success = false;
    res->message = "覆盖路径规划服务未就绪";
    return;
  }
  auto future = client_update_params_->async_send_request(req);
  if (future.wait_for(30s) != std::future_status::ready) {
    res->success = false;
    res->message = "update_params 超时";
    return;
  }
  auto r = future.get();
  res->success = r->success;
  res->message = r->message;
  res->path    = r->path;
}

void MappingManager::handle_get_map_list(
  std::shared_ptr<auto_construct::srv::GetMapList::Request>  req,
  std::shared_ptr<auto_construct::srv::GetMapList::Response> res)
{
  if (!wait_for_service<auto_construct::srv::GetMapList>(
        client_get_map_list_, "/coverage/get_map_list")) {
    res->success = false;
    res->message = "地图列表服务未就绪";
    return;
  }
  auto future = client_get_map_list_->async_send_request(req);
  if (future.wait_for(5s) != std::future_status::ready) {
    res->success = false;
    res->message = "get_map_list 超时";
    return;
  }
  auto r = future.get();
  res->success     = r->success;
  res->message     = r->message;
  res->map_folders = r->map_folders;
}

void MappingManager::handle_set_path_and_start(
  std::shared_ptr<auto_construct::srv::SetPathAndStart::Request>  req,
  std::shared_ptr<auto_construct::srv::SetPathAndStart::Response> res)
{
  if (!wait_for_service<auto_construct::srv::SetPathAndStart>(
        client_set_path_and_start_, "/coverage/set_path_and_start")) {
    res->success = false;
    res->message = "覆盖路径执行服务未就绪";
    return;
  }
  auto future = client_set_path_and_start_->async_send_request(req);
  if (future.wait_for(5s) != std::future_status::ready) {
    res->success = false;
    res->message = "set_path_and_start 超时";
    return;
  }
  auto r = future.get();
  res->success = r->success;
  res->message = r->message;
}

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MappingManager>();

  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions{}, 4);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
