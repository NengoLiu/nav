#include "auto_construct/mapping_manager.hpp"

#include <cerrno>
#include <cstring>
#include <signal.h>
#include <sys/wait.h>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

// ─────────────────────────────────────────────────────────────────────────────
// Constructor / Destructor
// ─────────────────────────────────────────────────────────────────────────────

MappingManager::MappingManager() : Node("mapping_manager")
{
  cb_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::Reentrant);

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

  // ── 覆盖路径规划服务 ───────────────────────────────────────────────────────
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

  // ── 客户端 ─────────────────────────────────────────────────────────────────
  client_save_map_ = this->create_client<std_srvs::srv::Trigger>(
    "/mapping/save_map",
    rmw_qos_profile_services_default,
    cb_group_);

  client_set_region_ = this->create_client<auto_construct::srv::SetRegion>(
    "/coverage/set_region",
    rmw_qos_profile_services_default, cb_group_);

  client_confirm_region_ = this->create_client<auto_construct::srv::ConfirmRegion>(
    "/coverage/confirm_region",
    rmw_qos_profile_services_default, cb_group_);

  client_update_params_ = this->create_client<auto_construct::srv::UpdateParams>(
    "/coverage/update_params",
    rmw_qos_profile_services_default, cb_group_);

  client_get_map_list_ = this->create_client<auto_construct::srv::GetMapList>(
    "/coverage/get_map_list",
    rmw_qos_profile_services_default, cb_group_);

  client_set_path_and_start_ = this->create_client<auto_construct::srv::SetPathAndStart>(
    "/coverage/coverage_path/set_path_and_start",
    rmw_qos_profile_services_default, cb_group_);

  RCLCPP_INFO(this->get_logger(), "🟢 MappingManager 已就绪");
}

MappingManager::~MappingManager()
{
  stop_current_process();
}

// ─────────────────────────────────────────────────────────────────────────────
// handle_start_mapping
// ─────────────────────────────────────────────────────────────────────────────

void MappingManager::handle_start_mapping(
  std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    if (current_mode_ == RobotMode::MAPPING) {
      res->success = true;
      res->message = "已在建图模式";
      return;
    }
    if (current_mode_ == RobotMode::TRANSITIONING) {
      res->success = false;
      res->message = "系统正在切换模式，请稍候";
      return;
    }
    current_mode_ = RobotMode::TRANSITIONING;
  }

  stop_current_process();

  bool ok = start_launch_process("mapping_plugin_optimized.launch.py");

  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    if (ok) {
      current_mode_ = RobotMode::MAPPING;
      res->success  = true;
      res->message  = "建图模式已拉起";
      RCLCPP_INFO(this->get_logger(), "🗺️  %s", res->message.c_str());
    } else {
      current_mode_ = RobotMode::IDLE;
      res->success  = false;
      res->message  = "建图 launch 启动失败";
      RCLCPP_ERROR(this->get_logger(), "❌ %s", res->message.c_str());
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// handle_finish_mapping
// ─────────────────────────────────────────────────────────────────────────────

void MappingManager::handle_finish_mapping(
  std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    if (current_mode_ != RobotMode::MAPPING) {
      res->success = false;
      res->message = "错误：当前不在建图模式";
      return;
    }
    current_mode_ = RobotMode::TRANSITIONING;
  }

  RCLCPP_INFO(this->get_logger(), "呼叫 PGO 存图...");

  if (!client_save_map_->wait_for_service(3s)) {
    std::lock_guard<std::mutex> lk(state_mtx_);
    current_mode_ = RobotMode::MAPPING;
    res->success  = false;
    res->message  = "存图失败：未响应";
    RCLCPP_ERROR(this->get_logger(), "❌ %s", res->message.c_str());
    return;
  }

  auto future = client_save_map_->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>());

  constexpr auto kPgoTimeout   = 30s;
  constexpr auto kPollInterval = std::chrono::milliseconds(50);
  auto deadline = std::chrono::steady_clock::now() + kPgoTimeout;

  while (std::chrono::steady_clock::now() < deadline) {
    if (future.wait_for(kPollInterval) == std::future_status::ready) {
      break;
    }
  }

  if (future.wait_for(0s) != std::future_status::ready) {
    std::lock_guard<std::mutex> lk(state_mtx_);
    current_mode_ = RobotMode::MAPPING;
    res->success  = false;
    res->message  = "存图超时（30s），请检查 PGO 节点状态";
    RCLCPP_ERROR(this->get_logger(), "❌ %s", res->message.c_str());
    return;
  }

  auto pgo_res = future.get();
  if (!pgo_res->success) {
    std::lock_guard<std::mutex> lk(state_mtx_);
    current_mode_ = RobotMode::MAPPING;
    res->success  = false;
    res->message  = "PGO 报错：" + pgo_res->message;
    RCLCPP_ERROR(this->get_logger(), "❌ %s", res->message.c_str());
    return;
  }

  RCLCPP_INFO(this->get_logger(), "✅ 地图落盘成功，清理建图进程...");
  stop_current_process();

  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    current_mode_ = RobotMode::IDLE;
  }

  res->success = true;
  res->message = "建图结束，地图已保存";
  RCLCPP_INFO(this->get_logger(), "✅ %s", res->message.c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
// handle_start_navigation
//
// 【改动】移除了原有的 has_saved_map_ 检查。
// 导航与建图相互独立：无需先完成建图存图，可直接切换到导航模式
// （例如使用已有地图或跳过建图阶段）。
// ─────────────────────────────────────────────────────────────────────────────

void MappingManager::handle_start_navigation(
  std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    if (current_mode_ == RobotMode::NAVIGATION) {
      res->success = true;
      res->message = "已在导航模式";
      return;
    }
    if (current_mode_ == RobotMode::TRANSITIONING) {
      res->success = false;
      res->message = "系统正在切换模式，请稍候";
      return;
    }
    current_mode_ = RobotMode::TRANSITIONING;
  }

  stop_current_process();

  bool ok = start_launch_process("nav_plugin_optimized.launch.py");

  if (ok) {
    std::this_thread::sleep_for(std::chrono::seconds(3));

    pid_t coverage_pid = ::fork();
    if (coverage_pid == 0) {
      ::setpgid(0, 0);
      ::execlp("ros2", "ros2", "launch", "auto_construct",
               "convert_map_optimized.launch.py", nullptr);
      ::_exit(EXIT_FAILURE);
    }
    if (coverage_pid > 0) {
      coverage_pid_ = coverage_pid;
      RCLCPP_INFO(this->get_logger(),
        "▶ 启动覆盖路径规划 PID=%d", coverage_pid);
    }
  }

  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    if (ok) {
      current_mode_ = RobotMode::NAVIGATION;
      res->success  = true;
      res->message  = "导航模式已拉起（含覆盖路径规划）";
      RCLCPP_INFO(this->get_logger(), "🧭  %s", res->message.c_str());
    } else {
      current_mode_ = RobotMode::IDLE;
      res->success  = false;
      res->message  = "导航 launch 启动失败";
      RCLCPP_ERROR(this->get_logger(), "❌ %s", res->message.c_str());
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// handle_stop_all
// ─────────────────────────────────────────────────────────────────────────────

void MappingManager::handle_stop_all(
  std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    if (current_mode_ == RobotMode::IDLE) {
      res->success = true;
      res->message = "已是 IDLE 状态";
      return;
    }
    current_mode_ = RobotMode::TRANSITIONING;
  }

  stop_current_process();

  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    current_mode_ = RobotMode::IDLE;
  }

  res->success = true;
  res->message = "已关闭所有任务";
  RCLCPP_WARN(this->get_logger(), "🛑 %s", res->message.c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
// 覆盖路径规划服务
//
// 【改动】以下四个 handler 移除了原有的模式检查（MAPPING 模式拦截）。
// 各功能独立：只要对应子系统服务可用，任何模式下均可调用。
// ─────────────────────────────────────────────────────────────────────────────

void MappingManager::handle_set_region(
  std::shared_ptr<auto_construct::srv::SetRegion::Request>  req,
  std::shared_ptr<auto_construct::srv::SetRegion::Response> res)
{
  if (!client_set_region_->wait_for_service(2s)) {
    res->success = false;
    res->message = "覆盖路径规划服务未就绪，请先启动导航模式";
    return;
  }
  auto future = client_set_region_->async_send_request(req);
  if (future.wait_for(5s) != std::future_status::ready) {
    res->success = false;
    res->message = "设置区域超时";
    return;
  }
  auto r = future.get();
  res->success = r->success;
  res->message = r->message;
  RCLCPP_INFO(this->get_logger(), "📍 %s", res->message.c_str());
}

void MappingManager::handle_confirm_region(
  std::shared_ptr<auto_construct::srv::ConfirmRegion::Request>  req,
  std::shared_ptr<auto_construct::srv::ConfirmRegion::Response> res)
{
  if (!client_confirm_region_->wait_for_service(2s)) {
    res->success = false;
    res->message = "覆盖路径规划服务未就绪，请先启动导航模式";
    return;
  }
  auto future = client_confirm_region_->async_send_request(req);
  if (future.wait_for(30s) != std::future_status::ready) {
    res->success = false;
    res->message = "路径规划超时";
    return;
  }
  auto r = future.get();
  res->success = r->success;
  res->message = r->message;
  res->path    = r->path;
  RCLCPP_INFO(this->get_logger(), "🔀 %s", res->message.c_str());
}

void MappingManager::handle_update_params(
  std::shared_ptr<auto_construct::srv::UpdateParams::Request>  req,
  std::shared_ptr<auto_construct::srv::UpdateParams::Response> res)
{
  if (!client_update_params_->wait_for_service(2s)) {
    res->success = false;
    res->message = "覆盖路径规划服务未就绪，请先启动导航模式";
    return;
  }
  auto future = client_update_params_->async_send_request(req);
  if (future.wait_for(30s) != std::future_status::ready) {
    res->success = false;
    res->message = "参数更新超时";
    return;
  }
  auto r = future.get();
  res->success = r->success;
  res->message = r->message;
  res->path    = r->path;
  RCLCPP_INFO(this->get_logger(), "⚙️  %s", res->message.c_str());
}

void MappingManager::handle_get_map_list(
  std::shared_ptr<auto_construct::srv::GetMapList::Request>  req,
  std::shared_ptr<auto_construct::srv::GetMapList::Response> res)
{
  if (!client_get_map_list_->wait_for_service(2s)) {
    res->success = false;
    res->message = "地图列表服务未就绪，请先启动导航模式";
    return;
  }
  auto future = client_get_map_list_->async_send_request(req);
  if (future.wait_for(5s) != std::future_status::ready) {
    res->success = false;
    res->message = "获取地图列表超时";
    return;
  }
  auto r = future.get();
  res->success     = r->success;
  res->message     = r->message;
  res->map_folders = r->map_folders;
  RCLCPP_INFO(this->get_logger(), "📋 %s", res->message.c_str());
}

// 【改动】移除了原有的 MAPPING 拦截和"必须先进入 NAVIGATION 模式"检查。
// 只要 /coverage/set_path_and_start 服务可用（导航子系统运行中），即可调用。
void MappingManager::handle_set_path_and_start(
  std::shared_ptr<auto_construct::srv::SetPathAndStart::Request>  req,
  std::shared_ptr<auto_construct::srv::SetPathAndStart::Response> res)
{
  if (!client_set_path_and_start_->wait_for_service(2s)) {
    res->success = false;
    res->message = "覆盖路径执行服务未就绪，请先启动导航模式";
    return;
  }
  auto future = client_set_path_and_start_->async_send_request(req);
  if (future.wait_for(30s) != std::future_status::ready) {
    res->success = false;
    res->message = "设置路径超时";
    return;
  }
  auto r = future.get();
  res->success = r->success;
  res->message = r->message;
  RCLCPP_INFO(this->get_logger(), "🚀 %s", res->message.c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
// Process Management
// ─────────────────────────────────────────────────────────────────────────────

bool MappingManager::start_launch_process(const std::string & launch_file)
{
  pid_t pid = ::fork();

  if (pid < 0) {
    RCLCPP_ERROR(this->get_logger(), "fork() 失败: %s", strerror(errno));
    return false;
  }

  if (pid == 0) {
    ::setpgid(0, 0);
    ::execlp("ros2", "ros2", "launch", "auto_construct",
             launch_file.c_str(), nullptr);
    ::_exit(EXIT_FAILURE);
  }

  current_pid_ = pid;
  RCLCPP_INFO(this->get_logger(),
    "▶ 启动 [%s] PID=%d", launch_file.c_str(), pid);
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────

void MappingManager::stop_current_process()
{
  // 先停覆盖路径规划进程
  if (coverage_pid_ > 0) {
    const pid_t cov_pid = coverage_pid_;
    coverage_pid_ = -1;

    RCLCPP_INFO(this->get_logger(),
      "⏹ 终止覆盖路径规划进程组 PGID=%d (SIGINT)", cov_pid);
    ::kill(-cov_pid, SIGINT);

    constexpr int kTimeoutMs  = 5000;
    constexpr int kIntervalMs = 100;
    int elapsed_ms = 0, status = 0;

    while (elapsed_ms < kTimeoutMs) {
      pid_t r = ::waitpid(cov_pid, &status, WNOHANG);
      if (r == cov_pid) {
        RCLCPP_INFO(this->get_logger(), "✅ 覆盖路径规划进程组 %d 已退出", cov_pid);
        break;
      }
      if (r == -1 && errno == ECHILD) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(kIntervalMs));
      elapsed_ms += kIntervalMs;
    }

    if (elapsed_ms >= kTimeoutMs) {
      RCLCPP_WARN(this->get_logger(),
        "⚠ 覆盖路径规划进程组 %d 未响应 SIGINT，发送 SIGKILL", cov_pid);
      ::kill(-cov_pid, SIGKILL);
      ::waitpid(cov_pid, &status, 0);
    }
  }

  // 再停主进程
  if (current_pid_ <= 0) return;

  const pid_t pid = current_pid_;
  current_pid_ = -1;

  RCLCPP_INFO(this->get_logger(), "⏹ 终止进程组 PGID=%d (SIGINT)", pid);
  ::kill(-pid, SIGINT);

  constexpr int kTimeoutMs  = 5000;
  constexpr int kIntervalMs = 100;
  int elapsed_ms = 0, status = 0;

  while (elapsed_ms < kTimeoutMs) {
    pid_t r = ::waitpid(pid, &status, WNOHANG);
    if (r == pid) {
      RCLCPP_INFO(this->get_logger(), "✅ 进程组 %d 已退出", pid);
      return;
    }
    if (r == -1 && errno == ECHILD) return;
    std::this_thread::sleep_for(std::chrono::milliseconds(kIntervalMs));
    elapsed_ms += kIntervalMs;
  }

  RCLCPP_WARN(this->get_logger(),
    "⚠ 进程组 %d 未响应 SIGINT，发送 SIGKILL", pid);
  ::kill(-pid, SIGKILL);
  ::waitpid(pid, &status, 0);
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
