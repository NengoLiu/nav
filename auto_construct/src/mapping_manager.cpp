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
  // Reentrant 回调组：允许 finish_mapping 在等待 PGO future 时
  // executor 仍能调度其他回调（含 client 的响应回调），避免死锁
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
    "/coverage/set_path_and_start",
    rmw_qos_profile_services_default, cb_group_);



  // client 同样挂在 Reentrant 回调组，响应回调才能被 executor 调度
  client_save_map_ = this->create_client<std_srvs::srv::Trigger>(
    "/mapping/save_map",
    rmw_qos_profile_services_default,
    cb_group_);

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
  // ── 阶段 1：持锁检查并切换为 TRANSITIONING ───────────────────────────────
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
    current_mode_  = RobotMode::TRANSITIONING;
    has_saved_map_ = false;
  }

  // ── 阶段 2：释放锁后执行阻塞操作（stop + fork）───────────────────────────
  stop_current_process();

  bool ok = start_launch_process("mapping_plugin.launch.py");

  // ── 阶段 3：更新最终状态 ─────────────────────────────────────────────────
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
  // ── 阶段 1：持锁检查并切换为 TRANSITIONING ───────────────────────────────
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    if (current_mode_ != RobotMode::MAPPING) {
      res->success = false;
      res->message = "错误：当前不在建图模式";
      return;
    }
    current_mode_ = RobotMode::TRANSITIONING;
  }

  // ── 阶段 2：等待 PGO 服务上线（释放锁，不阻塞其他 service）───────────────
  RCLCPP_INFO(this->get_logger(), "呼叫 PGO 存图...");

  if (!client_save_map_->wait_for_service(3s)) {
    std::lock_guard<std::mutex> lk(state_mtx_);
    current_mode_ = RobotMode::MAPPING;
    res->success  = false;
    res->message  = "存图失败：未响应";
    RCLCPP_ERROR(this->get_logger(), "❌ %s", res->message.c_str());
    return;
  }

  // ── 阶段 3：发起异步请求，用 future.wait_for 轮询等待 ────────────────────
  // ✅ 正确做法：不调用 spin_until_future_complete(this, ...)
  //    而是直接 wait_for，让 MultiThreadedExecutor 的其他线程处理响应回调
  auto future = client_save_map_->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>());

  constexpr auto kPgoTimeout  = 30s;
  constexpr auto kPollInterval = std::chrono::milliseconds(50);
  auto deadline = std::chrono::steady_clock::now() + kPgoTimeout;

  while (std::chrono::steady_clock::now() < deadline) {
    if (future.wait_for(kPollInterval) == std::future_status::ready) {
      break;
    }
    // 还未就绪，继续等待（executor 其他线程会处理 client 响应回调）
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

  // ── 阶段 4：存图成功，停止建图进程 ──────────────────────────────────────
  RCLCPP_INFO(this->get_logger(), "✅ 地图落盘成功，清理建图进程...");
  stop_current_process();

  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    has_saved_map_ = true;
    current_mode_  = RobotMode::IDLE;
  }

  res->success = true;
  res->message = "建图结束，地图已保存";
  RCLCPP_INFO(this->get_logger(), "✅ %s", res->message.c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
// handle_start_navigation
// ─────────────────────────────────────────────────────────────────────────────

void MappingManager::handle_start_navigation(
  std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  // ── 阶段 1：持锁检查并切换为 TRANSITIONING ───────────────────────────────
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    if (!has_saved_map_) {
      res->success = false;
      res->message = "拒绝：必须先完成建图存图才能导航";
      return;
    }
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

  // ── 阶段 2：释放锁后执行阻塞操作 ────────────────────────────────────────
  stop_current_process();

  // 启动导航模式（包括 Localizer 和 Nav2）
  bool ok = start_launch_process("nav_plugin_optimized.launch.py");

  if (ok) {
    // 等待导航系统启动
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // 启动覆盖路径规划节点
    pid_t coverage_pid = ::fork();
    if (coverage_pid == 0) {
      // 子进程：启动覆盖路径规划
      ::setpgid(0, 0);
      ::execlp("ros2", "ros2", "launch", "auto_construct",
               "convert_map_optimized.launch.py", nullptr);
      ::_exit(EXIT_FAILURE);
    }

    if (coverage_pid > 0) {
      // 父进程：记录覆盖路径规划进程 PID
      coverage_pid_ = coverage_pid;
      RCLCPP_INFO(this->get_logger(),
        "▶ 启动覆盖路径规划 PID=%d", coverage_pid);
    }
  }

  // ── 阶段 3：更新最终状态 ─────────────────────────────────────────────────
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
  // ── 阶段 1：持锁检查并切换为 TRANSITIONING ───────────────────────────────
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    if (current_mode_ == RobotMode::IDLE) {
      res->success = true;
      res->message = "已是 IDLE 状态";
      return;
    }
    current_mode_ = RobotMode::TRANSITIONING;
  }

  // ── 阶段 2：释放锁后停止进程（内部有阻塞等待，不能持锁）─────────────────
  stop_current_process();

  // ── 阶段 3：更新状态 ─────────────────────────────────────────────────────
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    current_mode_ = RobotMode::IDLE;
  }

  res->success = true;
  res->message = "已紧急关闭所有任务";
  RCLCPP_WARN(this->get_logger(), "🛑 %s", res->message.c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
// 覆盖路径规划服务实现
// ─────────────────────────────────────────────────────────────────────────────

void MappingManager::handle_set_region(
  std::shared_ptr<auto_construct::srv::SetRegion::Request>  req,
  std::shared_ptr<auto_construct::srv::SetRegion::Response> res)
{
  // 检查当前模式
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    if (current_mode_ == RobotMode::MAPPING) {
      res->success = false;
      res->message = "建图模式下不能设置覆盖区域";
      return;
    }
  }

  // 检查服务是否可用
  if (!client_set_region_->wait_for_service(2s)) {
    res->success = false;
    res->message = "覆盖路径规划服务未就绪，请先启动导航模式";
    return;
  }

  // 转发请求到 coverage_manager
  auto future = client_set_region_->async_send_request(req);
  if (future.wait_for(5s) != std::future_status::ready) {
    res->success = false;
    res->message = "设置区域超时";
    return;
  }

  auto response = future.get();
  res->success = response->success;
  res->message = response->message;
  RCLCPP_INFO(this->get_logger(), "📍 %s", res->message.c_str());
}

void MappingManager::handle_confirm_region(
  std::shared_ptr<auto_construct::srv::ConfirmRegion::Request>  req,
  std::shared_ptr<auto_construct::srv::ConfirmRegion::Response> res)
{
  // 检查当前模式
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    if (current_mode_ == RobotMode::MAPPING) {
      res->success = false;
      res->message = "建图模式下不能规划覆盖路径";
      return;
    }
  }

  // 检查服务是否可用
  if (!client_confirm_region_->wait_for_service(2s)) {
    res->success = false;
    res->message = "覆盖路径规划服务未就绪，请先启动导航模式";
    return;
  }

  // 转发请求到 coverage_manager
  auto future = client_confirm_region_->async_send_request(req);
  if (future.wait_for(30s) != std::future_status::ready) {
    res->success = false;
    res->message = "路径规划超时";
    return;
  }

  auto response = future.get();
  res->success = response->success;
  res->message = response->message;
  res->path = response->path;
  RCLCPP_INFO(this->get_logger(), "🔀 %s", res->message.c_str());
}

void MappingManager::handle_update_params(
  std::shared_ptr<auto_construct::srv::UpdateParams::Request>  req,
  std::shared_ptr<auto_construct::srv::UpdateParams::Response> res)
{
  // 检查当前模式
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    if (current_mode_ == RobotMode::MAPPING) {
      res->success = false;
      res->message = "建图模式下不能更新路径规划参数";
      return;
    }
  }

  // 检查服务是否可用
  if (!client_update_params_->wait_for_service(2s)) {
    res->success = false;
    res->message = "覆盖路径规划服务未就绪，请先启动导航模式";
    return;
  }

  // 转发请求到 coverage_manager
  auto future = client_update_params_->async_send_request(req);
  if (future.wait_for(30s) != std::future_status::ready) {
    res->success = false;
    res->message = "参数更新超时";
    return;
  }

  auto response = future.get();
  res->success = response->success;
  res->message = response->message;
  res->path = response->path;
  RCLCPP_INFO(this->get_logger(), "⚙️  %s", res->message.c_str());
}

void MappingManager::handle_get_map_list(
  std::shared_ptr<auto_construct::srv::GetMapList::Request>  req,
  std::shared_ptr<auto_construct::srv::GetMapList::Response> res)
{
  // 检查服务是否可用
  if (!client_get_map_list_->wait_for_service(2s)) {
    res->success = false;
    res->message = "地图列表服务未就绪，请先启动导航模式";
    return;
  }

  // 转发请求到 coverage_manager
  auto future = client_get_map_list_->async_send_request(req);
  if (future.wait_for(5s) != std::future_status::ready) {
    res->success = false;
    res->message = "获取地图列表超时";
    return;
  }

  auto response = future.get();
  res->success = response->success;
  res->message = response->message;
  res->map_folders = response->map_folders;
  RCLCPP_INFO(this->get_logger(), "📋 %s", res->message.c_str());
}

void MappingManager::handle_set_path_and_start(
  std::shared_ptr<auto_construct::srv::SetPathAndStart::Request>  req,
  std::shared_ptr<auto_construct::srv::SetPathAndStart::Response> res)
{
  // 检查当前模式
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    if (current_mode_ == RobotMode::MAPPING) {
      res->success = false;
      res->message = "建图模式下不能执行覆盖路径";
      return;
    }
    if (current_mode_ != RobotMode::NAVIGATION) {
      res->success = false;
      res->message = "请先启动导航模式";
      return;
    }
  }

  // 检查服务是否可用
  if (!client_set_path_and_start_->wait_for_service(2s)) {
    res->success = false;
    res->message = "覆盖路径执行服务未就绪";
    return;
  }

  // 转发请求到 coverage_path
  auto future = client_set_path_and_start_->async_send_request(req);
  if (future.wait_for(5s) != std::future_status::ready) {
    res->success = false;
    res->message = "设置路径超时";
    return;
  }

  auto response = future.get();
  res->success = response->success;
  res->message = response->message;
  RCLCPP_INFO(this->get_logger(), "🚀 %s", res->message.c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
// Process Management
// ─────────────────────────────────────────────────────────────────────────────

bool MappingManager::start_launch_process(const std::string & launch_file)
{
  // ⚠️ 调用此函数前必须已释放 state_mtx_（内部无锁）
  pid_t pid = ::fork();

  if (pid < 0) {
    RCLCPP_ERROR(this->get_logger(), "fork() 失败: %s", strerror(errno));
    return false;
  }

  if (pid == 0) {
    // ── 子进程 ────────────────────────────────────────────────────────────
    // setpgid(0,0)：成为新进程组的组长
    // 之后 kill(-pid, SIG) 可以将信号发给整个进程组（含 launch 的所有子节点）
    ::setpgid(0, 0);
    ::execlp("ros2", "ros2", "launch", "auto_construct",
             launch_file.c_str(), nullptr);
    // execlp 仅失败才返回
    ::_exit(EXIT_FAILURE);
  }

  // ── 父进程 ────────────────────────────────────────────────────────────────
  current_pid_ = pid;
  RCLCPP_INFO(this->get_logger(),
    "▶ 启动 [%s] PID=%d", launch_file.c_str(), pid);
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────

void MappingManager::stop_current_process()
{
  // ⚠️ 调用此函数前必须已释放 state_mtx_（内部有阻塞等待）

  // 停止覆盖路径规划进程
  if (coverage_pid_ > 0) {
    const pid_t cov_pid = coverage_pid_;
    coverage_pid_ = -1;

    RCLCPP_INFO(this->get_logger(), "⏹ 终止覆盖路径规划进程组 PGID=%d (SIGINT)", cov_pid);
    ::kill(-cov_pid, SIGINT);

    constexpr int kTimeoutMs   = 5000;
    constexpr int kIntervalMs  = 100;
    int elapsed_ms = 0;
    int status     = 0;

    while (elapsed_ms < kTimeoutMs) {
      pid_t r = ::waitpid(cov_pid, &status, WNOHANG);
      if (r == cov_pid) {
        RCLCPP_INFO(this->get_logger(), "✅ 覆盖路径规划进程组 %d 已退出", cov_pid);
        break;
      }
      if (r == -1 && errno == ECHILD) {
        break;
      }
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

  // 停止主进程
  if (current_pid_ <= 0) return;

  const pid_t pid = current_pid_;
  current_pid_ = -1;  // 先置 -1，防止析构时重复 kill

  RCLCPP_INFO(this->get_logger(), "⏹ 终止进程组 PGID=%d (SIGINT)", pid);

  // 向整个进程组发 SIGINT（ROS2 节点会响应 SIGINT 做优雅退出）
  ::kill(-pid, SIGINT);

  constexpr int kTimeoutMs   = 5000;
  constexpr int kIntervalMs  = 100;
  int elapsed_ms = 0;
  int status     = 0;

  while (elapsed_ms < kTimeoutMs) {
    pid_t r = ::waitpid(pid, &status, WNOHANG);
    if (r == pid) {
      RCLCPP_INFO(this->get_logger(), "✅ 进程组 %d 已退出", pid);
      return;
    }
    if (r == -1 && errno == ECHILD) {
      return;  // 子进程已不存在
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(kIntervalMs));
    elapsed_ms += kIntervalMs;
  }

  // 超时 → SIGKILL
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

  // ✅ MultiThreadedExecutor + Reentrant 回调组 的组合：
  //
  //  线程 A：执行 handle_finish_mapping，在 future.wait_for 轮询
  //  线程 B：执行 /save_map client 的响应回调，将 future 置为 ready
  //
  // 两个线程并发，future.wait_for 轮询退出，不会死锁。
  // 线程数建议 >= 2，保证至少一个线程处理 client 响应。
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions{}, 4);

  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}