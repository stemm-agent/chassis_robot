/**
 * @file body_avoider.cpp
 * @brief VFH (Vector Field Histogram) 避障节点实现
 *
 * 架构：
 *   follower → /cmd_vel_raw → body_avoider (VFH) → /cmd_vel → chassis
 *                                ↑
 *                           lidar → /scan
 *
 * VFH 算法步骤：
 *   1. 将 360° 扫描构建极坐标直方图（障碍物密度）
 *   2. 直方图平滑（移动平均）
 *   3. 二值化 + 寻找无障碍扇区（Valleys）
 *   4. 选择最接近期望方向的安全扇区
 *   5. 根据扇区宽度和最近障碍物动态限速输出
 *
 * 参考：
 *   - Borenstein, J., & Koren, Y. (1991). The vector field histogram
 *   - Navigation2 DWB 局部规划器
 */

#include "body_avoider.hpp"
#include <algorithm>
#include <limits>

namespace body_avoider
{

BodyAvoider::BodyAvoider()
: Node("body_avoider"),
  scan_received_(false),
  cmd_raw_received_(false),
  all_blocked_(false),
  closest_obstacle_(std::numeric_limits<double>::infinity()),
  closest_obstacle_bin_(0)
{
  // === 声明参数（带默认值，可通过 launch 文件覆盖） ===
  this->declare_parameter("hist_bins",            72);
  this->declare_parameter("safe_distance",         1.2);   // 前方感知距离，障碍物在此范围内开始进入直方图
  this->declare_parameter("safe_distance_stop",    0.5);   // 硬停距离0.5m
  this->declare_parameter("robot_radius",          0.20);  // 机器人半径
  this->declare_parameter("hist_threshold",        0.25);  // 直方图阻塞阈值
  this->declare_parameter("smooth_width",          2);     // 平滑窗口半径
  this->declare_parameter("max_linear_speed",      0.5);   // 最大线速度
  this->declare_parameter("max_angular_speed",     1.0);   // 最大角速度
  this->declare_parameter("angular_p_gain",        2.5);   // 转向P增益
  this->declare_parameter("repulsive_gain",         0.8);   // 排斥力增益（越大避障越提前，0.8避免与follower PD振荡）
  this->declare_parameter("min_valley_width_deg",  30.0);  // 最小安全扇区30°

  // 人体豁免参数
  this->declare_parameter("exemption_angle_deg",     25.0);  // 豁免半角25°
  this->declare_parameter("exemption_dist_margin",   0.3);   // 距离门控0.3m

  // 绕行侧方脱离阈值
  this->declare_parameter("bypass_clearance_distance", 0.3); // 侧方障碍物脱离距离(m)

  // === 读取参数 ===
  this->get_parameter("hist_bins",           hist_bins_);
  this->get_parameter("safe_distance",       safe_distance_);
  this->get_parameter("safe_distance_stop",  safe_distance_stop_);
  this->get_parameter("robot_radius",        robot_radius_);
  this->get_parameter("hist_threshold",      hist_threshold_);
  this->get_parameter("smooth_width",        smooth_width_);
  this->get_parameter("max_linear_speed",    max_linear_speed_);
  this->get_parameter("max_angular_speed",   max_angular_speed_);
  this->get_parameter("angular_p_gain",      angular_p_gain_);
  this->get_parameter("repulsive_gain",       repulsive_gain_);
  this->get_parameter("min_valley_width_deg", min_valley_width_deg_);
  this->get_parameter("exemption_angle_deg",  exemption_angle_deg_);
  this->get_parameter("exemption_dist_margin", exemption_dist_margin_);
  this->get_parameter("bypass_clearance_distance", bypass_clearance_distance_);

  exemption_angle_rad_ = exemption_angle_deg_ * M_PI / 180.0;

  bin_angle_ = 2.0 * M_PI / hist_bins_;

  // 预分配直方图内存
  polar_hist_.resize(hist_bins_, 0.0f);
  smoothed_hist_.resize(hist_bins_, 0.0f);
  bin_min_dist_.resize(hist_bins_, std::numeric_limits<float>::infinity());

  // 初始化期望速度为零
  desired_cmd_.linear.x  = 0.0;
  desired_cmd_.linear.y  = 0.0;
  desired_cmd_.linear.z  = 0.0;
  desired_cmd_.angular.x = 0.0;
  desired_cmd_.angular.y = 0.0;
  desired_cmd_.angular.z = 0.0;

  // 初始化人体追踪状态
  person_valid_    = false;
  person_angle_    = 0.0;
  person_distance_ = 0.0;

  // === 订阅 ===
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", rclcpp::SensorDataQoS(),
    std::bind(&BodyAvoider::scan_callback, this, std::placeholders::_1));

  cmd_raw_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel_raw", 10,
    std::bind(&BodyAvoider::cmd_raw_callback, this, std::placeholders::_1));

  body_posture_sub_ = this->create_subscription<bodyreader_msg::msg::Bodyposture>(
    "/body_posture", 10,
    std::bind(&BodyAvoider::body_posture_callback, this, std::placeholders::_1));

  // === 发布 ===
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // === 10Hz 控制循环 ===
  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&BodyAvoider::control_loop, this));

  RCLCPP_INFO(this->get_logger(),
    "BodyAvoider initialized: bins=%d, safe_dist=%.2fm, stop_dist=%.2fm, "
    "robot_radius=%.2fm, threshold=%.2f, max_linear=%.2f, max_angular=%.2f, "
    "repulsive_gain=%.1f, exemption_angle=%.1fdeg, exemption_margin=%.2fm",
    hist_bins_, safe_distance_, safe_distance_stop_, robot_radius_,
    hist_threshold_, max_linear_speed_, max_angular_speed_,
    repulsive_gain_, exemption_angle_deg_, exemption_dist_margin_);

  // 初始化逃逸状态
  escape_active_ = false;
  escape_phase_  = 0;

  // === 初始化避障状态机 (2026-06-11) ===
  avoidance_state_          = AvoidanceState::NORMAL;
  captured_yaw_             = 0.0;
  current_yaw_              = 0.0;
  current_x_                = 0.0;
  current_y_                = 0.0;
  avoidance_attempt_        = 0;
  avoidance_direction_      = 1;
  front_angle_range_        = 12.0 * M_PI / 180.0;  // ±12°
  person_exemption_enabled_ = true;

  // EXPLORE 前进距离控制
  explore_start_x_ = 0.0;
  explore_start_y_ = 0.0;
  explore_d_min_   = 0.0;
  explore_d_max_   = 2.5;  // 硬上限2.5m
  explore_front_clear_count_ = 0;  // 衰减计数器

  // === 避障相关订阅/发布 ===
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom_combined", 10,
    std::bind(&BodyAvoider::odom_callback, this, std::placeholders::_1));

  avoidance_state_pub_ = this->create_publisher<std_msgs::msg::String>(
    "/avoidance_state", 10);

  RCLCPP_INFO(this->get_logger(),
    "Avoidance state machine: front_angle=%.1f°, "
    "rotate_timeout=2.5s, explore_timeout=8s, total_timeout=25s, safe_dist=%.2fm",
    front_angle_range_ * 180.0 / M_PI, safe_distance_);
}

// =============================================================================
// 回调函数
// =============================================================================

void BodyAvoider::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  last_scan_ = *msg;
  scan_received_ = true;
}

void BodyAvoider::cmd_raw_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  desired_cmd_ = *msg;
  cmd_raw_received_ = true;
}

void BodyAvoider::body_posture_callback(const bodyreader_msg::msg::Bodyposture::SharedPtr msg)
{
  // lock_status == 2 表示已锁定追踪人体
  // centerofmass_z > 0.1 避免无效数据 (0,0,0)
  if (msg->lock_status == 2 && msg->centerofmass_z > 0.1f)
  {
    person_valid_    = true;
    // 相机坐标系(Astra): X=左, Z=前, atan2(x,z)>0 = 人在左侧
    // 激光坐标系(ROS标准): 0=前, +角度=左(CCW), -角度=右
    // 两者方向一致，无需转换
    person_angle_    = std::atan2(msg->centerofmass_x, msg->centerofmass_z);
    person_distance_ = msg->centerofmass_z / 1000.0f;  // mm → m
  }
  else
  {
    person_valid_ = false;
  }
}

void BodyAvoider::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_yaw_ = get_yaw_from_quaternion(msg->pose.pose.orientation);
  current_x_   = msg->pose.pose.position.x;
  current_y_   = msg->pose.pose.position.y;
}

// =============================================================================
// 10Hz 主控制循环
// =============================================================================

void BodyAvoider::control_loop()
{
  geometry_msgs::msg::Twist safe_cmd;

  // 如果没有收到扫描数据或期望速度，停止
  if (!scan_received_ || !cmd_raw_received_)
  {
    safe_cmd.linear.x  = 0.0;
    safe_cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(safe_cmd);
    return;
  }

  // 检查扫描数据是否过期（超过 0.5 秒视为过期）
  auto now = this->now();
  auto scan_age = now - last_scan_.header.stamp;
  if (scan_age.seconds() > 0.5)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Laser scan data is stale (%.2fs old), stopping", scan_age.seconds());
    safe_cmd.linear.x  = 0.0;
    safe_cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(safe_cmd);
    return;
  }

  // === 步骤1: 构建极坐标直方图 ===
  build_polar_histogram(last_scan_);

  // === 步骤2: 平滑直方图 ===
  smooth_histogram();

  // === 步骤2.5: 人体豁免区条件清零（防smooth泄漏 + 保留真实障碍物） ===
  if (person_exemption_enabled_ && person_valid_)
  {
    int person_bin = angle_to_bin(person_angle_);
    int exempt_half_bins = static_cast<int>(exemption_angle_rad_ / bin_angle_);
    int clear_half_bins = exempt_half_bins + smooth_width_;

    double person_dist_threshold = person_distance_ - exemption_dist_margin_;
    if (person_dist_threshold < 0.0) person_dist_threshold = 0.0;

    for (int d = -clear_half_bins; d <= clear_half_bins; d++)
    {
      int bin = (person_bin + d + hist_bins_) % hist_bins_;
      if (bin_min_dist_[bin] >= person_dist_threshold) {
        smoothed_hist_[bin] = 0.0f;
        polar_hist_[bin]    = 0.0f;
      }
    }
  }

  // === 避障状态机 (2026-06-11) ===
  if (avoidance_state_ != AvoidanceState::NORMAL) {
    update_avoidance();
    return;
  }

  // 触发避障：正前方 ±front_angle_range_ 内检测到障碍物 (< safe_distance_)
  if (is_front_blocked()) {
    enter_avoidance();
    update_avoidance();
    return;
  }

  // === 正常模式：人体追踪 + VFH 安全门控 ===
  // 人体未追踪时发送零速（bodydata_process也会发零速，双重保险）
  if (!person_valid_)
  {
    safe_cmd.linear.x  = 0.0;
    safe_cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(safe_cmd);
    return;
  }

  // === 步骤3: 寻找无障碍扇区 ===
  find_valleys();

  // === 步骤4: 选择最佳扇区 ===
  // 期望方向：人体追踪为最高优先级，其次使用 follower 的 cmd_vel 推导
  double desired_direction = 0.0;
  if (person_valid_) {
    // 有人在 → 始终指向人体方向（VFH 在确保安全的前提下尽量靠近人）
    desired_direction = person_angle_;
  } else if (fabs(desired_cmd_.linear.x) > 0.01) {
    // 无人体追踪但有前进意图 → 正前方
    desired_direction = 0.0;
  } else if (fabs(desired_cmd_.angular.z) > 0.01) {
    // 无人体追踪且纯旋转 → 跟随转向方向
    desired_direction = (desired_cmd_.angular.z > 0) ? M_PI_2 : -M_PI_2;
  } else {
    desired_direction = 0.0;  // 停止
  }

  int best_valley = select_best_valley(desired_direction);

  // === 步骤5: 计算安全速度 ===
  if (best_valley >= 0) {
    // 找到安全方向 → 退出逃逸模式
    escape_active_ = false;
    // 传入 follower 的完整期望速度 + 期望方向，由 VFH 做安全门控 + 渐进排斥
    compute_safe_velocity(best_valley, desired_cmd_.linear.x,
                          desired_cmd_.angular.z, desired_direction);
  } else {
    // 没有安全扇区：带超时的旋转探索
    if (!escape_active_) {
      escape_active_ = true;
      escape_start_time_ = this->now();
      escape_phase_ = 0;
    }

    double elapsed = (this->now() - escape_start_time_).seconds();

    // 每 5 秒切换逃逸策略
    if (elapsed > 5.0) {
      escape_phase_ = 1 - escape_phase_;
      escape_start_time_ = this->now();
      RCLCPP_WARN(this->get_logger(),
        "Escape timeout (%.1fs), switching to phase %d", elapsed, escape_phase_);
    }

    if (escape_phase_ == 0) {
      // 阶段0: 远离最近障碍物
      double escape_angle = bin_to_angle(closest_obstacle_bin_) + M_PI;
      escape_angle = normalize_angle(escape_angle);
      safe_cmd.angular.z = (escape_angle > 0 ? 1.0 : -1.0) * max_angular_speed_ * 0.5;
    } else {
      // 阶段1: 转向人体方向（如果可见），否则前方
      if (person_valid_) {
        safe_cmd.angular.z = person_angle_ * 0.8;
      } else {
        safe_cmd.angular.z = 0.5;  // 原地右转
      }
    }

    // 关键改进：加小线速度避免纯旋转死锁
    safe_cmd.linear.x = 0.05;

    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "All blocked! Escaping phase=%d elapsed=%.1fs", escape_phase_, elapsed);
    cmd_vel_pub_->publish(safe_cmd);
    return;
  }
}

// =============================================================================
// VFH 算法核心
// =============================================================================

/**
 * @brief 构建极坐标直方图
 *
 * 将 360° 激光扫描数据映射到 hist_bins_ 个扇形中。
 * 每个扇形的障碍物密度 = clamp( (safe_distance - range) / safe_distance, 0, 1 )
 * 距离越近 → 密度越高。
 * 同时考虑角度扩散：一个激光点会影响相邻几个扇形。
 */
void BodyAvoider::build_polar_histogram(const sensor_msgs::msg::LaserScan &scan)
{
  // 清空直方图
  std::fill(polar_hist_.begin(), polar_hist_.end(), 0.0f);
  std::fill(bin_min_dist_.begin(), bin_min_dist_.end(),
            std::numeric_limits<float>::infinity());

  closest_obstacle_ = std::numeric_limits<double>::infinity();
  closest_obstacle_bin_ = 0;

  const size_t num_readings = scan.ranges.size();
  const double angle_min = scan.angle_min;
  const double angle_increment = scan.angle_increment;

  // 角度扩散半宽度（弧度）
  // 用 atan2(robot_radius, safe_distance) 决定一个激光点影响多少个扇形
  // 不再硬顶 0.50，让 spread 随 safe_distance 自然缩放
  const double spread_angle = std::atan2(robot_radius_ + 0.05, safe_distance_);
  const int spread_bins = std::max(3,
                                   static_cast<int>(spread_angle / bin_angle_));

  // === 人体豁免预计算（距离门控） ===
  // 只有当 person_exemption_enabled_ && person_valid_ 时才启用豁免
  int exempt_half_bins = 0;
  int person_bin      = 0;
  double person_dist_threshold = 0.0;  // 低于此距离的是障碍物，高于的是人体
  if (person_exemption_enabled_ && person_valid_)
  {
    person_bin = angle_to_bin(person_angle_);
    exempt_half_bins = static_cast<int>(exemption_angle_rad_ / bin_angle_);
    // 距离门控: 测距 < person_distance - margin → 真实障碍物（不豁免）
    person_dist_threshold = person_distance_ - exemption_dist_margin_;
    if (person_dist_threshold < 0.0) person_dist_threshold = 0.0;
  }

  for (size_t i = 0; i < num_readings; ++i)
  {
    float range = scan.ranges[i];

    // 过滤无效数据
    if (std::isnan(range) || std::isinf(range))
      continue;
    if (range < scan.range_min || range > scan.range_max)
      continue;

    // 计算该激光点的角度（0 = 正前方）
    double angle = angle_min + i * angle_increment;
    // 归一化到 [-pi, pi]
    angle = std::atan2(std::sin(angle), std::cos(angle));

    // === 人体方向豁免（距离门控） ===
    if (person_exemption_enabled_ && person_valid_)
    {
      int this_bin = angle_to_bin(angle);
      // 环形距离（最短弧上的 bin 差）
      int bin_diff = std::abs(this_bin - person_bin);
      if (bin_diff > hist_bins_ / 2)
        bin_diff = hist_bins_ - bin_diff;

      if (bin_diff <= exempt_half_bins)
      {
        // 激光点落在人体豁免扇区内
        if (range >= person_dist_threshold)
        {
          // 测距 ≥ 人体距离 - margin → 这是人体自身 → 跳过
          continue;
        }
        // 否则：测距明显小于人体距离 → 人机之间的真实障碍物 → 正常处理
      }
    }

    int center_bin = angle_to_bin(angle);

    // 计算障碍物密度 — 线性下降: safe_distance处=0, safe_distance_stop处=1.0
    // 配合 hist_threshold 控制探测距离:
    //   0.5m→0.0, 0.4m→0.33, 0.3m→0.67, 0.2m→1.0
    // 配合 hist_threshold=0.25 + spread扩散, 连续墙壁在 ~0.45m 被探测到
    float density = 0.0f;
    if (range < safe_distance_stop_) {
      density = 1.0f;  // 极近距离 → 完全阻塞
    } else if (range < safe_distance_) {
      density = static_cast<float>((safe_distance_ - range) /
                                   (safe_distance_ - safe_distance_stop_));
      if (density > 1.0f) density = 1.0f;
    }

    // 跟踪最近障碍物（全局）
    if (range < closest_obstacle_) {
      closest_obstacle_ = range;
      closest_obstacle_bin_ = center_bin;
    }

    // 跟踪每bin最近障碍物距离（用于条件清零——只有"比人体远"的bin才清零）
    if (density > 0.0f && range < bin_min_dist_[center_bin]) {
      bin_min_dist_[center_bin] = range;
    }

    if (density > 0.0f) {
      // 角度扩散：障碍物影响相邻扇形
      for (int d = -spread_bins; d <= spread_bins; ++d) {
        int bin = (center_bin + d + hist_bins_) % hist_bins_;
        // 越远离中心，影响越小
        float weight = 1.0f - static_cast<float>(std::abs(d)) / (spread_bins + 1);
        polar_hist_[bin] = std::max(polar_hist_[bin], density * weight);
        // 扩散也更新该bin的最近距离
        if (range < bin_min_dist_[bin]) {
          bin_min_dist_[bin] = range;
        }
      }
    }
  }

  // === 人体豁免区条件清零 ===
  // 只清零"距离门控判断为人体自身"的bin（min_dist >= person_dist_threshold），
  // 保留真正障碍物的bin（min_dist < person_dist_threshold）。
  //
  // 关键：无条件清零会抹掉人机之间的墙壁/障碍物 → 撞墙；
  //       条件清零则保留近距离障碍，同时清除人体自身反射和spread泄漏。
  if (person_valid_)
  {
    for (int d = -exempt_half_bins; d <= exempt_half_bins; d++)
    {
      int bin = (person_bin + d + hist_bins_) % hist_bins_;
      // 只有比人体远的才清零（人体自身反射 / spread泄漏）
      if (bin_min_dist_[bin] >= person_dist_threshold) {
        polar_hist_[bin] = 0.0f;
      }
      // 否则：有障碍物比人体更近 → 保留障碍物密度，不清零！
    }
  }
}

/**
 * @brief 移动平均平滑直方图
 *
 * 减少噪声，使扇区边界更连续。
 */
void BodyAvoider::smooth_histogram()
{
  std::fill(smoothed_hist_.begin(), smoothed_hist_.end(), 0.0f);

  for (int i = 0; i < hist_bins_; ++i)
  {
    float sum = 0.0f;
    int count = 0;
    for (int d = -smooth_width_; d <= smooth_width_; ++d)
    {
      int idx = (i + d + hist_bins_) % hist_bins_;
      sum += polar_hist_[idx];
      count++;
    }
    smoothed_hist_[i] = sum / count;
  }
}

/**
 * @brief 寻找无障碍扇区（Valleys）
 *
 * 扫描平滑直方图，找到连续低于阈值的扇形段。
 * 每个 valley 记录为 (start_bin, end_bin)。
 * 由于是环形 360°，需要处理环绕情况。
 */
void BodyAvoider::find_valleys()
{
  valleys_.clear();

  // 最小扇区宽度（bin数）
  const double min_valley_width_rad = min_valley_width_deg_ * M_PI / 180.0;
  const int min_valley_bins = std::max(2, static_cast<int>(min_valley_width_rad / bin_angle_));

  // 从直方图最小值处开始搜索（避免从阻塞区中间开始）
  int start_search = 0;
  float min_density = 1.0f;
  for (int i = 0; i < hist_bins_; ++i) {
    if (smoothed_hist_[i] < min_density) {
      min_density = smoothed_hist_[i];
      start_search = i;
    }
  }

  // 环形扫描两次以确保跨越 0° 的扇区被正确处理
  bool in_valley = false;
  int valley_start = 0;
  int total_scanned = 0;

  for (int round = 0; round < 2; ++round)
  {
    for (int i = 0; i < hist_bins_; ++i)
    {
      int idx = (start_search + i) % hist_bins_;
      total_scanned++;
      if (total_scanned > hist_bins_ * 2) break;  // 安全保护

      bool is_free = (smoothed_hist_[idx] < hist_threshold_);

      if (is_free && !in_valley)
      {
        // 进入无障碍区
        in_valley = true;
        valley_start = idx;
      }
      else if (!is_free && in_valley)
      {
        // 退出无障碍区
        in_valley = false;
        int valley_end = (idx - 1 + hist_bins_) % hist_bins_;
        int valley_width = (valley_end - valley_start + hist_bins_) % hist_bins_ + 1;

        if (valley_width >= min_valley_bins)
        {
          valleys_.push_back({valley_start, valley_end});
        }
      }
    }
    if (!in_valley && total_scanned >= hist_bins_) break;  // 已经完整扫描一圈
  }

  // 如果最后一个扇区跨越到开头
  if (in_valley)
  {
    int valley_end = (start_search - 1 + hist_bins_) % hist_bins_;
    int valley_width = (valley_end - valley_start + hist_bins_) % hist_bins_ + 1;
    if (valley_width >= min_valley_bins)
    {
      valleys_.push_back({valley_start, valley_end});
    }
  }

  // 检查是否所有方向都被阻塞
  all_blocked_ = valleys_.empty();
}

/**
 * @brief 选择最接近期望方向的安全扇区
 *
 * @param desired_direction 期望的移动方向（弧度，0=正前方）
 * @return 最佳扇区索引，-1 表示没有安全扇区
 */
int BodyAvoider::select_best_valley(double desired_direction)
{
  if (valleys_.empty()) return -1;

  // 将期望方向归一化到 [-pi, pi]
  desired_direction = normalize_angle(desired_direction);
  int desired_bin = angle_to_bin(desired_direction);

  // === 第一轮：只考虑覆盖期望方向的valley ===
  // 豁免区清零后，valley可能分裂。覆盖期望方向的valley
  // 无条件优先于"中心近但不覆盖"的valley。
  int best_covering_idx = -1;
  double best_covering_score = -std::numeric_limits<double>::infinity();

  for (size_t v = 0; v < valleys_.size(); ++v)
  {
    int start = valleys_[v].first;
    int end   = valleys_[v].second;

    bool covers = false;
    if (end >= start) {
      covers = (desired_bin >= start && desired_bin <= end);
    } else {
      covers = (desired_bin >= start || desired_bin <= end);
    }
    if (!covers) continue;

    // 计算该valley评分
    int center_bin;
    if (end < start) {
      int width = (end - start + hist_bins_) % hist_bins_;
      center_bin = (start + width / 2) % hist_bins_;
    } else {
      center_bin = start + (end - start) / 2;
    }
    double valley_angle = bin_to_angle(center_bin);
    double angle_diff = std::abs(normalize_angle(valley_angle - desired_direction));
    int valley_width = (end - start + hist_bins_) % hist_bins_ + 1;

    double score = -angle_diff * 2.0 + valley_width * 0.1;
    if (score > best_covering_score) {
      best_covering_score = score;
      best_covering_idx = static_cast<int>(v);
    }
  }

  // 如果有覆盖valley，直接返回
  if (best_covering_idx >= 0) {
    return best_covering_idx;
  }

  // === 第二轮：没有覆盖valley，回退到最近中心 ===
  int best_idx = 0;
  double best_score = -std::numeric_limits<double>::infinity();

  for (size_t v = 0; v < valleys_.size(); ++v)
  {
    int start = valleys_[v].first;
    int end   = valleys_[v].second;

    int center_bin;
    if (end < start) {
      int width = (end - start + hist_bins_) % hist_bins_;
      center_bin = (start + width / 2) % hist_bins_;
    } else {
      center_bin = start + (end - start) / 2;
    }
    double valley_angle = bin_to_angle(center_bin);
    double angle_diff = std::abs(normalize_angle(valley_angle - desired_direction));
    int valley_width = (end - start + hist_bins_) % hist_bins_ + 1;

    double score = -angle_diff * 2.0 + valley_width * 0.1;
    if (score > best_score) {
      best_score = score;
      best_idx = static_cast<int>(v);
    }
  }

  return best_idx;
}

/**
 * @brief 计算安全的速度指令
 *
 * 架构: follower 的 angular.z 是主控（PD 精确定位人体），
 *       VFH 只做安全门控——当前方安全时原样放行，前方被挡时才介入修正。
 *
 * @param best_valley     选中的安全扇区索引
 * @param desired_linear   期望线速度（来自 follower）
 * @param desired_angular  期望角速度（来自 follower）
 * @param desired_direction 期望方向（人体角度）
 */
void BodyAvoider::compute_safe_velocity(int best_valley, double desired_linear,
                                         double desired_angular, double desired_direction)
{
  geometry_msgs::msg::Twist safe_cmd;
  safe_cmd.linear.x  = 0.0;
  safe_cmd.linear.y  = 0.0;
  safe_cmd.linear.z  = 0.0;
  safe_cmd.angular.x = 0.0;
  safe_cmd.angular.y = 0.0;
  safe_cmd.angular.z = 0.0;

  if (best_valley < 0 || static_cast<size_t>(best_valley) >= valleys_.size())
  {
    cmd_vel_pub_->publish(safe_cmd);
    return;
  }

  int start = valleys_[best_valley].first;
  int end   = valleys_[best_valley].second;

  // 计算扇区中心
  int center_bin;
  if (end < start) {
    int width = (end - start + hist_bins_) % hist_bins_;
    center_bin = (start + width / 2) % hist_bins_;
  } else {
    center_bin = start + (end - start) / 2;
  }

  double valley_angle = bin_to_angle(center_bin);
  int valley_width = (end - start + hist_bins_) % hist_bins_ + 1;

  // 最小安全扇区宽度(bins)，角速度门控和线速度因子共用
  double min_valley_width_rad = min_valley_width_deg_ * M_PI / 180.0;
  int min_valley_bins = std::max(2, static_cast<int>(min_valley_width_rad / bin_angle_));

  // 计算 valley 是否覆盖期望方向
  int desired_bin = angle_to_bin(desired_direction);
  bool valley_covers_desired = false;
  if (end >= start) {
    valley_covers_desired = (desired_bin >= start && desired_bin <= end);
  } else {
    valley_covers_desired = (desired_bin >= start || desired_bin <= end);
  }
  bool valley_is_wide = (valley_width >= min_valley_bins);

  // === 角速度：渐进排斥（Strategy A — 主动避障） ===
  // 不再用二值门控，而是让障碍物密度产生连续的排斥力矩，
  // 与 follower 的 PD 追踪角速度平滑混合。
  // - 障碍物远 → 排斥力弱 → 主要由 PD 追踪人体
  // - 障碍物近 → 排斥力强 → 机器人主动绕行
  double repulsive_ang = compute_repulsive_angular(desired_direction);

  if (valley_covers_desired && valley_is_wide) {
    // NORMAL追踪模式：前方通畅 + 扇区够宽 → PD直接放行，不加排斥力
    //
    // 排斥力在正常追踪时会与 follower PD 形成正反馈振荡：
    //   侧墙排斥推右 → PD拉左 → 延迟导致过冲 → PD推右 → 循环
    // VFH 的安全门控（前方障碍→状态机、valley限速）已足够保证安全，
    // 排斥力仅在避障状态机中发挥作用（valley不覆盖期望方向时）。
    safe_cmd.angular.z = desired_angular;
  } else {
    // 期望方向被挡 → VFH 接管，转向安全valley中心
    // 混入排斥力使转向更自然（不必硬跳到valley中心）
    double angle_error = normalize_angle(valley_angle);
    double repulsive_term = repulsive_ang * repulsive_gain_ * 0.5;
    if (std::fabs(repulsive_term) < 0.02) repulsive_term = 0.0;
    safe_cmd.angular.z = angle_error * angular_p_gain_
                       + repulsive_term;
  }

  // 限幅 (C++14 兼容)
  if (safe_cmd.angular.z > max_angular_speed_)  safe_cmd.angular.z = max_angular_speed_;
  if (safe_cmd.angular.z < -max_angular_speed_) safe_cmd.angular.z = -max_angular_speed_;

  // === 线速度：根据所选valley内最近障碍物动态限速 ===
  // 关键：必须使用valley内的最近距离（bin_min_dist_），而非全局 closest_obstacle_。
  // 否则当机器人转向安全方向后，全局最近距离仍来自前方障碍物 → speed_factor
  // 被压低 → 机器人转对了方向也无法前进 → 死锁在障碍物前。
  double min_dist_in_valley = std::numeric_limits<double>::infinity();
  if (end >= start) {
    for (int b = start; b <= end; ++b) {
      if (bin_min_dist_[b] < min_dist_in_valley) {
        min_dist_in_valley = bin_min_dist_[b];
      }
    }
  } else {
    // 环绕情况：valley 跨越 0°（如 start=70, end=5）
    for (int b = start; b < hist_bins_; ++b) {
      if (bin_min_dist_[b] < min_dist_in_valley) {
        min_dist_in_valley = bin_min_dist_[b];
      }
    }
    for (int b = 0; b <= end; ++b) {
      if (bin_min_dist_[b] < min_dist_in_valley) {
        min_dist_in_valley = bin_min_dist_[b];
      }
    }
  }

  // 动态线速度：valley内越近越慢（而非全局最近距离）
  double speed_factor = 1.0;
  if (min_dist_in_valley < safe_distance_) {
    speed_factor = std::max(0.0, (min_dist_in_valley - safe_distance_stop_) /
                                   (safe_distance_ - safe_distance_stop_));
  }

  // 扇区宽度因子：扇区越窄，越要减速
  double width_factor = std::min(1.0, static_cast<double>(valley_width) / min_valley_bins);

  double limited_linear = desired_linear * speed_factor * width_factor;

  // [DEBUG] 每3秒打印状态，便于远程诊断
  static int debug_counter = 0;
  if (++debug_counter % 30 == 0) {
    RCLCPP_INFO(this->get_logger(),
      "[DIAG] person=%.1f° valley=%.1f° | "
      "cover=%s | ang_in=%.2f ang_out=%.2f rep_ang=%.2f | "
      "lin_in=%.2f lin_out=%.2f vally_min=%.2fm glob_min=%.2fm sf=%.2f wf=%.2f",
      person_angle_ * 180.0 / M_PI, valley_angle * 180.0 / M_PI,
      (valley_covers_desired && valley_is_wide) ? "OK" : "VFH",
      desired_angular, safe_cmd.angular.z, repulsive_ang,
      desired_linear, safe_cmd.linear.x,
      min_dist_in_valley, closest_obstacle_, speed_factor, width_factor);
  }

  if (limited_linear > max_linear_speed_)  limited_linear = max_linear_speed_;
  if (limited_linear < -max_linear_speed_) limited_linear = -max_linear_speed_;
  safe_cmd.linear.x = limited_linear;

  // 如果前方有紧迫障碍，强制减速
  if (closest_obstacle_ < safe_distance_stop_) {
    safe_cmd.linear.x = 0.0;
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
      "Obstacle too close (%.2fm)! Stopping.", closest_obstacle_);
  }

  // 角速度死区
  if (fabs(safe_cmd.angular.z) < 0.02) {
    safe_cmd.angular.z = 0.0;
  }
  // 线速度死区
  if (fabs(safe_cmd.linear.x) < 0.01) {
    safe_cmd.linear.x = 0.0;
  }

  cmd_vel_pub_->publish(safe_cmd);
}

// =============================================================================
// 工具函数
// =============================================================================

int BodyAvoider::angle_to_bin(double angle) const
{
  // 归一化到 [-pi, pi]
  angle = std::atan2(std::sin(angle), std::cos(angle));
  // 映射到 [0, 2π)
  if (angle < 0) angle += 2.0 * M_PI;
  // 映射到 bin 索引
  int bin = static_cast<int>(std::round(angle / bin_angle_)) % hist_bins_;
  return (bin + hist_bins_) % hist_bins_;
}

double BodyAvoider::bin_to_angle(int bin) const
{
  double angle = bin * bin_angle_;
  // 映射到 [-pi, pi]
  if (angle > M_PI) angle -= 2.0 * M_PI;
  return angle;
}

double BodyAvoider::normalize_angle(double angle) const
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

// =============================================================================
// Strategy A: 渐进排斥力矩
// =============================================================================

/**
 * @brief 从所有有障碍物的 bin 计算排斥角速度
 *
 * 核心思想：每个被阻塞的 bin 产生一个排斥力矩，方向是远离该 bin。
 * 排斥力大小 = 密度 × sin(障碍物相对期望方向的角度)
 *
 * 效果：
 *   - 期望方向前方有障碍 → 排斥力推机器人偏转
 *   - 障碍物越近（密度越高）→ 排斥越强
 *   - 左右对称有障碍 → 排斥力抵消，机器人直走
 *
 * @param desired_direction 期望运动方向 (rad)
 * @return 排斥角速度 (rad/s)，正值=右转，负值=左转
 */
double BodyAvoider::compute_repulsive_angular(double desired_direction) const
{
  double repulsive = 0.0;

  for (int i = 0; i < hist_bins_; ++i)
  {
    float density = smoothed_hist_[i];
    if (density <= 0.0f) continue;

    double bin_angle = bin_to_angle(i);
    double angle_diff = normalize_angle(bin_angle - desired_direction);

    // sin(angle_diff): 障碍物在右侧 → sin>0 → 需要左转（-sin）
    // 障碍物在左侧 → sin<0 → 需要右转（-sin>0）
    // 所以用 -sin 作为排斥方向
    repulsive -= density * std::sin(angle_diff);
  }

  return repulsive;
}

// =============================================================================
// 避障状态机 (2026-06-11)
// =============================================================================

double BodyAvoider::get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion &q) const
{
  // From tf2: siny_cosp = 2 * (qw * qz + qx * qy)
  //          cosy_cosp = 1 - 2 * (qy*qy + qz*qz)
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

bool BodyAvoider::is_front_blocked() const
{
  // 直接检查原始激光距离，绕过直方图管线避免信号衰减
  // 直方图管线：密度→扩散→平滑，总衰减系数~0.7，墙壁有效检测仅~1.03m
  // 直接用原始距离，检测到 safe_distance_=1.2m 即触发（零衰减）
  if (!scan_received_ || last_scan_.ranges.empty()) return false;

  const auto& ranges = last_scan_.ranges;
  const double angle_min = last_scan_.angle_min;
  const double angle_increment = last_scan_.angle_increment;
  const float range_min = last_scan_.range_min;
  const float range_max = last_scan_.range_max;

  for (size_t i = 0; i < ranges.size(); ++i)
  {
    float range = ranges[i];
    if (std::isnan(range) || std::isinf(range)) continue;
    if (range < range_min || range > range_max) continue;

    double angle = angle_min + i * angle_increment;
    angle = std::atan2(std::sin(angle), std::cos(angle));  // 归一化到[-pi,pi]

    // 人体豁免：该激光点落在人体豁免区内且距离符合人体特征 → 跳过
    if (person_exemption_enabled_ && person_valid_)
    {
      double angle_diff = std::fabs(normalize_angle(angle - person_angle_));
      if (angle_diff <= exemption_angle_rad_ && range >= person_distance_ - exemption_dist_margin_)
      {
        continue;  // 人体，非障碍物
      }
    }

    if (std::fabs(angle) <= front_angle_range_ && range < safe_distance_)
    {
      return true;
    }
  }
  return false;
}

bool BodyAvoider::is_obstacle_side_clear() const
{
  // 直接检查原始激光距离，绕过直方图管线避免信号衰减
  // avoidance_direction_ > 0: 左转避开右侧障碍物 → 检查右侧 [-80°, -20°]
  // avoidance_direction_ < 0: 右转避开左侧障碍物 → 检查左侧 [20°, 80°]
  if (!scan_received_ || last_scan_.ranges.empty()) return false;

  const auto& ranges = last_scan_.ranges;
  const double angle_min = last_scan_.angle_min;
  const double angle_increment = last_scan_.angle_increment;
  const float range_min = last_scan_.range_min;
  const float range_max = last_scan_.range_max;

  double check_angle_min, check_angle_max;
  if (avoidance_direction_ < 0)
  {
    // 障碍物在左侧 → 检查左侧 [20°, 80°]
    check_angle_min =  20.0 * M_PI / 180.0;
    check_angle_max =  80.0 * M_PI / 180.0;
  }
  else
  {
    // 障碍物在右侧 → 检查右侧 [-80°, -20°]
    check_angle_min = -80.0 * M_PI / 180.0;
    check_angle_max = -20.0 * M_PI / 180.0;
  }

  for (size_t i = 0; i < ranges.size(); ++i)
  {
    float range = ranges[i];
    if (std::isnan(range) || std::isinf(range)) continue;
    if (range < range_min || range > range_max) continue;

    double angle = angle_min + i * angle_increment;
    angle = std::atan2(std::sin(angle), std::cos(angle));

    // 人体豁免：该激光点落在人体豁免区内 → 跳过
    if (person_exemption_enabled_ && person_valid_)
    {
      double angle_diff = std::fabs(normalize_angle(angle - person_angle_));
      if (angle_diff <= exemption_angle_rad_ && range >= person_distance_ - exemption_dist_margin_)
      {
        continue;
      }
    }

    if (angle >= check_angle_min && angle <= check_angle_max && range < safe_distance_)
    {
      return false;  // 侧方仍有障碍物
    }
  }
  return true;  // 侧方已脱离
}

void BodyAvoider::enter_avoidance()
{
  avoidance_state_          = AvoidanceState::ROTATE_AWAY;
  captured_yaw_             = current_yaw_;
  avoidance_start_time_     = this->now();
  phase_start_time_         = this->now();
  avoidance_attempt_        = 0;
  // 避障期间保持人体豁免：人体是追踪目标，不是障碍物
  // person_exemption_enabled_ 保持 true（默认值），由 is_front_blocked/is_obstacle_side_clear 的人体豁免逻辑处理

  // 选择避障方向：转向障碍物密度更小的一侧
  // ROS: angular.z > 0 = CCW(左转), angular.z < 0 = CW(右转)
  // avoidance_direction_ > 0 → 左转避开右侧障碍物
  // avoidance_direction_ < 0 → 右转避开左侧障碍物
  double left_density = 0.0, right_density = 0.0;
  int quarter = hist_bins_ / 4;
  for (int i = 1; i <= quarter; i++) {
    left_density  += smoothed_hist_[i];
    right_density += smoothed_hist_[hist_bins_ - i];
  }
  // 右侧障碍物更少 → 应该左转(angular.z>0) → direction>0
  avoidance_direction_ = (right_density < left_density) ? 1 : -1;

  // 发布避障状态
  auto msg = std_msgs::msg::String();
  msg.data = "rotating_away";
  avoidance_state_pub_->publish(msg);

  RCLCPP_INFO(this->get_logger(),
    "Enter avoidance: captured_yaw=%.1f°, direction=%s",
    captured_yaw_ * 180.0 / M_PI,
    avoidance_direction_ > 0 ? "left" : "right");
}

void BodyAvoider::update_avoidance()
{
  geometry_msgs::msg::Twist cmd;
  auto now = this->now();
  double elapsed_phase = (now - phase_start_time_).seconds();
  double elapsed_total = (now - avoidance_start_time_).seconds();

  // === 全局超时检查（25s） ===
  if (elapsed_total > 25.0) {
    RCLCPP_WARN(this->get_logger(),
      "Avoidance timeout (%.1fs), entering backoff", elapsed_total);
    avoidance_state_ = AvoidanceState::TIMEOUT_BACKOFF;
    phase_start_time_ = now;
    auto msg = std_msgs::msg::String();
    msg.data = "timeout_backoff";
    avoidance_state_pub_->publish(msg);
  }

  switch (avoidance_state_)
  {
    case AvoidanceState::ROTATE_AWAY:
    {
      double target_yaw = captured_yaw_ + avoidance_direction_ * M_PI / 4.0;
      double yaw_error = normalize_angle(target_yaw - current_yaw_);

      if (elapsed_phase > 2.5 || std::fabs(yaw_error) < 0.05) {
        // === 计算前进距离下界 D_min ===
        // 基于实际旋转角度 α，查表得几何下界 D_min = d₀/cos(α)
        // d₀=0.8m (触发距离), 保证旋转回避障前朝向后障碍物已通过
        double alpha_deg = std::fabs(normalize_angle(current_yaw_ - captured_yaw_))
                           * 180.0 / M_PI;
        if (alpha_deg <= 15.0)       explore_d_min_ = 0.83;
        else if (alpha_deg <= 30.0)  explore_d_min_ = 0.92;
        else if (alpha_deg <= 45.0)  explore_d_min_ = 1.13;
        else if (alpha_deg <= 60.0)  explore_d_min_ = 1.60;
        else if (alpha_deg <= 75.0)  explore_d_min_ = 3.09;
        else                         explore_d_min_ = 2.50;  // >75°用D_max兜底
        explore_d_max_   = 2.50;
        explore_start_x_ = current_x_;
        explore_start_y_ = current_y_;

        avoidance_state_ = AvoidanceState::EXPLORE;
        phase_start_time_ = now;
        avoidance_attempt_ = 0;
        explore_front_clear_count_ = 0;
        auto msg = std_msgs::msg::String();
        msg.data = "exploring";
        avoidance_state_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(),
          "Rotate done (α=%.1f°), D_min=%.2fm D_max=%.2fm, exploring forward",
          alpha_deg, explore_d_min_, explore_d_max_);
        return;
      }

      // 边旋转边前进：radius = 0.15/0.55 ≈ 0.27m
      cmd.linear.x = 0.15;
      cmd.angular.z = avoidance_direction_ * 0.55;
      cmd_vel_pub_->publish(cmd);
      return;
    }

    case AvoidanceState::EXPLORE:
    {
      // 计算已行进距离
      double dx = current_x_ - explore_start_x_;
      double dy = current_y_ - explore_start_y_;
      double distance_traveled = std::sqrt(dx * dx + dy * dy);

      // === 退出条件0: 侧方已脱离 且 至少前进0.5m → 直接绕过 (绕过D_min) ===
      // 问题3修复：长障碍物侧方脱离检测改用原始激光距离，
      // 配合此条件让机器人在侧方已空旷时立即退出EXPLORE
      if (distance_traveled >= 0.5 && is_obstacle_side_clear()) {
        avoidance_state_ = AvoidanceState::BYPASSED;
        auto msg = std_msgs::msg::String();
        msg.data = "bypassed";
        avoidance_state_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(),
          "Early exit: side clear at %.2fm, bypassed", distance_traveled);
        return;
      }

      // === 退出条件1: D_max 硬上限 (2.5m)，无条件退出 ===
      if (distance_traveled >= explore_d_max_) {
        avoidance_state_ = AvoidanceState::BYPASSED;
        auto msg = std_msgs::msg::String();
        msg.data = "bypassed";
        avoidance_state_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(),
          "D_max reached (%.2fm ≥ %.2fm), bypassed",
          distance_traveled, explore_d_max_);
        return;
      }

      // === 退出条件2: D_min 达标 且 侧方已脱离 ===
      if (distance_traveled >= explore_d_min_ && is_obstacle_side_clear()) {
        avoidance_state_ = AvoidanceState::BYPASSED;
        auto msg = std_msgs::msg::String();
        msg.data = "bypassed";
        avoidance_state_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(),
          "D_min met (%.2fm ≥ %.2fm) and side clear, bypassed",
          distance_traveled, explore_d_min_);
        return;
      }

      // === 退出条件3: 超时回退 (8s) ===
      if (elapsed_phase > 8.0) {
        if (!is_front_blocked()) {
          avoidance_state_ = AvoidanceState::BYPASSED;
          auto msg = std_msgs::msg::String();
          msg.data = "bypassed";
          avoidance_state_pub_->publish(msg);
          RCLCPP_INFO(this->get_logger(),
            "Explore timeout (front clear), bypassed");
        } else {
          avoidance_state_ = AvoidanceState::TIMEOUT_BACKOFF;
          phase_start_time_ = now;
          auto msg = std_msgs::msg::String();
          msg.data = "timeout_backoff";
          avoidance_state_pub_->publish(msg);
          RCLCPP_WARN(this->get_logger(),
            "Explore timeout (%.1fs), backoff", elapsed_phase);
        }
        return;
      }

      // === 继续绕行 ===
      // 默认宽弧线绕行：转弯半径 = 0.18 / 0.08 ≈ 2.25m
      double linear = 0.18;
      double angular = avoidance_direction_ * 0.08;

      if (is_front_blocked()) {
        avoidance_attempt_++;
        explore_front_clear_count_ = 0;  // 前方被挡，重置连续畅通计数
        // 前方再次被挡：渐进收紧转向但不停止前进
        angular = avoidance_direction_ * std::min(0.15 + avoidance_attempt_ * 0.18, 0.60);
        linear = 0.08;
        RCLCPP_INFO(this->get_logger(),
          "Front blocked during explore (attempt %d, dist=%.2fm), tightening turn to %.2f rad/s",
          avoidance_attempt_, distance_traveled, angular);
      } else {
        // 问题3修复：连续1秒（10帧）前方畅通 → 逐步衰减avoidance_attempt_
        // 防止avoidance_attempt永久递增导致转向越来越紧
        explore_front_clear_count_++;
        if (explore_front_clear_count_ >= 10 && avoidance_attempt_ > 0) {
          avoidance_attempt_--;
          explore_front_clear_count_ = 0;
          RCLCPP_DEBUG(this->get_logger(),
            "Explore front clear, decaying attempt to %d", avoidance_attempt_);
        }

        // 每3秒打印前进进度
        static int explore_log_counter = 0;
        if (++explore_log_counter % 30 == 0) {
          RCLCPP_INFO(this->get_logger(),
            "Exploring: dist=%.2fm D_min=%.2fm D_max=%.2fm side_clear=%s attempt=%d",
            distance_traveled, explore_d_min_, explore_d_max_,
            is_obstacle_side_clear() ? "yes" : "no", avoidance_attempt_);
        }
      }

      cmd.linear.x = linear;
      cmd.angular.z = angular;
      cmd_vel_pub_->publish(cmd);
      return;
    }

    case AvoidanceState::BYPASSED:
    {
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      cmd_vel_pub_->publish(cmd);

      avoidance_state_ = AvoidanceState::RESTORE_HEADING;
      phase_start_time_ = now;
      auto msg = std_msgs::msg::String();
      msg.data = "restoring_heading";
      avoidance_state_pub_->publish(msg);
      RCLCPP_INFO(this->get_logger(), "Restoring heading to %.1f°",
                  captured_yaw_ * 180.0 / M_PI);
      return;
    }

    case AvoidanceState::RESTORE_HEADING:
    {
      double yaw_error = normalize_angle(captured_yaw_ - current_yaw_);

      if (elapsed_phase > 5.0 || std::fabs(yaw_error) < 0.08) {
        // 简化：不再进入 RECALIBRATE 搜索人体，直接恢复 NORMAL
        // follower 节点负责重新捕获人体
        exit_avoidance();
        RCLCPP_INFO(this->get_logger(), "Heading restored, back to normal tracking");
        return;
      }

      cmd.linear.x = 0.0;
      cmd.angular.z = std::clamp(yaw_error * 1.5, -0.8, 0.8);
      cmd_vel_pub_->publish(cmd);
      return;
    }

    case AvoidanceState::RECALIBRATE:
    {
      // 简化 (2026-06-11)：不再搜索人体，立即恢复 NORMAL
      // 正常情况下 RESTORE_HEADING 完成后直接 exit_avoidance()
      // 此分支仅作安全兜底
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      cmd_vel_pub_->publish(cmd);
      exit_avoidance();
      RCLCPP_INFO(this->get_logger(), "Recalibrate fast-exit, back to normal");
      return;
    }

    case AvoidanceState::TIMEOUT_BACKOFF:
    {
      if (elapsed_phase > 2.0) {
        exit_avoidance();
        RCLCPP_INFO(this->get_logger(), "Backoff done, exiting avoidance");
        return;
      }

      cmd.linear.x = -0.1;
      cmd.angular.z = 0.0;
      cmd_vel_pub_->publish(cmd);
      return;
    }

    default:
      break;
  }
}

void BodyAvoider::exit_avoidance()
{
  avoidance_state_          = AvoidanceState::NORMAL;
  person_exemption_enabled_ = true;

  auto msg = std_msgs::msg::String();
  msg.data = "normal";
  avoidance_state_pub_->publish(msg);

  RCLCPP_INFO(this->get_logger(), "Avoidance exited, normal tracking resumed");
}

}  // namespace body_avoider

// =============================================================================
// main
// =============================================================================

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<body_avoider::BodyAvoider>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
