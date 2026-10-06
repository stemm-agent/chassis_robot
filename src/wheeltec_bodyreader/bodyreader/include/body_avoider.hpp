#ifndef BODY_AVOIDER_HPP_
#define BODY_AVOIDER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <bodyreader_msg/msg/bodyposture.hpp>
#include <vector>
#include <cmath>

namespace body_avoider
{

/**
 * @brief VFH (Vector Field Histogram) 避障节点
 *
 * 订阅 /cmd_vel_raw（跟随节点的期望速度）和 /scan（360°激光雷达），
 * 通过构建极坐标直方图找到安全方向，输出安全的 /cmd_vel。
 *
 * 避障状态机 (2026-06-11 新增):
 *   正常追踪 → 前方障碍物<0.5m → 旋转远离 → 探索前进 → 绕开判定
 *   → 恢复避障前朝向 → 重新标定人体 → 恢复正常追踪
 *
 * 算法参考：
 *   Borenstein & Koren (1991) "The Vector Field Histogram"
 *   Navigation2 DWB/Costmap 的局部避障思路
 */
class BodyAvoider : public rclcpp::Node
{
public:
  BodyAvoider();

  /**
   * @brief 避障状态机枚举
   */
  enum class AvoidanceState
  {
    NORMAL = 0,          // 正常追踪模式（VFH安全门控 + PD跟随）
    ROTATE_AWAY,         // 阶段1: 旋转远离障碍物
    EXPLORE,             // 阶段2: 缓慢前进探索绕行路径
    BYPASSED,            // 阶段3: 绕开成功（正前方 > safe_distance）
    RESTORE_HEADING,     // 阶段4: 旋转回避障前朝向
    RECALIBRATE,         // 阶段5: 等待人体重新标定
    TIMEOUT_BACKOFF      // 超时退避
  };

private:
  // === 回调 ===
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void cmd_raw_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void body_posture_callback(const bodyreader_msg::msg::Bodyposture::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // === VFH 核心 ===
  void build_polar_histogram(const sensor_msgs::msg::LaserScan &scan);
  void smooth_histogram();
  void find_valleys();
  int  select_best_valley(double desired_direction);
  void compute_safe_velocity(int best_valley, double desired_linear,
                              double desired_angular, double desired_direction);
  double compute_repulsive_angular(double desired_direction) const;

  // === 避障状态机 (2026-06-11) ===
  void enter_avoidance();
  void update_avoidance();
  void exit_avoidance();
  bool is_front_blocked() const;
  bool is_obstacle_side_clear() const;       // 侧方障碍物脱离检测（基于原始激光距离）
  double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion &q) const;

  // === 工具 ===
  int angle_to_bin(double angle) const;
  double bin_to_angle(int bin) const;
  double normalize_angle(double angle) const;

  // === 订阅/发布 ===
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_raw_sub_;
  rclcpp::Subscription<bodyreader_msg::msg::Bodyposture>::SharedPtr body_posture_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr avoidance_state_pub_;

  // === 定时器（10Hz 控制循环） ===
  rclcpp::TimerBase::SharedPtr control_timer_;
  void control_loop();

  // === VFH 数据结构 ===
  int hist_bins_;                    // 直方图扇形数量（默认72，对应5°分辨率）
  double bin_angle_;                 // 每个扇形的角度 (rad)
  std::vector<float> polar_hist_;    // 极坐标直方图 [0~1] 障碍物密度
  std::vector<float> smoothed_hist_; // 平滑后的直方图
  std::vector<float> bin_min_dist_;  // 每个bin的最近障碍物距离(m)，用于条件清零
  std::vector<std::pair<int,int>> valleys_; // 无障碍扇区 (start_bin, end_bin)

  // === 参数 ===
  double safe_distance_;       // 安全距离 (m)，障碍物距离 < 此值计为阻塞
  double safe_distance_stop_;  // 硬停距离 (m)，障碍物距离 < 此值强制停车
  double robot_radius_;        // 机器人半径 (m)，用于判断扇区宽度
  double hist_threshold_;      // 直方图阈值，密度超过此值视为阻塞
  int    smooth_width_;        // 平滑窗口半径（bin数）

  // 人体豁免参数
  double exemption_angle_deg_;   // 豁免半角（度），人体周围豁免扇形
  double exemption_angle_rad_;   // 同上（弧度）
  double exemption_dist_margin_; // 距离门控阈值(m): 测距 < person_dist-此值 → 真实障碍物

  // 速度约束
  double max_linear_speed_;
  double max_angular_speed_;
  double angular_p_gain_;      // 转向P增益
  double repulsive_gain_;      // 排斥力增益（越大越早避开）
  double min_valley_width_deg_; // 最小安全扇区宽度（度）

  // === 状态 ===
  geometry_msgs::msg::Twist desired_cmd_;  // follower 期望速度
  sensor_msgs::msg::LaserScan last_scan_;  // 最近一次扫描
  bool scan_received_;
  bool cmd_raw_received_;
  bool all_blocked_;           // 是否所有方向都被阻塞
  double closest_obstacle_;    // 最近的障碍物距离
  int    closest_obstacle_bin_;// 最近障碍物所在扇形

  // 人体追踪状态
  bool   person_valid_;        // 是否追踪到人体 (lock_status==2)
  double person_angle_;        // 人体方位角 (rad), [-pi, pi]
  double person_distance_;     // 人体距离 (m)

  // 逃逸状态（all_blocked 时用）
  bool        escape_active_;       // 是否处于逃逸模式
  rclcpp::Time escape_start_time_;  // 逃逸开始时间
  int         escape_phase_;        // 0=远离最近障碍, 1=看向人体方向

  // === 避障状态机 (2026-06-11) ===
  AvoidanceState avoidance_state_;    // 当前避障状态
  double captured_yaw_;               // 避障触发时记录的机器人朝向 (rad)
  double current_yaw_;                // 实时朝向 (rad)，来自 /odom
  double current_x_;                  // 实时X坐标 (m)，来自 /odom
  double current_y_;                  // 实时Y坐标 (m)，来自 /odom
  rclcpp::Time avoidance_start_time_; // 避障开始时间
  rclcpp::Time phase_start_time_;     // 当前阶段开始时间
  int    avoidance_attempt_;          // 探索重试次数
  int    avoidance_direction_;        // 避障旋转方向 (+1=右, -1=左)
  double front_angle_range_;          // 正前方检测半角 (rad)，默认 12°
  double bypass_clearance_distance_;  // 侧方障碍物脱离阈值 (m)，默认 0.3
  bool   person_exemption_enabled_;   // 人体豁免开关（避障中关闭）

  // EXPLORE 前进距离控制 (2026-06-11)
  double explore_start_x_;            // EXPLORE阶段起始X坐标
  double explore_start_y_;            // EXPLORE阶段起始Y坐标
  double explore_d_min_;              // 几何下界前进距离 (m)
  double explore_d_max_;              // 硬上限前进距离 (m)，默认2.5m
  int    explore_front_clear_count_;  // 连续前方无阻挡计数（衰减avoidance_attempt_用）
};

}  // namespace body_avoider

#endif  // BODY_AVOIDER_HPP_
