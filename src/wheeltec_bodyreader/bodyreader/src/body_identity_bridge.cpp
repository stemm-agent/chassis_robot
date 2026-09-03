#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "bodyreader_msg/msg/body.hpp"
#include "bodyreader_msg/msg/bodylist.hpp"
#include "bodyreader_msg/msg/bodyposture.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{
constexpr int kFollowMode = 2;
constexpr double kUpperHueWeight = 0.35;
constexpr double kLowerHueWeight = 0.45;
constexpr double kHeightRatioWeight = 0.20;
constexpr double kHueDistanceScale = 45.0;
constexpr double kHeightRatioScale = 2.5;
constexpr double kTrackingBboxMarginRatio = 0.08;
constexpr double kAstraNoBodyGraceS = 3.0;
constexpr double kAstraRestartCooldownS = 8.0;
constexpr int64_t kSourceResetQuarantineNs = 200000000LL;
constexpr int kAstraMaxRestartsPerFollowSession = 5;
constexpr double kGeometrySwitchMaxJumpM = 1.00;
constexpr double kGeometrySwitchConsistencyM = 0.15;
constexpr int kGeometrySwitchConfirmFrames = 4;
constexpr int64_t kGeometrySwitchMinDurationNs = 600000000LL;
constexpr double kGeometryDistanceEpsilonM = 1e-9;
}

class BodyIdentityBridge : public rclcpp::Node
{
public:
  BodyIdentityBridge()
  : Node("body_identity_bridge")
  {
    bodylist_topic_ = declare_parameter<std::string>("bodylist_topic", "/bodylist");
    validated_bodyposture_topic_ = declare_parameter<std::string>(
      "validated_bodyposture_topic", "/body_posture_yolo_validated");
    features_topic_ = declare_parameter<std::string>("features_topic", "/torso_trt_demo/features");
    lock_command_topic_ =
      declare_parameter<std::string>("lock_command_topic", "/torso_trt_demo/lock_command");
    state_topic_ = declare_parameter<std::string>("state_topic", "/body_identity_state");

    image_width_ = declare_parameter<double>("image_width", 640.0);
    horizontal_fov_rad_ = declare_parameter<double>("horizontal_fov_rad", 1.05);
    yolo_max_age_s_ = declare_parameter<double>("yolo_max_age_s", 0.8);
    body_max_age_s_ = declare_parameter<double>("body_max_age_s", 0.5);
    body_yolo_max_skew_s_ = std::max(
      0.0, declare_parameter<double>("body_yolo_max_skew_s", 0.15));
    initial_yolo_wait_s_ = declare_parameter<double>("initial_yolo_wait_s", 1.5);
    lock_command_period_s_ = declare_parameter<double>("lock_command_period_s", 0.8);
    match_max_angle_rad_ = declare_parameter<double>("match_max_angle_rad", 0.18);
    match_max_depth_diff_m_ = declare_parameter<double>("match_max_depth_diff_m", 0.85);
    match_accept_score_ = declare_parameter<double>("match_accept_score", 1.45);
    reacquire_accept_score_ = declare_parameter<double>("reacquire_accept_score", 1.65);
    appearance_accept_score_ = declare_parameter<double>("appearance_accept_score", 1.35);
    bound_appearance_accept_score_ =
      declare_parameter<double>("bound_appearance_accept_score", 1.15);
    bound_body_bbox_margin_ratio_ = std::max(
      kTrackingBboxMarginRatio,
      declare_parameter<double>(
        "bound_body_bbox_margin_ratio", kTrackingBboxMarginRatio));
    reacquire_candidate_margin_ =
      declare_parameter<double>("reacquire_candidate_margin", 0.18);
    body_pair_candidate_margin_ =
      std::max(0.0, declare_parameter<double>("body_pair_candidate_margin", 0.12));
    reacquire_confirm_frames_ = static_cast<int>(
      declare_parameter<int>("reacquire_confirm_frames", 4));
    reacquire_confirm_frames_ = std::max(1, reacquire_confirm_frames_);
    initial_confirm_frames_ = static_cast<int>(
      declare_parameter<int>("initial_confirm_frames", 4));
    initial_confirm_frames_ = std::max(1, initial_confirm_frames_);
    require_yolo_for_initial_lock_ =
      declare_parameter<bool>("require_yolo_for_initial_lock", true);

    bodylist_sub_ = create_subscription<bodyreader_msg::msg::Bodylist>(
      bodylist_topic_, 5,
      std::bind(&BodyIdentityBridge::bodylist_callback, this, std::placeholders::_1));
    raw_body_count_sub_ = create_subscription<std_msgs::msg::Int8>(
      "/body_main/raw_body_count", rclcpp::QoS(1).best_effort(),
      std::bind(&BodyIdentityBridge::raw_body_count_callback, this, std::placeholders::_1));
    features_sub_ = create_subscription<std_msgs::msg::String>(
      features_topic_, 5,
      std::bind(&BodyIdentityBridge::features_callback, this, std::placeholders::_1));
    mode_sub_ = create_subscription<std_msgs::msg::Int8>(
      "/mode", rclcpp::QoS(1).best_effort(),
      std::bind(&BodyIdentityBridge::mode_callback, this, std::placeholders::_1));

    validated_bodyposture_pub_ = create_publisher<bodyreader_msg::msg::Bodyposture>(
      validated_bodyposture_topic_, 5);
    lock_command_pub_ = create_publisher<std_msgs::msg::String>(lock_command_topic_, 5);
    state_pub_ = create_publisher<std_msgs::msg::String>(state_topic_, 5);
    astra_restart_pub_ = create_publisher<std_msgs::msg::Empty>(
      "/body_main/restart_body_stream", 1);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&BodyIdentityBridge::control_loop, this));

    start_time_ = now();
    last_bodylist_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_features_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_lock_command_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_validated_body_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_raw_body_count_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    yolo_without_body_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_astra_restart_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());

    RCLCPP_INFO(
      get_logger(),
      "BodyIdentityBridge ready: body=%s features=%s exact_validated=%s "
      "require_yolo_initial=%s initial_confirm_frames=%d "
      "reacquire_confirm_frames=%d yolo_margin=%.2f body_margin=%.2f max_skew=%.3fs "
      "bound_appearance=%.2f bbox_margin=%.2f/%.2f "
      "geometry_switch=%.2fm/%dframes/%.2fm/%.2fs",
      bodylist_topic_.c_str(), features_topic_.c_str(), validated_bodyposture_topic_.c_str(),
      require_yolo_for_initial_lock_ ? "true" : "false",
      initial_confirm_frames_, reacquire_confirm_frames_, reacquire_candidate_margin_,
      body_pair_candidate_margin_, body_yolo_max_skew_s_,
      bound_appearance_accept_score_, kTrackingBboxMarginRatio,
      bound_body_bbox_margin_ratio_, kGeometrySwitchMaxJumpM,
      kGeometrySwitchConfirmFrames, kGeometrySwitchConsistencyM,
      static_cast<double>(kGeometrySwitchMinDurationNs) * 1e-9);
  }

private:
  struct BodyCandidate
  {
    int id{0};
    double angle{0.0};
    double depth_m{0.0};
    bool valid_depth{false};
    double center_x{0.0};
    double center_y{0.0};
    double center_z{0.0};
    double image_center_x{0.0};
    double image_center_y{0.0};
    bool valid_image_center{false};
  };

  struct YoloTarget
  {
    int track_id{-1};
    bool locked{false};
    double center_x{0.0};
    double center_y{0.0};
    double bbox_x1{0.0};
    double bbox_y1{0.0};
    double bbox_x2{0.0};
    double bbox_y2{0.0};
    bool valid_bbox{false};
    double score{0.0};
    double depth_m{0.0};
    double height_ratio{0.0};
    double upper_hue{-1.0};
    double lower_hue{-1.0};
  };

  struct Match
  {
    bool valid{false};
    BodyCandidate body;
    YoloTarget yolo;
    double score{std::numeric_limits<double>::infinity()};
    double second_score{std::numeric_limits<double>::infinity()};
  };

  struct SignatureChoice
  {
    const YoloTarget * target{nullptr};
    double best_score{std::numeric_limits<double>::infinity()};
    double second_score{std::numeric_limits<double>::infinity()};
    bool ambiguous{false};
  };

  static double normalize_angle(double angle)
  {
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }
    return angle;
  }

  static double clamp_value(double value, double low, double high)
  {
    return std::max(low, std::min(value, high));
  }

  static bool find_double(const std::string & text, const std::string & key, double & value)
  {
    const size_t pos = text.find(key);
    if (pos == std::string::npos) {
      return false;
    }
    const char * start = text.c_str() + pos + key.size();
    char * end = nullptr;
    value = std::strtod(start, &end);
    return end != start;
  }

  static bool find_int(const std::string & text, const std::string & key, int & value)
  {
    double parsed = 0.0;
    if (!find_double(text, key, parsed)) {
      return false;
    }
    value = static_cast<int>(std::lround(parsed));
    return true;
  }

  static bool find_bool(const std::string & text, const std::string & key, bool & value)
  {
    const size_t pos = text.find(key);
    if (pos == std::string::npos) {
      return false;
    }
    const size_t start = pos + key.size();
    if (text.compare(start, 4, "true") == 0) {
      value = true;
      return true;
    }
    if (text.compare(start, 5, "false") == 0) {
      value = false;
      return true;
    }
    return false;
  }

  static bool find_array2(
    const std::string & text,
    const std::string & key,
    double & first,
    double & second)
  {
    const size_t pos = text.find(key);
    if (pos == std::string::npos) {
      return false;
    }
    const char * start = text.c_str() + pos + key.size();
    char * end = nullptr;
    first = std::strtod(start, &end);
    if (end == start) {
      return false;
    }
    const char * comma = std::strchr(end, ',');
    if (comma == nullptr) {
      return false;
    }
    start = comma + 1;
    second = std::strtod(start, &end);
    return end != start;
  }

  static bool find_array4(
    const std::string & text,
    const std::string & key,
    double & first,
    double & second,
    double & third,
    double & fourth)
  {
    const size_t pos = text.find(key);
    if (pos == std::string::npos) {
      return false;
    }
    const char * start = text.c_str() + pos + key.size();
    char * end = nullptr;
    double * values[] = {&first, &second, &third, &fourth};
    for (size_t i = 0; i < 4; ++i) {
      *values[i] = std::strtod(start, &end);
      if (end == start) {
        return false;
      }
      if (i < 3) {
        const char * comma = std::strchr(end, ',');
        if (comma == nullptr) {
          return false;
        }
        start = comma + 1;
      }
    }
    return true;
  }

  static size_t matching_brace(const std::string & text, size_t open_pos)
  {
    int depth = 0;
    for (size_t i = open_pos; i < text.size(); ++i) {
      if (text[i] == '{') {
        ++depth;
      } else if (text[i] == '}') {
        --depth;
        if (depth == 0) {
          return i;
        }
      }
    }
    return std::string::npos;
  }

  static double hue_distance(double a, double b)
  {
    if (a < 0.0 || b < 0.0) {
      return 45.0;
    }
    double diff = std::fabs(a - b);
    diff = std::min(diff, 180.0 - diff);
    return diff;
  }

  void raw_body_count_callback(const std_msgs::msg::Int8::SharedPtr msg)
  {
    raw_body_count_ = static_cast<int>(msg->data);
    last_raw_body_count_time_ = now();
  }

  void bodylist_callback(const bodyreader_msg::msg::Bodylist::SharedPtr msg)
  {
    const auto t = now();
    if (t.nanoseconds() < source_quarantine_until_ns_) {
      return;
    }
    const int count = std::min(
      std::max(0, static_cast<int>(msg->count)),
      static_cast<int>(msg->bodies.size()));
    bodies_.clear();
    bodies_.reserve(static_cast<size_t>(count));
    for (int i = 0; i < count; ++i) {
      const auto & body = msg->bodies[static_cast<size_t>(i)];
      const double center_x = static_cast<double>(body.centerofmass.x);
      const double center_y = static_cast<double>(body.centerofmass.y);
      const double center_z = static_cast<double>(body.centerofmass.z);
      if (!std::isfinite(center_x) || !std::isfinite(center_y) ||
          !std::isfinite(center_z) || center_z <= 100.0)
      {
        continue;
      }
      BodyCandidate candidate;
      candidate.id = body.bodyid;
      candidate.depth_m = center_z * 0.001;
      candidate.valid_depth = candidate.depth_m > 0.1;
      candidate.angle = std::atan2(center_x, center_z);
      candidate.center_x = center_x * 0.001;
      candidate.center_y = center_y * 0.001;
      candidate.center_z = center_z * 0.001;
      double min_x = std::numeric_limits<double>::infinity();
      double min_y = std::numeric_limits<double>::infinity();
      double max_x = -std::numeric_limits<double>::infinity();
      double max_y = -std::numeric_limits<double>::infinity();
      int visible_joint_count = 0;
      for (const auto & joint : body.joints) {
        const double x = static_cast<double>(joint.depthposition.x);
        const double y = static_cast<double>(joint.depthposition.y);
        if (!std::isfinite(x) || !std::isfinite(y) || x <= 0.0 || y <= 0.0) {
          continue;
        }
        min_x = std::min(min_x, x);
        min_y = std::min(min_y, y);
        max_x = std::max(max_x, x);
        max_y = std::max(max_y, y);
        ++visible_joint_count;
      }
      if (visible_joint_count >= 3 && max_x > min_x && max_y > min_y) {
        candidate.image_center_x = (min_x + max_x) * 0.5;
        candidate.image_center_y = (min_y + max_y) * 0.5;
        candidate.valid_image_center = true;
      }
      bodies_.push_back(candidate);
    }
    last_bodylist_time_ = t;
  }

  void features_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (now().nanoseconds() < source_quarantine_until_ns_) {
      return;
    }
    int stamp_sec = 0;
    int stamp_nanosec = 0;
    const bool has_source_stamp =
      find_int(msg->data, "\"sec\":", stamp_sec) &&
      find_int(msg->data, "\"nanosec\":", stamp_nanosec) &&
      stamp_sec >= 0 && stamp_nanosec >= 0;
    const int64_t source_stamp_ns = has_source_stamp ?
      static_cast<int64_t>(stamp_sec) * 1000000000LL + stamp_nanosec : 0;
    if (source_stamp_ns > 0 && last_feature_source_stamp_ns_ > 0 &&
        source_stamp_ns <= last_feature_source_stamp_ns_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Ignore duplicate/out-of-order YOLO feature frame stamp=%lld last=%lld",
        static_cast<long long>(source_stamp_ns),
        static_cast<long long>(last_feature_source_stamp_ns_));
      return;
    }
    if (source_stamp_ns > 0) {
      last_feature_source_stamp_ns_ = source_stamp_ns;
    }
    ++feature_generation_;
    feature_has_valid_person_bbox_ = false;
    yolo_targets_.clear();
    locked_track_id_ = -1;
    locked_yolo_visible_ = false;
    find_int(msg->data, "\"locked_track_id\":", locked_track_id_);
    find_bool(msg->data, "\"locked_visible\":", locked_yolo_visible_);

    const size_t array_pos = msg->data.find("\"targets\":[");
    if (array_pos == std::string::npos) {
      last_features_time_ = now();
      return;
    }

    size_t pos = msg->data.find('{', array_pos);
    while (pos != std::string::npos) {
      const size_t end = matching_brace(msg->data, pos);
      if (end == std::string::npos) {
        break;
      }
      YoloTarget target;
      const std::string object = msg->data.substr(pos, end - pos + 1);
      find_int(object, "\"track_id\":", target.track_id);
      find_bool(object, "\"locked\":", target.locked);
      find_double(object, "\"score\":", target.score);
      find_array2(object, "\"center\":[", target.center_x, target.center_y);
      target.valid_bbox = find_array4(
        object, "\"person_bbox\":[", target.bbox_x1, target.bbox_y1,
        target.bbox_x2, target.bbox_y2) &&
        target.bbox_x2 > target.bbox_x1 && target.bbox_y2 > target.bbox_y1;
      feature_has_valid_person_bbox_ =
        feature_has_valid_person_bbox_ ||
        (target.valid_bbox &&
        std::isfinite(target.bbox_x1) && std::isfinite(target.bbox_y1) &&
        std::isfinite(target.bbox_x2) && std::isfinite(target.bbox_y2));
      find_double(object, "\"depth_m\":", target.depth_m);
      find_double(object, "\"height_ratio_smooth\":", target.height_ratio);
      find_double(object, "\"dominant_hue\":", target.upper_hue);
      find_double(object, "\"lower_dominant_hue\":", target.lower_hue);
      if (target.track_id > 0) {
        yolo_targets_.push_back(target);
      }
      pos = msg->data.find('{', end + 1);
    }
    last_features_time_ = now();
  }

  bool body_recent(const rclcpp::Time & t) const
  {
    return !bodies_.empty() && (t - last_bodylist_time_).seconds() <= body_max_age_s_;
  }

  bool yolo_recent(const rclcpp::Time & t) const
  {
    return !yolo_targets_.empty() && (t - last_features_time_).seconds() <= yolo_max_age_s_;
  }

  bool body_yolo_synchronized() const
  {
    return last_bodylist_time_.nanoseconds() != 0 &&
           last_features_time_.nanoseconds() != 0 &&
           std::fabs((last_bodylist_time_ - last_features_time_).seconds()) <=
           body_yolo_max_skew_s_;
  }

  bool raw_body_empty_recent(const rclcpp::Time & t) const
  {
    return raw_body_count_ == 0 &&
           last_raw_body_count_time_.nanoseconds() != 0 &&
           (t - last_raw_body_count_time_).seconds() <= body_max_age_s_;
  }

  double yolo_angle(const YoloTarget & target) const
  {
    const double width = std::max(1.0, image_width_);
    const double normalized = (target.center_x - width * 0.5) / (width * 0.5);
    return normalized * horizontal_fov_rad_ * 0.5;
  }

  double appearance_score(const YoloTarget & target) const
  {
    if (!has_signature_) {
      return 0.0;
    }

    double weighted_score = 0.0;
    double weight_sum = 0.0;
    const auto add_hue = [&weighted_score, &weight_sum](
      double target_hue, double signature_hue, double weight)
      {
        if (target_hue < 0.0 || signature_hue < 0.0) {
          return;
        }
        weighted_score +=
          weight * hue_distance(target_hue, signature_hue) / kHueDistanceScale;
        weight_sum += weight;
      };

    add_hue(target.upper_hue, signature_.upper_hue, kUpperHueWeight);
    add_hue(target.lower_hue, signature_.lower_hue, kLowerHueWeight);
    if (target.height_ratio > 0.0 && signature_.height_ratio > 0.0) {
      weighted_score += kHeightRatioWeight *
        std::fabs(target.height_ratio - signature_.height_ratio) * kHeightRatioScale;
      weight_sum += kHeightRatioWeight;
    }

    if (weight_sum <= 1e-6) {
      return std::numeric_limits<double>::infinity();
    }
    return weighted_score / weight_sum;
  }

  bool is_bound_yolo_track(const YoloTarget & target) const
  {
    return locked_identity_track_id_ > 0 &&
           target.track_id == locked_identity_track_id_;
  }

  bool tracking_appearance_matches(const YoloTarget & target) const
  {
    if (!has_signature_) {
      return true;
    }
    const double accept_score = is_bound_yolo_track(target) ?
      bound_appearance_accept_score_ : appearance_accept_score_;
    return appearance_score(target) <= accept_score;
  }

  double pair_score(const BodyCandidate & body, const YoloTarget & target) const
  {
    if (!body.valid_image_center || !target.valid_bbox) {
      return std::numeric_limits<double>::infinity();
    }

    const double bbox_width = target.bbox_x2 - target.bbox_x1;
    const double bbox_height = target.bbox_y2 - target.bbox_y1;
    const double bbox_margin_ratio = is_bound_yolo_track(target) ?
      bound_body_bbox_margin_ratio_ : kTrackingBboxMarginRatio;
    const double margin_x = bbox_width * bbox_margin_ratio;
    const double margin_y = bbox_height * bbox_margin_ratio;
    if (body.image_center_x < target.bbox_x1 - margin_x ||
        body.image_center_x > target.bbox_x2 + margin_x ||
        body.image_center_y < target.bbox_y1 - margin_y ||
        body.image_center_y > target.bbox_y2 + margin_y) {
      return std::numeric_limits<double>::infinity();
    }
    const double angle =
      std::fabs(normalize_angle(body.angle - yolo_angle(target))) /
      std::max(0.01, match_max_angle_rad_);
    double depth = 0.0;
    if (body.valid_depth && target.depth_m > 0.1) {
      depth = std::fabs(body.depth_m - target.depth_m) /
        std::max(0.05, match_max_depth_diff_m_);
    }
    const double appearance =
      is_bound_yolo_track(target) ? 0.0 : appearance_score(target);
    return 0.65 * angle + 0.25 * depth + 0.10 * appearance;
  }

  Match match_yolo_to_body(const YoloTarget & yolo) const
  {
    Match best;
    for (const auto & body : bodies_) {
      const double score = pair_score(body, yolo);
      if (score < best.score) {
        best.second_score = best.score;
        best.valid = true;
        best.body = body;
        best.yolo = yolo;
        best.score = score;
      } else if (score < best.second_score) {
        best.second_score = score;
      }
    }
    if (best.valid && std::isfinite(best.second_score) &&
        best.second_score - best.score < body_pair_candidate_margin_)
    {
      best.valid = false;
    }
    return best;
  }

  Match match_yolo_to_body_id(const YoloTarget & yolo, int body_id) const
  {
    Match match;
    if (body_id <= 0) {
      return match;
    }
    for (const auto & body : bodies_) {
      if (body.id != body_id) {
        continue;
      }
      const double score = pair_score(body, yolo);
      if (!std::isfinite(score)) {
        return match;
      }
      match.valid = true;
      match.body = body;
      match.yolo = yolo;
      match.score = score;
      return match;
    }
    return match;
  }

  Match match_yolo_with_current_carrier(
    const YoloTarget & yolo, double accept_score) const
  {
    Match match = match_yolo_to_body(yolo);
    const Match current_carrier = match_yolo_to_body_id(
      yolo, current_geometry_body_id_);
    if (current_carrier.valid && current_carrier.score <= accept_score) {
      return current_carrier;
    }
    return match;
  }

  void reset_geometry_switch_confirmation()
  {
    pending_geometry_track_id_ = -1;
    pending_geometry_body_id_ = 0;
    pending_geometry_count_ = 0;
    pending_geometry_last_feature_generation_ = 0;
    pending_geometry_since_ns_ = 0;
    pending_geometry_last_evidence_ns_ = 0;
    pending_geometry_anchor_x_m_ = 0.0;
    pending_geometry_anchor_z_m_ = 0.0;
  }

  void start_geometry_switch_confirmation(const Match & match, int64_t evidence_ns)
  {
    pending_geometry_track_id_ = match.yolo.track_id;
    pending_geometry_body_id_ = match.body.id;
    pending_geometry_count_ = 1;
    pending_geometry_last_feature_generation_ = feature_generation_;
    pending_geometry_since_ns_ = evidence_ns;
    pending_geometry_last_evidence_ns_ = evidence_ns;
    pending_geometry_anchor_x_m_ = match.body.center_x;
    pending_geometry_anchor_z_m_ = match.body.center_z;
  }

  bool geometry_switch_ready(const Match & match)
  {
    if (!has_last_exact_geometry_) {
      reset_geometry_switch_confirmation();
      return true;
    }

    const double jump_m = std::hypot(
      match.body.center_x - last_exact_x_m_,
      match.body.center_z - last_exact_z_m_);
    if (jump_m <= kGeometrySwitchMaxJumpM + kGeometryDistanceEpsilonM) {
      if (pending_geometry_count_ > 0) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "GEOMETRY_SWITCH_CANCEL reason=trusted_geometry_recovered "
          "track=%d pending_body=%d count=%d",
          pending_geometry_track_id_, pending_geometry_body_id_,
          pending_geometry_count_);
      }
      reset_geometry_switch_confirmation();
      return true;
    }

    if (pending_geometry_count_ > 0 &&
        pending_geometry_last_feature_generation_ >= feature_generation_)
    {
      return false;
    }

    const int64_t evidence_ns = last_features_time_.nanoseconds();
    const bool same_candidate =
      pending_geometry_count_ > 0 &&
      pending_geometry_track_id_ == match.yolo.track_id &&
      pending_geometry_body_id_ == match.body.id &&
      evidence_ns > 0 && evidence_ns >= pending_geometry_since_ns_;
    if (!same_candidate) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "GEOMETRY_SWITCH_CANDIDATE track=%d current_body=%d candidate_body=%d "
        "jump=%.3fm pair=%.3f reason=%s",
        match.yolo.track_id, current_geometry_body_id_, match.body.id,
        jump_m, match.score,
        pending_geometry_count_ > 0 ? "candidate_changed" : "large_jump");
      reset_geometry_switch_confirmation();
      start_geometry_switch_confirmation(match, evidence_ns);
      return false;
    }

    if (evidence_ns <= pending_geometry_last_evidence_ns_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "GEOMETRY_SWITCH_CANDIDATE track=%d current_body=%d candidate_body=%d "
        "jump=%.3fm reason=evidence_time_reset",
        match.yolo.track_id, current_geometry_body_id_, match.body.id, jump_m);
      reset_geometry_switch_confirmation();
      start_geometry_switch_confirmation(match, evidence_ns);
      return false;
    }

    const double anchor_distance_m = std::hypot(
      match.body.center_x - pending_geometry_anchor_x_m_,
      match.body.center_z - pending_geometry_anchor_z_m_);
    if (anchor_distance_m >
        kGeometrySwitchConsistencyM + kGeometryDistanceEpsilonM)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "GEOMETRY_SWITCH_CANDIDATE track=%d current_body=%d candidate_body=%d "
        "jump=%.3fm spread=%.3fm reason=inconsistent_candidate",
        match.yolo.track_id, current_geometry_body_id_, match.body.id,
        jump_m, anchor_distance_m);
      reset_geometry_switch_confirmation();
      start_geometry_switch_confirmation(match, evidence_ns);
      return false;
    }

    pending_geometry_last_feature_generation_ = feature_generation_;
    pending_geometry_last_evidence_ns_ = evidence_ns;
    ++pending_geometry_count_;
    const int64_t elapsed_ns =
      pending_geometry_last_evidence_ns_ - pending_geometry_since_ns_;
    if (pending_geometry_count_ < kGeometrySwitchConfirmFrames ||
        elapsed_ns < kGeometrySwitchMinDurationNs)
    {
      return false;
    }

    RCLCPP_INFO(
      get_logger(),
      "GEOMETRY_SWITCH_CONFIRMED track=%d old_body=%d new_body=%d "
      "jump=%.3fm spread=%.3fm frames=%d duration=%.3fs pair=%.3f",
      match.yolo.track_id, current_geometry_body_id_, match.body.id,
      jump_m, anchor_distance_m, pending_geometry_count_,
      static_cast<double>(elapsed_ns) * 1e-9, match.score);
    return true;
  }

  const YoloTarget * identity_yolo_target() const
  {
    if (locked_identity_track_id_ <= 0) {
      return nullptr;
    }
    for (const auto & target : yolo_targets_) {
      if (target.track_id == locked_identity_track_id_) {
        return &target;
      }
    }
    return nullptr;
  }

  void log_identity_rejection(
    const char * reason,
    const YoloTarget * target,
    const Match * match)
  {
    const auto t = now();
    const double inf = std::numeric_limits<double>::infinity();
    const double body_age = last_bodylist_time_.nanoseconds() == 0 ?
      inf : (t - last_bodylist_time_).seconds();
    const double yolo_age = last_features_time_.nanoseconds() == 0 ?
      inf : (t - last_features_time_).seconds();
    const double skew =
      last_bodylist_time_.nanoseconds() == 0 || last_features_time_.nanoseconds() == 0 ?
      inf : std::fabs((last_bodylist_time_ - last_features_time_).seconds());
    const double appearance = target == nullptr ? inf : appearance_score(*target);
    const bool has_pair = match != nullptr && std::isfinite(match->score);
    const double angle_diff = has_pair ?
      std::fabs(normalize_angle(match->body.angle - yolo_angle(match->yolo))) : inf;
    const double depth_diff = has_pair && match->body.valid_depth && match->yolo.depth_m > 0.1 ?
      std::fabs(match->body.depth_m - match->yolo.depth_m) : inf;

    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "IDENTITY_REJECT reason=%s locked_track=%d candidate_track=%d bound=%s "
      "appearance=%.3f/%.3f pair_valid=%s pair=%.3f second=%.3f "
      "angle_diff=%.3f depth_diff=%.3f bbox=%s bodies=%zu yolo=%zu "
      "age=%.3f/%.3f skew=%.3f/%.3f",
      reason, locked_identity_track_id_, target == nullptr ? -1 : target->track_id,
      target != nullptr && is_bound_yolo_track(*target) ? "true" : "false",
      appearance,
      target != nullptr && is_bound_yolo_track(*target) ?
      bound_appearance_accept_score_ : appearance_accept_score_,
      match != nullptr && match->valid ? "true" : "false",
      has_pair ? match->score : inf,
      match != nullptr ? match->second_score : inf,
      angle_diff, depth_diff,
      target != nullptr && target->valid_bbox ? "true" : "false",
      bodies_.size(), yolo_targets_.size(), body_age, yolo_age, skew,
      body_yolo_max_skew_s_);
  }

  void publish_exact_invalid_once(const std::string & state, bool force = false)
  {
    if (!force && !exact_stream_valid_) {
      publish_state(state);
      return;
    }
    bodyreader_msg::msg::Bodyposture invalid;
    invalid.lock_status = 0;
    validated_bodyposture_pub_->publish(invalid);
    exact_stream_valid_ = false;
    publish_state(state);
  }

  void publish_exact_observation(const Match & match, const std::string & state)
  {
    if (!match.valid || feature_generation_ == 0 ||
        feature_generation_ == last_exact_feature_generation_)
    {
      publish_state(state);
      return;
    }
    if (!geometry_switch_ready(match)) {
      publish_exact_invalid_once("identity_geometry_switch_confirming");
      return;
    }

    bodyreader_msg::msg::Bodyposture observation;
    observation.bodyid = static_cast<int16_t>(match.body.id);
    observation.centerofmass_x = static_cast<float>(match.body.center_x * 1000.0);
    observation.centerofmass_y = static_cast<float>(match.body.center_y * 1000.0);
    observation.centerofmass_z = static_cast<float>(match.body.center_z * 1000.0);
    observation.lock_status = 2;
    validated_bodyposture_pub_->publish(observation);
    exact_stream_valid_ = true;
    last_exact_feature_generation_ = feature_generation_;
    last_validated_body_time_ = now();

    if (current_geometry_body_id_ != match.body.id) {
      RCLCPP_INFO(
        get_logger(),
        "YOLO identity track=%d selected skeleton=%d (previous=%d) pair=%.3f; "
        "skeleton ID is diagnostic only",
        match.yolo.track_id, match.body.id, current_geometry_body_id_, match.score);
    }
    current_geometry_body_id_ = match.body.id;
    last_exact_x_m_ = match.body.center_x;
    last_exact_z_m_ = match.body.center_z;
    has_last_exact_geometry_ = true;
    reset_geometry_switch_confirmation();
    publish_state(state);
  }

  SignatureChoice best_signature_yolo() const
  {
    SignatureChoice choice;
    if (!has_signature_) {
      return choice;
    }

    // When the original locked track is still present, keep it authoritative.
    // Duplicate detections of the same person must not make the identity ambiguous,
    // and the bound-track threshold should remain consistent during body-ID rebinding.
    for (const auto & target : yolo_targets_) {
      if (!is_bound_yolo_track(target)) {
        continue;
      }
      choice.best_score = appearance_score(target);
      if (choice.best_score <= bound_appearance_accept_score_) {
        choice.target = &target;
      }
      return choice;
    }

    for (const auto & target : yolo_targets_) {
      const double score = appearance_score(target);
      if (score < choice.best_score) {
        choice.second_score = choice.best_score;
        choice.best_score = score;
        choice.target = &target;
      } else if (score < choice.second_score) {
        choice.second_score = score;
      }
    }
    if (choice.target == nullptr || choice.best_score > appearance_accept_score_) {
      choice.target = nullptr;
      return choice;
    }
    choice.ambiguous = std::isfinite(choice.second_score) &&
      choice.second_score - choice.best_score < reacquire_candidate_margin_;
    if (choice.ambiguous) {
      choice.target = nullptr;
    }
    return choice;
  }

  void reset_rebind_confirmation()
  {
    pending_track_id_ = -1;
    pending_confirm_count_ = 0;
    pending_feature_generation_ = 0;
  }

  void lock_yolo_track(int track_id)
  {
    if (track_id <= 0 || track_id == commanded_yolo_track_id_) {
      return;
    }
    const auto t = now();
    if ((t - last_lock_command_time_).seconds() < lock_command_period_s_) {
      return;
    }
    std_msgs::msg::String msg;
    msg.data = "track:" + std::to_string(track_id);
    lock_command_pub_->publish(msg);
    commanded_yolo_track_id_ = track_id;
    last_lock_command_time_ = t;
  }

  void bind_yolo_track(int track_id)
  {
    if (track_id <= 0) {
      return;
    }
    desired_yolo_track_id_ = track_id;
    locked_identity_track_id_ = track_id;
    lock_yolo_track(track_id);
  }

  bool confirm_rebind(const Match & match)
  {
    if (match.yolo.track_id != pending_track_id_) {
      pending_track_id_ = match.yolo.track_id;
      pending_confirm_count_ = 0;
      pending_feature_generation_ = 0;
      RCLCPP_INFO(
        get_logger(),
        "identity_rebind_candidate: old_track=%d candidate_track=%d skeleton=%d "
        "appearance_score=%.3f pair_score=%.3f required_frames=%d",
        locked_identity_track_id_, match.yolo.track_id, match.body.id,
        appearance_score(match.yolo), match.score, reacquire_confirm_frames_);
    }

    if (pending_feature_generation_ == feature_generation_) {
      publish_state("identity_rebind_confirming");
      return false;
    }
    pending_feature_generation_ = feature_generation_;
    ++pending_confirm_count_;
    if (pending_confirm_count_ < reacquire_confirm_frames_) {
      publish_state("identity_rebind_confirming");
      return false;
    }

    RCLCPP_INFO(
      get_logger(),
      "identity_rebind_confirmed: old_track=%d new_track=%d skeleton=%d frames=%d; "
      "identity remains YOLO-owned",
      locked_identity_track_id_, match.yolo.track_id, match.body.id, pending_confirm_count_);
    bind_yolo_track(match.yolo.track_id);
    reset_rebind_confirmation();
    return true;
  }

  const YoloTarget * initial_yolo_candidate() const
  {
    if (pending_track_id_ > 0) {
      for (const auto & target : yolo_targets_) {
        if (target.track_id == pending_track_id_) {
          return &target;
        }
      }
      return nullptr;
    }
    if (locked_yolo_visible_ && locked_track_id_ > 0) {
      for (const auto & target : yolo_targets_) {
        if (target.track_id == locked_track_id_) {
          return &target;
        }
      }
    }
    return yolo_targets_.size() == 1 ? &yolo_targets_.front() : nullptr;
  }

  bool confirm_initial_match(const Match & match)
  {
    if (match.yolo.track_id != pending_track_id_) {
      pending_track_id_ = match.yolo.track_id;
      pending_confirm_count_ = 0;
      pending_feature_generation_ = 0;
      RCLCPP_INFO(
        get_logger(),
        "identity_initial_candidate: track=%d skeleton=%d pair_score=%.3f required_frames=%d",
        match.yolo.track_id, match.body.id, match.score, initial_confirm_frames_);
    }

    if (pending_feature_generation_ == feature_generation_) {
      publish_state("identity_initial_confirming");
      return false;
    }
    pending_feature_generation_ = feature_generation_;
    ++pending_confirm_count_;
    if (pending_confirm_count_ < initial_confirm_frames_) {
      publish_state("identity_initial_confirming");
      return false;
    }

    RCLCPP_INFO(
      get_logger(),
      "identity_initial_confirmed: track=%d skeleton=%d frames=%d; "
      "skeleton ID was not part of confirmation",
      match.yolo.track_id, match.body.id, pending_confirm_count_);
    return true;
  }

  void capture_initial_signature(const YoloTarget & target)
  {
    signature_ = YoloTarget{};
    signature_.upper_hue = target.upper_hue;
    signature_.lower_hue = target.lower_hue;
    signature_.height_ratio = target.height_ratio;
    has_signature_ =
      signature_.upper_hue >= 0.0 ||
      signature_.lower_hue >= 0.0 ||
      signature_.height_ratio > 0.0;
    bind_yolo_track(target.track_id);
    reset_rebind_confirmation();

    RCLCPP_INFO(
      get_logger(),
      "identity_signature_locked: track=%d upper_hue=%.1f lower_hue=%.1f height_ratio=%.3f "
      "usable=%s weights=upper:%.2f,lower:%.2f,height:%.2f",
      target.track_id, signature_.upper_hue, signature_.lower_hue,
      signature_.height_ratio, has_signature_ ? "true" : "false",
      kUpperHueWeight, kLowerHueWeight, kHeightRatioWeight);
  }

  void clear_yolo_lock()
  {
    std_msgs::msg::String msg;
    msg.data = "clear";
    lock_command_pub_->publish(msg);
  }

  void reset_identity(const std::string & state)
  {
    bodies_.clear();
    yolo_targets_.clear();
    signature_ = YoloTarget{};
    has_signature_ = false;
    has_identity_ = false;
    locked_yolo_visible_ = false;
    current_geometry_body_id_ = 0;
    has_last_exact_geometry_ = false;
    last_exact_x_m_ = 0.0;
    last_exact_z_m_ = 0.0;
    reset_geometry_switch_confirmation();
    locked_track_id_ = -1;
    locked_identity_track_id_ = -1;
    desired_yolo_track_id_ = -1;
    commanded_yolo_track_id_ = -1;
    feature_has_valid_person_bbox_ = false;
    feature_generation_ = 0;
    last_feature_source_stamp_ns_ = 0;
    source_quarantine_until_ns_ =
      now().nanoseconds() + kSourceResetQuarantineNs;
    pending_feature_generation_ = 0;
    last_exact_feature_generation_ = 0;
    pending_track_id_ = -1;
    pending_confirm_count_ = 0;
    start_time_ = now();
    last_bodylist_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_features_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_lock_command_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_validated_body_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    raw_body_count_ = -1;
    last_raw_body_count_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    yolo_without_body_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_astra_restart_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    astra_restart_count_ = 0;
    astra_restart_limit_reported_ = false;
    last_state_.clear();
    clear_yolo_lock();
    publish_exact_invalid_once(state, true);
  }

  void mode_callback(const std_msgs::msg::Int8::SharedPtr msg)
  {
    const int next_mode = static_cast<int>(msg->data);
    if (next_mode == current_mode_) {
      return;
    }
    current_mode_ = next_mode;
    reset_identity(
      current_mode_ == kFollowMode ? "identity_wait_body" : "identity_inactive");
  }

  void update_astra_watchdog(const rclcpp::Time & t)
  {
    if (current_mode_ != kFollowMode) {
      return;
    }

    if (!yolo_recent(t) || !raw_body_empty_recent(t)) {
      yolo_without_body_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      return;
    }

    if (yolo_without_body_since_.nanoseconds() == 0) {
      yolo_without_body_since_ = t;
      return;
    }
    if ((t - yolo_without_body_since_).seconds() < kAstraNoBodyGraceS) {
      return;
    }
    if (astra_restart_count_ >= kAstraMaxRestartsPerFollowSession) {
      if (!astra_restart_limit_reported_) {
        RCLCPP_ERROR(
          get_logger(),
          "Astra watchdog stopped after %d restart requests in this follow session",
          astra_restart_count_);
        astra_restart_limit_reported_ = true;
      }
      return;
    }
    if (last_astra_restart_time_.nanoseconds() != 0 &&
        (t - last_astra_restart_time_).seconds() < kAstraRestartCooldownS) {
      return;
    }

    astra_restart_pub_->publish(std_msgs::msg::Empty{});
    current_geometry_body_id_ = 0;
    reset_geometry_switch_confirmation();
    ++astra_restart_count_;
    last_astra_restart_time_ = t;
    yolo_without_body_since_ = t;
    RCLCPP_WARN(
      get_logger(),
      "Astra watchdog requested body stream restart: YOLO person present, raw Astra body list empty "
      "for %.1f s (attempt %d/%d)",
      kAstraNoBodyGraceS, astra_restart_count_, kAstraMaxRestartsPerFollowSession);
  }

  void acquire_initial_identity(const rclcpp::Time & t)
  {
    if (!body_recent(t)) {
      publish_state("identity_wait_body");
      return;
    }

    if (yolo_recent(t)) {
      const YoloTarget * candidate = initial_yolo_candidate();
      if (candidate == nullptr) {
        if (pending_track_id_ > 0) {
          reset_rebind_confirmation();
          publish_state("identity_initial_candidate_lost");
        } else {
          publish_state(
            yolo_targets_.size() > 1 ? "identity_initial_ambiguous" :
            "identity_wait_initial_yolo_target");
        }
        return;
      }

      const Match match = match_yolo_to_body(*candidate);
      if (match.valid && match.score <= match_accept_score_ &&
          confirm_initial_match(match)) {
        has_identity_ = true;
        capture_initial_signature(match.yolo);
        publish_exact_observation(match, "identity_tracking");
        return;
      }
      if (!match.valid || match.score > match_accept_score_) {
        log_identity_rejection("initial_pair", candidate, &match);
        reset_rebind_confirmation();
        publish_state("identity_wait_yolo_body_match");
      }
      return;
    }

    if (require_yolo_for_initial_lock_ &&
        (t - start_time_).seconds() < initial_yolo_wait_s_) {
      publish_state("identity_wait_initial_yolo");
      return;
    }

    publish_state("identity_initial_yolo_missing");
  }

  void reacquire_identity(const rclcpp::Time & t)
  {
    if (!body_recent(t)) {
      reset_geometry_switch_confirmation();
      reset_rebind_confirmation();
      publish_exact_invalid_once("identity_wait_body_reacquire");
      return;
    }
    if (!yolo_recent(t)) {
      reset_geometry_switch_confirmation();
      reset_rebind_confirmation();
      publish_exact_invalid_once("identity_wait_yolo_reacquire");
      return;
    }

    const SignatureChoice choice = best_signature_yolo();
    if (choice.target == nullptr) {
      reset_geometry_switch_confirmation();
      log_identity_rejection(
        choice.ambiguous ? "signature_ambiguous" : "signature_mismatch",
        identity_yolo_target(), nullptr);
      reset_rebind_confirmation();
      publish_exact_invalid_once(
        choice.ambiguous ? "identity_yolo_signature_ambiguous" :
        "identity_no_yolo_signature_match");
      return;
    }

    const Match match = match_yolo_with_current_carrier(
      *choice.target, reacquire_accept_score_);
    if (match.valid && match.score <= reacquire_accept_score_ &&
        tracking_appearance_matches(match.yolo)) {
      if (confirm_rebind(match)) {
        publish_exact_observation(match, "identity_tracking");
      } else {
        publish_exact_invalid_once("identity_rebind_confirming");
      }
      return;
    }
    log_identity_rejection("reacquire_pair", choice.target, &match);
    reset_geometry_switch_confirmation();
    reset_rebind_confirmation();
    publish_exact_invalid_once("identity_reacquire_match_rejected");
  }

  void control_loop()
  {
    if (current_mode_ >= 0 && current_mode_ != kFollowMode) {
      return;
    }

    const auto t = now();
    if (desired_yolo_track_id_ > 0 &&
        desired_yolo_track_id_ != commanded_yolo_track_id_)
    {
      lock_yolo_track(desired_yolo_track_id_);
    }
    update_astra_watchdog(t);
    if (body_recent(t) && yolo_recent(t) && !body_yolo_synchronized()) {
      reset_geometry_switch_confirmation();
      reset_rebind_confirmation();
      publish_exact_invalid_once("identity_wait_synchronized_body_yolo");
      return;
    }
    if (!has_identity_) {
      acquire_initial_identity(t);
      return;
    }

    const YoloTarget * current_identity = identity_yolo_target();
    if (current_identity != nullptr && body_recent(t) && yolo_recent(t)) {
      const Match match = match_yolo_with_current_carrier(
        *current_identity, match_accept_score_);
      if (match.valid && match.score <= match_accept_score_ &&
          is_bound_yolo_track(match.yolo) &&
          tracking_appearance_matches(match.yolo)) {
        reset_rebind_confirmation();
        publish_exact_observation(match, "identity_tracking");
        return;
      }

      reacquire_identity(t);
      return;
    }

    if (!body_recent(t) || !yolo_recent(t)) {
      reset_geometry_switch_confirmation();
      publish_exact_invalid_once(
        !body_recent(t) ? "identity_wait_body_tracking" : "identity_wait_yolo_tracking");
      return;
    }

    reacquire_identity(t);
  }

  void publish_state(const std::string & state)
  {
    if (state == last_state_) {
      return;
    }
    std_msgs::msg::String msg;
    msg.data = state;
    state_pub_->publish(msg);
    last_state_ = state;
    RCLCPP_INFO(get_logger(), "%s", state.c_str());
  }

  std::string bodylist_topic_;
  std::string validated_bodyposture_topic_;
  std::string features_topic_;
  std::string lock_command_topic_;
  std::string state_topic_;
  std::string last_state_;

  double image_width_{640.0};
  double horizontal_fov_rad_{1.05};
  double yolo_max_age_s_{0.8};
  double body_max_age_s_{0.5};
  double body_yolo_max_skew_s_{0.15};
  double initial_yolo_wait_s_{1.5};
  double lock_command_period_s_{0.8};
  double match_max_angle_rad_{0.18};
  double match_max_depth_diff_m_{0.85};
  double match_accept_score_{1.45};
  double reacquire_accept_score_{1.65};
  double appearance_accept_score_{1.35};
  double bound_appearance_accept_score_{1.15};
  double bound_body_bbox_margin_ratio_{kTrackingBboxMarginRatio};
  double reacquire_candidate_margin_{0.18};
  double body_pair_candidate_margin_{0.12};
  bool require_yolo_for_initial_lock_{true};
  int initial_confirm_frames_{4};
  int reacquire_confirm_frames_{4};
  int astra_restart_count_{0};
  bool astra_restart_limit_reported_{false};

  std::vector<BodyCandidate> bodies_;
  std::vector<YoloTarget> yolo_targets_;
  YoloTarget signature_;
  bool has_signature_{false};
  bool has_identity_{false};
  bool exact_stream_valid_{false};
  bool locked_yolo_visible_{false};
  bool feature_has_valid_person_bbox_{false};
  int current_mode_{-1};
  int current_geometry_body_id_{0};
  bool has_last_exact_geometry_{false};
  double last_exact_x_m_{0.0};
  double last_exact_z_m_{0.0};
  int pending_geometry_track_id_{-1};
  int pending_geometry_body_id_{0};
  int pending_geometry_count_{0};
  size_t pending_geometry_last_feature_generation_{0};
  int64_t pending_geometry_since_ns_{0};
  int64_t pending_geometry_last_evidence_ns_{0};
  double pending_geometry_anchor_x_m_{0.0};
  double pending_geometry_anchor_z_m_{0.0};
  int raw_body_count_{-1};
  int locked_track_id_{-1};
  int locked_identity_track_id_{-1};
  int desired_yolo_track_id_{-1};
  int commanded_yolo_track_id_{-1};
  int pending_track_id_{-1};
  int pending_confirm_count_{0};
  size_t feature_generation_{0};
  size_t pending_feature_generation_{0};
  size_t last_exact_feature_generation_{0};
  int64_t last_feature_source_stamp_ns_{0};
  int64_t source_quarantine_until_ns_{0};

  rclcpp::Time start_time_;
  rclcpp::Time last_bodylist_time_;
  rclcpp::Time last_features_time_;
  rclcpp::Time last_lock_command_time_;
  rclcpp::Time last_validated_body_time_;
  rclcpp::Time last_raw_body_count_time_;
  rclcpp::Time yolo_without_body_since_;
  rclcpp::Time last_astra_restart_time_;

  rclcpp::Subscription<bodyreader_msg::msg::Bodylist>::SharedPtr bodylist_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr raw_body_count_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr features_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr mode_sub_;
  rclcpp::Publisher<bodyreader_msg::msg::Bodyposture>::SharedPtr validated_bodyposture_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lock_command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr astra_restart_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BodyIdentityBridge>());
  rclcpp::shutdown();
  return 0;
}
