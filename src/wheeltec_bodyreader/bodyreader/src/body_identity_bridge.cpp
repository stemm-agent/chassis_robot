#include <algorithm>
#include <cmath>
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
#include "std_msgs/msg/int16.hpp"
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
constexpr int kAstraMaxRestartsPerFollowSession = 2;
}

class BodyIdentityBridge : public rclcpp::Node
{
public:
  BodyIdentityBridge()
  : Node("body_identity_bridge")
  {
    bodylist_topic_ = declare_parameter<std::string>("bodylist_topic", "/bodylist");
    bodyposture_topic_ = declare_parameter<std::string>("bodyposture_topic", "/body_posture");
    validated_bodyposture_topic_ = declare_parameter<std::string>(
      "validated_bodyposture_topic", "/body_posture_yolo_validated");
    features_topic_ = declare_parameter<std::string>("features_topic", "/torso_trt_demo/features");
    lock_command_topic_ =
      declare_parameter<std::string>("lock_command_topic", "/torso_trt_demo/lock_command");
    recoveryid_topic_ = declare_parameter<std::string>("recoveryid_topic", "/recoveryid");
    state_topic_ = declare_parameter<std::string>("state_topic", "/body_identity_state");

    image_width_ = declare_parameter<double>("image_width", 640.0);
    horizontal_fov_rad_ = declare_parameter<double>("horizontal_fov_rad", 1.05);
    yolo_max_age_s_ = declare_parameter<double>("yolo_max_age_s", 0.8);
    body_max_age_s_ = declare_parameter<double>("body_max_age_s", 0.5);
    initial_yolo_wait_s_ = declare_parameter<double>("initial_yolo_wait_s", 1.5);
    recovery_publish_period_s_ = declare_parameter<double>("recovery_publish_period_s", 0.45);
    lock_command_period_s_ = declare_parameter<double>("lock_command_period_s", 0.8);
    match_max_angle_rad_ = declare_parameter<double>("match_max_angle_rad", 0.18);
    match_max_depth_diff_m_ = declare_parameter<double>("match_max_depth_diff_m", 0.85);
    match_accept_score_ = declare_parameter<double>("match_accept_score", 1.45);
    reacquire_accept_score_ = declare_parameter<double>("reacquire_accept_score", 1.65);
    appearance_accept_score_ = declare_parameter<double>("appearance_accept_score", 1.35);
    bound_appearance_accept_score_ =
      declare_parameter<double>("bound_appearance_accept_score", 1.15);
    identity_validation_grace_s_ =
      std::max(0.0, declare_parameter<double>("identity_validation_grace_s", 0.5));
    reacquire_candidate_margin_ =
      declare_parameter<double>("reacquire_candidate_margin", 0.18);
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
    bodyposture_sub_ = create_subscription<bodyreader_msg::msg::Bodyposture>(
      bodyposture_topic_, 5,
      std::bind(&BodyIdentityBridge::bodyposture_callback, this, std::placeholders::_1));
    features_sub_ = create_subscription<std_msgs::msg::String>(
      features_topic_, 5,
      std::bind(&BodyIdentityBridge::features_callback, this, std::placeholders::_1));
    mode_sub_ = create_subscription<std_msgs::msg::Int8>(
      "/mode", rclcpp::QoS(1).best_effort(),
      std::bind(&BodyIdentityBridge::mode_callback, this, std::placeholders::_1));

    recovery_pub_ = create_publisher<std_msgs::msg::Int16>(recoveryid_topic_, 5);
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
    last_recovery_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_lock_command_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_validated_body_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    yolo_without_body_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_astra_restart_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());

    RCLCPP_INFO(
      get_logger(),
      "BodyIdentityBridge ready: body=%s features=%s validated=%s recovery=%s "
      "require_yolo_initial=%s initial_confirm_frames=%d "
      "reacquire_confirm_frames=%d candidate_margin=%.2f",
      bodylist_topic_.c_str(), features_topic_.c_str(), validated_bodyposture_topic_.c_str(),
      recoveryid_topic_.c_str(),
      require_yolo_for_initial_lock_ ? "true" : "false",
      initial_confirm_frames_, reacquire_confirm_frames_, reacquire_candidate_margin_);
  }

private:
  struct BodyCandidate
  {
    int id{0};
    double angle{0.0};
    double depth_m{0.0};
    bool valid_depth{false};
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

  void bodylist_callback(const bodyreader_msg::msg::Bodylist::SharedPtr msg)
  {
    const int count = std::max(0, static_cast<int>(msg->count));
    bodies_.clear();
    bodies_.reserve(static_cast<size_t>(count));
    for (int i = 0; i < count; ++i) {
      const auto & body = msg->bodies[static_cast<size_t>(i)];
      if (body.centerofmass.z <= 100.0f) {
        continue;
      }
      BodyCandidate candidate;
      candidate.id = body.bodyid;
      candidate.depth_m = static_cast<double>(body.centerofmass.z) * 0.001;
      candidate.valid_depth = candidate.depth_m > 0.1;
      candidate.angle = std::atan2(
        static_cast<double>(body.centerofmass.x),
        static_cast<double>(body.centerofmass.z));
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
    last_bodylist_time_ = now();
  }

  void bodyposture_callback(const bodyreader_msg::msg::Bodyposture::SharedPtr msg)
  {
    current_lock_status_ = msg->lock_status;
    current_body_id_ = msg->bodyid;

    const auto t = now();
    bool yolo_validated = false;
    if (has_identity_ && msg->lock_status == 2 && msg->bodyid == target_body_id_ &&
        msg->centerofmass_z > 100.0f && body_recent(t) && yolo_recent(t)) {
      const Match match = match_body_to_yolo(msg->bodyid);
      yolo_validated =
        match.valid && match.score <= match_accept_score_ && is_bound_yolo_track(match.yolo) &&
        tracking_appearance_matches(match.yolo);
    }

    if (yolo_validated) {
      last_validated_body_time_ = t;
    } else {
      const bool same_locked_body =
        has_identity_ && msg->lock_status == 2 && msg->bodyid == target_body_id_ &&
        msg->centerofmass_z > 100.0f;
      const bool within_validation_grace =
        last_validated_body_time_.nanoseconds() != 0 &&
        (t - last_validated_body_time_).seconds() <= identity_validation_grace_s_;
      if (same_locked_body && within_validation_grace) {
        yolo_validated = true;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Identity validation grace active for body=%d (age=%.3fs limit=%.3fs)",
          msg->bodyid, (t - last_validated_body_time_).seconds(),
          identity_validation_grace_s_);
      }
    }

    auto validated = *msg;
    if (!yolo_validated) {
      validated.lock_status = 0;
    }
    validated_bodyposture_pub_->publish(validated);
  }

  void features_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    ++feature_generation_;
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

  bool appearance_matches(const YoloTarget & target) const
  {
    return !has_signature_ || appearance_score(target) <= appearance_accept_score_;
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
    const double margin_x = bbox_width * kTrackingBboxMarginRatio;
    const double margin_y = bbox_height * kTrackingBboxMarginRatio;
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

  Match best_pair_match(bool require_signature) const
  {
    Match best;
    for (const auto & body : bodies_) {
      for (const auto & yolo : yolo_targets_) {
        if (require_signature && appearance_score(yolo) > appearance_accept_score_) {
          continue;
        }
        const double score = pair_score(body, yolo);
        if (score < best.score) {
          best.valid = true;
          best.body = body;
          best.yolo = yolo;
          best.score = score;
        }
      }
    }
    return best;
  }

  Match match_body_to_yolo(int body_id) const
  {
    Match best;
    for (const auto & body : bodies_) {
      if (body.id != body_id) {
        continue;
      }
      for (const auto & yolo : yolo_targets_) {
        const double score = pair_score(body, yolo);
        if (score < best.score) {
          best.valid = true;
          best.body = body;
          best.yolo = yolo;
          best.score = score;
        }
      }
    }
    return best;
  }

  Match match_yolo_to_body(const YoloTarget & yolo) const
  {
    Match best;
    for (const auto & body : bodies_) {
      const double score = pair_score(body, yolo);
      if (score < best.score) {
        best.valid = true;
        best.body = body;
        best.yolo = yolo;
        best.score = score;
      }
    }
    return best;
  }

  const BodyCandidate * find_body(int id) const
  {
    for (const auto & body : bodies_) {
      if (body.id == id) {
        return &body;
      }
    }
    return nullptr;
  }

  const YoloTarget * locked_yolo() const
  {
    if (locked_track_id_ < 0) {
      return nullptr;
    }
    for (const auto & target : yolo_targets_) {
      if (target.track_id == locked_track_id_) {
        return &target;
      }
    }
    return nullptr;
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
    pending_body_id_ = 0;
    pending_confirm_count_ = 0;
    pending_feature_generation_ = 0;
  }

  void publish_recovery_id(int body_id, const std::string & state)
  {
    const auto t = now();
    if ((t - last_recovery_time_).seconds() < recovery_publish_period_s_ &&
        body_id == last_recovery_body_id_) {
      return;
    }
    std_msgs::msg::Int16 msg;
    msg.data = static_cast<int16_t>(body_id);
    recovery_pub_->publish(msg);
    last_recovery_time_ = t;
    last_recovery_body_id_ = body_id;
    publish_state(state);
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
    locked_identity_track_id_ = track_id;
    lock_yolo_track(track_id);
  }

  bool confirm_rebind(const Match & match)
  {
    if (match.yolo.track_id != pending_track_id_ || match.body.id != pending_body_id_) {
      pending_track_id_ = match.yolo.track_id;
      pending_body_id_ = match.body.id;
      pending_confirm_count_ = 0;
      pending_feature_generation_ = 0;
      RCLCPP_INFO(
        get_logger(),
        "identity_rebind_candidate: old_track=%d candidate_track=%d body=%d "
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
      "identity_rebind_confirmed: old_track=%d new_track=%d body=%d frames=%d",
      locked_identity_track_id_, match.yolo.track_id, match.body.id, pending_confirm_count_);
    target_body_id_ = match.body.id;
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
    return yolo_targets_.size() == 1 ? &yolo_targets_.front() : nullptr;
  }

  bool confirm_initial_match(const Match & match)
  {
    if (match.yolo.track_id != pending_track_id_ || match.body.id != pending_body_id_) {
      pending_track_id_ = match.yolo.track_id;
      pending_body_id_ = match.body.id;
      pending_confirm_count_ = 0;
      pending_feature_generation_ = 0;
      RCLCPP_INFO(
        get_logger(),
        "identity_initial_candidate: track=%d body=%d pair_score=%.3f required_frames=%d",
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
      "identity_initial_confirmed: track=%d body=%d frames=%d",
      match.yolo.track_id, match.body.id, pending_confirm_count_);
    return true;
  }

  void capture_initial_signature(const YoloTarget & target)
  {
    signature_ = YoloTarget{};
    signature_.upper_hue = target.upper_hue;
    signature_.lower_hue = target.lower_hue;
    signature_.height_ratio = target.height_ratio;
    has_signature_ = true;
    bind_yolo_track(target.track_id);
    reset_rebind_confirmation();

    RCLCPP_INFO(
      get_logger(),
      "identity_signature_locked: track=%d upper_hue=%.1f lower_hue=%.1f height_ratio=%.3f "
      "weights=upper:%.2f,lower:%.2f,height:%.2f",
      target.track_id, signature_.upper_hue, signature_.lower_hue,
      signature_.height_ratio, kUpperHueWeight, kLowerHueWeight, kHeightRatioWeight);
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
    target_body_id_ = 0;
    current_body_id_ = 0;
    current_lock_status_ = 0;
    locked_track_id_ = -1;
    locked_identity_track_id_ = -1;
    commanded_yolo_track_id_ = -1;
    last_recovery_body_id_ = 0;
    feature_generation_ = 0;
    pending_feature_generation_ = 0;
    pending_track_id_ = -1;
    pending_body_id_ = 0;
    pending_confirm_count_ = 0;
    start_time_ = now();
    last_bodylist_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_features_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_recovery_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_lock_command_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_validated_body_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    yolo_without_body_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_astra_restart_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    astra_restart_count_ = 0;
    astra_restart_limit_reported_ = false;
    last_state_.clear();
    clear_yolo_lock();
    publish_state(state);
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

    if (body_recent(t) || !yolo_recent(t)) {
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
    ++astra_restart_count_;
    last_astra_restart_time_ = t;
    yolo_without_body_since_ = t;
    RCLCPP_WARN(
      get_logger(),
      "Astra watchdog requested body stream restart: YOLO person present, no valid body "
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
        target_body_id_ = match.body.id;
        has_identity_ = true;
        capture_initial_signature(match.yolo);
        publish_recovery_id(target_body_id_, "identity_initial_yolo_lock");
        return;
      }
      if (!match.valid || match.score > match_accept_score_) {
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

    if (require_yolo_for_initial_lock_) {
      publish_state("identity_initial_yolo_missing");
      return;
    }

    const auto best_body = std::min_element(
      bodies_.begin(), bodies_.end(),
      [](const BodyCandidate & a, const BodyCandidate & b) {
        return std::fabs(a.angle) < std::fabs(b.angle);
      });
    if (best_body != bodies_.end()) {
      target_body_id_ = best_body->id;
      has_identity_ = true;
      publish_recovery_id(target_body_id_, "identity_initial_skeleton_lock");
    }
  }

  void reacquire_identity(const rclcpp::Time & t)
  {
    if (!body_recent(t)) {
      reset_rebind_confirmation();
      publish_state("identity_wait_body_reacquire");
      return;
    }
    if (!yolo_recent(t)) {
      reset_rebind_confirmation();
      publish_state("identity_wait_yolo_reacquire");
      return;
    }

    const SignatureChoice choice = best_signature_yolo();
    if (choice.target == nullptr) {
      reset_rebind_confirmation();
      publish_state(
        choice.ambiguous ? "identity_yolo_signature_ambiguous" :
        "identity_no_yolo_signature_match");
      return;
    }

    const Match match = match_yolo_to_body(*choice.target);
    if (match.valid && match.score <= reacquire_accept_score_ &&
        tracking_appearance_matches(match.yolo)) {
      if (confirm_rebind(match)) {
        publish_recovery_id(target_body_id_, "identity_reacquired");
      }
      return;
    }
    reset_rebind_confirmation();
    publish_state("identity_reacquire_match_rejected");
  }

  void control_loop()
  {
    if (current_mode_ >= 0 && current_mode_ != kFollowMode) {
      return;
    }

    const auto t = now();
    update_astra_watchdog(t);
    if (!has_identity_) {
      acquire_initial_identity(t);
      return;
    }

    const BodyCandidate * current_target = find_body(target_body_id_);
    if (current_target != nullptr && yolo_recent(t)) {
      const Match match = match_body_to_yolo(target_body_id_);
      if (match.valid && match.score <= match_accept_score_ &&
          is_bound_yolo_track(match.yolo) &&
          tracking_appearance_matches(match.yolo)) {
        reset_rebind_confirmation();
        if (current_lock_status_ != 2 || current_body_id_ != target_body_id_) {
          publish_recovery_id(target_body_id_, "identity_restore_body_lock");
        }
        publish_state("identity_tracking");
        return;
      }

      reacquire_identity(t);
      return;
    }

    if (current_target != nullptr) {
      publish_state("identity_wait_yolo_tracking");
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
  std::string bodyposture_topic_;
  std::string validated_bodyposture_topic_;
  std::string features_topic_;
  std::string lock_command_topic_;
  std::string recoveryid_topic_;
  std::string state_topic_;
  std::string last_state_;

  double image_width_{640.0};
  double horizontal_fov_rad_{1.05};
  double yolo_max_age_s_{0.8};
  double body_max_age_s_{0.5};
  double initial_yolo_wait_s_{1.5};
  double recovery_publish_period_s_{0.45};
  double lock_command_period_s_{0.8};
  double match_max_angle_rad_{0.18};
  double match_max_depth_diff_m_{0.85};
  double match_accept_score_{1.45};
  double reacquire_accept_score_{1.65};
  double appearance_accept_score_{1.35};
  double bound_appearance_accept_score_{1.15};
  double identity_validation_grace_s_{0.5};
  double reacquire_candidate_margin_{0.18};
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
  bool locked_yolo_visible_{false};
  int current_mode_{-1};
  int target_body_id_{0};
  int current_body_id_{0};
  int current_lock_status_{0};
  int locked_track_id_{-1};
  int locked_identity_track_id_{-1};
  int commanded_yolo_track_id_{-1};
  int last_recovery_body_id_{0};
  int pending_track_id_{-1};
  int pending_body_id_{0};
  int pending_confirm_count_{0};
  size_t feature_generation_{0};
  size_t pending_feature_generation_{0};

  rclcpp::Time start_time_;
  rclcpp::Time last_bodylist_time_;
  rclcpp::Time last_features_time_;
  rclcpp::Time last_recovery_time_;
  rclcpp::Time last_lock_command_time_;
  rclcpp::Time last_validated_body_time_;
  rclcpp::Time yolo_without_body_since_;
  rclcpp::Time last_astra_restart_time_;

  rclcpp::Subscription<bodyreader_msg::msg::Bodylist>::SharedPtr bodylist_sub_;
  rclcpp::Subscription<bodyreader_msg::msg::Bodyposture>::SharedPtr bodyposture_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr features_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr mode_sub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr recovery_pub_;
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
