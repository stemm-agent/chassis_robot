#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

#include <NvInfer.h>
#include <NvInferPlugin.h>
#include <cuda_fp16.h>
#include <cuda_runtime_api.h>

#include <builtin_interfaces/msg/time.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

#include "bodyreader_msg/msg/bodylist.hpp"
#include "bodyreader_msg/msg/bodyposture.hpp"

namespace
{

volatile sig_atomic_t g_shutdown_requested = 0;

void handle_signal(int)
{
  g_shutdown_requested = 1;
}

class TrtLogger : public nvinfer1::ILogger
{
public:
  void log(Severity severity, const char * msg) noexcept override
  {
    if (severity <= Severity::kWARNING) {
      std::cerr << "[TensorRT] " << msg << std::endl;
    }
  }
};

template<typename T>
struct TrtDeleter
{
  void operator()(T * obj) const
  {
    delete obj;
  }
};

template<typename T>
using TrtUniquePtr = std::unique_ptr<T, TrtDeleter<T>>;

struct LetterboxInfo
{
  float scale{1.0f};
  float pad_x{0.0f};
  float pad_y{0.0f};
  int original_width{0};
  int original_height{0};
};

struct Detection
{
  float x1{0.0f};
  float y1{0.0f};
  float x2{0.0f};
  float y2{0.0f};
  float score{0.0f};
  int class_id{0};
};

constexpr int kHueBins = 36;
constexpr double kPi = 3.14159265358979323846;

struct PersonFeature
{
  Detection person;
  Detection torso;
  Detection lower_body;
  Detection upper_color_region;
  Detection lower_color_region;
  int track_id{-1};
  float center_x{0.0f};
  float center_y{0.0f};
  float person_height_px{0.0f};
  float person_height_px_smooth{0.0f};
  float person_height_ratio{0.0f};
  float person_height_ratio_smooth{0.0f};
  float person_area_ratio{0.0f};
  float torso_area_ratio{0.0f};
  float aspect_ratio{0.0f};
  float depth_m{0.0f};
  float height_m{0.0f};
  std::string height_source{"image_only"};
  cv::Scalar mean_bgr{0.0, 0.0, 0.0, 0.0};
  cv::Scalar stddev_bgr{0.0, 0.0, 0.0, 0.0};
  cv::Scalar mean_hsv{0.0, 0.0, 0.0, 0.0};
  std::array<float, kHueBins> hue_hist{};
  std::string dominant_color{"unknown"};
  float dominant_hue{0.0f};
  float color_confidence{0.0f};
  cv::Scalar lower_mean_bgr{0.0, 0.0, 0.0, 0.0};
  cv::Scalar lower_stddev_bgr{0.0, 0.0, 0.0, 0.0};
  cv::Scalar lower_mean_hsv{0.0, 0.0, 0.0, 0.0};
  std::array<float, kHueBins> lower_hue_hist{};
  std::string lower_dominant_color{"unknown"};
  float lower_dominant_hue{0.0f};
  float lower_color_confidence{0.0f};
};

struct TrackState
{
  int id{0};
  Detection person;
  float center_x{0.0f};
  float center_y{0.0f};
  float person_height_ratio{0.0f};
  float smoothed_height_px{0.0f};
  float smoothed_height_ratio{0.0f};
  cv::Scalar mean_hsv{0.0, 0.0, 0.0, 0.0};
  std::array<float, kHueBins> hue_hist{};
  cv::Scalar lower_mean_hsv{0.0, 0.0, 0.0, 0.0};
  std::array<float, kHueBins> lower_hue_hist{};
  int missed{0};
};

struct TrackCandidate
{
  double score{0.0};
  size_t feature_index{0};
  size_t track_index{0};
};

struct TensorBuffer
{
  std::string name;
  nvinfer1::TensorIOMode mode{nvinfer1::TensorIOMode::kNONE};
  nvinfer1::DataType type{nvinfer1::DataType::kFLOAT};
  nvinfer1::Dims dims{};
  size_t elem_count{0};
  size_t byte_count{0};
  void * device{nullptr};
  std::vector<uint8_t> host;
};

bool has_dynamic_dim(const nvinfer1::Dims & dims)
{
  for (int i = 0; i < dims.nbDims; ++i) {
    if (dims.d[i] < 0) {
      return true;
    }
  }
  return false;
}

size_t data_type_size(nvinfer1::DataType type)
{
  switch (type) {
    case nvinfer1::DataType::kFLOAT:
      return 4;
    case nvinfer1::DataType::kHALF:
      return 2;
    case nvinfer1::DataType::kINT8:
      return 1;
    case nvinfer1::DataType::kINT32:
      return 4;
    case nvinfer1::DataType::kBOOL:
      return 1;
    default:
      return 0;
  }
}

size_t dims_volume(const nvinfer1::Dims & dims)
{
  if (dims.nbDims <= 0) {
    return 0;
  }
  size_t volume = 1;
  for (int i = 0; i < dims.nbDims; ++i) {
    if (dims.d[i] <= 0) {
      return 0;
    }
    volume *= static_cast<size_t>(dims.d[i]);
  }
  return volume;
}

std::vector<int64_t> dims_to_vector(const nvinfer1::Dims & dims)
{
  std::vector<int64_t> shape;
  for (int i = 0; i < dims.nbDims; ++i) {
    shape.push_back(dims.d[i]);
  }
  return shape;
}

std::string shape_to_string(const nvinfer1::Dims & dims)
{
  std::string out = "[";
  for (int i = 0; i < dims.nbDims; ++i) {
    if (i > 0) {
      out += ", ";
    }
    out += std::to_string(dims.d[i]);
  }
  out += "]";
  return out;
}

float clamp_float(float value, float low, float high)
{
  return std::max(low, std::min(value, high));
}

float intersection_over_union(const Detection & a, const Detection & b)
{
  const float x1 = std::max(a.x1, b.x1);
  const float y1 = std::max(a.y1, b.y1);
  const float x2 = std::min(a.x2, b.x2);
  const float y2 = std::min(a.y2, b.y2);
  const float inter_w = std::max(0.0f, x2 - x1);
  const float inter_h = std::max(0.0f, y2 - y1);
  const float inter = inter_w * inter_h;
  const float area_a = std::max(0.0f, a.x2 - a.x1) * std::max(0.0f, a.y2 - a.y1);
  const float area_b = std::max(0.0f, b.x2 - b.x1) * std::max(0.0f, b.y2 - b.y1);
  const float denom = area_a + area_b - inter;
  if (denom <= 1e-6f) {
    return 0.0f;
  }
  return inter / denom;
}

float detection_width(const Detection & detection)
{
  return std::max(0.0f, detection.x2 - detection.x1);
}

float detection_height(const Detection & detection)
{
  return std::max(0.0f, detection.y2 - detection.y1);
}

float detection_area(const Detection & detection)
{
  return detection_width(detection) * detection_height(detection);
}

cv::Rect detection_to_rect(const Detection & detection, int image_width, int image_height)
{
  if (image_width <= 1 || image_height <= 1) {
    return cv::Rect();
  }

  const int x1 = std::max(0, std::min(image_width - 1, static_cast<int>(std::round(detection.x1))));
  const int y1 = std::max(0, std::min(image_height - 1, static_cast<int>(std::round(detection.y1))));
  const int x2 = std::max(x1 + 1, std::min(image_width, static_cast<int>(std::round(detection.x2))));
  const int y2 = std::max(y1 + 1, std::min(image_height, static_cast<int>(std::round(detection.y2))));
  return cv::Rect(x1, y1, x2 - x1, y2 - y1);
}

std::string trim_copy(const std::string & value)
{
  const size_t start = value.find_first_not_of(" \t\r\n");
  if (start == std::string::npos) {
    return "";
  }
  const size_t end = value.find_last_not_of(" \t\r\n");
  return value.substr(start, end - start + 1);
}

std::string lower_copy(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

}  // namespace

class TorsoTrtDemo : public rclcpp::Node
{
public:
  TorsoTrtDemo()
  : Node("torso_trt_demo")
  {
    engine_path_ = declare_parameter<std::string>(
      "engine_path",
      "/home/wheeltec/wheeltec_ros2/src/ultralytics_ros2/model/yolo11n_fp16.engine");
    image_topic_ = declare_parameter<std::string>("image_topic", "/camera/color/image_raw");
    annotated_topic_ = declare_parameter<std::string>(
      "annotated_topic", "/torso_trt_demo/image");
    detections_topic_ = declare_parameter<std::string>(
      "detections_topic", "/torso_trt_demo/detections");
    state_topic_ = declare_parameter<std::string>(
      "state_topic", "/torso_trt_demo/state");
    features_topic_ = declare_parameter<std::string>(
      "features_topic", "/torso_trt_demo/features");
    locked_topic_ = declare_parameter<std::string>(
      "locked_topic", "/torso_trt_demo/locked_target");
    lock_command_topic_ = declare_parameter<std::string>(
      "lock_command_topic", "/torso_trt_demo/lock_command");
    depth_topic_ = declare_parameter<std::string>(
      "depth_topic", "/camera/depth/image_raw");
    camera_info_topic_ = declare_parameter<std::string>(
      "camera_info_topic", "/camera/color/camera_info");
    bodylist_topic_ = declare_parameter<std::string>("bodylist_topic", "/bodylist");
    bodyposture_topic_ = declare_parameter<std::string>("bodyposture_topic", "/body_posture");

    input_width_ = declare_parameter<int>("input_width", 640);
    input_height_ = declare_parameter<int>("input_height", 640);
    conf_threshold_ = declare_parameter<double>("conf_threshold", 0.45);
    nms_iou_threshold_ = declare_parameter<double>("nms_iou_threshold", 0.45);
    person_class_id_ = declare_parameter<int>("person_class_id", 0);
    max_detections_ = declare_parameter<int>("max_detections", 20);
    min_person_area_px_ = declare_parameter<double>("min_person_area_px", 600.0);
    torso_side_crop_ratio_ = declare_parameter<double>("torso_side_crop_ratio", 0.18);
    torso_top_ratio_ = declare_parameter<double>("torso_top_ratio", 0.22);
    torso_bottom_ratio_ = declare_parameter<double>("torso_bottom_ratio", 0.68);
    upper_color_side_crop_ratio_ = declare_parameter<double>("upper_color_side_crop_ratio", 0.28);
    upper_color_band_top_ratio_ = declare_parameter<double>("upper_color_band_top_ratio", 0.00);
    upper_color_band_bottom_ratio_ = declare_parameter<double>("upper_color_band_bottom_ratio", 0.50);
    upper_color_inner_top_ratio_ = declare_parameter<double>("upper_color_inner_top_ratio", 0.34);
    upper_color_inner_bottom_ratio_ = declare_parameter<double>("upper_color_inner_bottom_ratio", 0.84);
    lower_color_side_crop_ratio_ = declare_parameter<double>("lower_color_side_crop_ratio", 0.26);
    lower_color_band_top_ratio_ = declare_parameter<double>("lower_color_band_top_ratio", 0.50);
    lower_color_band_bottom_ratio_ = declare_parameter<double>("lower_color_band_bottom_ratio", 1.00);
    lower_color_inner_top_ratio_ = declare_parameter<double>("lower_color_inner_top_ratio", 0.08);
    lower_color_inner_bottom_ratio_ = declare_parameter<double>("lower_color_inner_bottom_ratio", 0.44);
    publish_annotated_ = declare_parameter<bool>("publish_annotated", true);
    log_image_features_ = declare_parameter<bool>("log_image_features", true);
    feature_log_period_sec_ = declare_parameter<double>("feature_log_period_sec", 1.0);
    max_logged_targets_ = declare_parameter<int>("max_logged_targets", 6);
    color_saturation_min_ = declare_parameter<double>("color_saturation_min", 22.0);
    color_value_min_ = declare_parameter<double>("color_value_min", 35.0);
    height_smoothing_alpha_ = declare_parameter<double>("height_smoothing_alpha", 0.35);
    enable_depth_height_ = declare_parameter<bool>("enable_depth_height", true);
    camera_fy_px_ = declare_parameter<double>("camera_fy_px", 0.0);
    depth_max_age_sec_ = declare_parameter<double>("depth_max_age_sec", 0.5);
    depth_unit_scale_ = declare_parameter<double>("depth_unit_scale", 0.001);
    min_valid_depth_m_ = declare_parameter<double>("min_valid_depth_m", 0.20);
    max_valid_depth_m_ = declare_parameter<double>("max_valid_depth_m", 8.0);
    height_reference_m_ = declare_parameter<double>("height_reference_m", 0.0);
    height_reference_px_ = declare_parameter<double>("height_reference_px", 0.0);
    auto_lock_center_ = declare_parameter<bool>("auto_lock_center", false);
    track_max_missed_ = declare_parameter<int>("track_max_missed", 12);
    track_match_threshold_ = declare_parameter<double>("track_match_threshold", 0.82);
    track_iou_weight_ = declare_parameter<double>("track_iou_weight", 0.35);
    track_appearance_weight_ = declare_parameter<double>("track_appearance_weight", 0.35);
    track_size_weight_ = declare_parameter<double>("track_size_weight", 0.15);
    track_center_weight_ = declare_parameter<double>("track_center_weight", 0.15);
    overlay_skeleton_ = declare_parameter<bool>("overlay_skeleton", true);
    skeleton_max_age_sec_ = declare_parameter<double>("skeleton_max_age_sec", 0.5);
    skeleton_base_width_ = declare_parameter<double>("skeleton_base_width", 640.0);
    skeleton_base_height_ = declare_parameter<double>("skeleton_base_height", 480.0);
    respect_mode_topic_ = declare_parameter<bool>("respect_mode_topic", false);
    mode_required_ = declare_parameter<int>("mode_required", 2);
    current_mode_ = declare_parameter<int>("initial_mode", 1);
    max_inference_fps_ = std::max(
      0.0, declare_parameter<double>("max_inference_fps", 0.0));
    max_annotated_fps_ = std::max(
      0.0, declare_parameter<double>("max_annotated_fps", 0.0));
    annotated_only_if_subscribed_ =
      declare_parameter<bool>("annotated_only_if_subscribed", false);
    opencv_num_threads_ = std::max(
      0,
      static_cast<int>(declare_parameter<int>("opencv_num_threads", 0)));
    if (opencv_num_threads_ > 0) {
      cv::setNumThreads(opencv_num_threads_);
    }

    annotated_pub_ =
      create_publisher<sensor_msgs::msg::Image>(annotated_topic_, 10);
    detections_pub_ =
      create_publisher<vision_msgs::msg::Detection2DArray>(detections_topic_, 10);
    state_pub_ = create_publisher<std_msgs::msg::String>(state_topic_, 10);
    features_pub_ = create_publisher<std_msgs::msg::String>(features_topic_, 10);
    locked_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>(locked_topic_, 10);
    lock_command_sub_ = create_subscription<std_msgs::msg::String>(
      lock_command_topic_, 10,
      std::bind(&TorsoTrtDemo::lock_command_callback, this, std::placeholders::_1));
    mode_sub_ = create_subscription<std_msgs::msg::Int8>(
      "/mode", rclcpp::QoS(1).best_effort(),
      std::bind(&TorsoTrtDemo::mode_callback, this, std::placeholders::_1));
    if (enable_depth_height_) {
      depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        depth_topic_, rclcpp::SensorDataQoS(),
        std::bind(&TorsoTrtDemo::depth_callback, this, std::placeholders::_1));
      camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_, 10,
        std::bind(&TorsoTrtDemo::camera_info_callback, this, std::placeholders::_1));
    }
    if (overlay_skeleton_) {
      bodylist_sub_ = create_subscription<bodyreader_msg::msg::Bodylist>(
        bodylist_topic_, 10,
        std::bind(&TorsoTrtDemo::bodylist_callback, this, std::placeholders::_1));
      bodyposture_sub_ = create_subscription<bodyreader_msg::msg::Bodyposture>(
        bodyposture_topic_, 10,
        std::bind(&TorsoTrtDemo::bodyposture_callback, this, std::placeholders::_1));
    }

    ready_ = load_engine();
    if (!ready_) {
      publish_state("engine_not_ready");
      RCLCPP_ERROR(
        get_logger(),
        "TensorRT torso demo is not ready. Expected FP16 engine: %s",
        engine_path_.c_str());
      RCLCPP_ERROR(
        get_logger(),
        "This Jetson has TensorRT runtime, but no torch/ultralytics/trtexec in the default Python environment.");
    }

    if (g_shutdown_requested) {
      return;
    }

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&TorsoTrtDemo::image_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "TorsoTrtDemo subscribed to %s, annotated=%s, detections=%s, features=%s, locked=%s, "
      "mode_gate=%s mode=%d inference_fps=%.1f annotated_fps=%.1f cv_threads=%d",
      image_topic_.c_str(), annotated_topic_.c_str(), detections_topic_.c_str(),
      features_topic_.c_str(), locked_topic_.c_str(),
      respect_mode_topic_ ? "on" : "off", mode_required_, max_inference_fps_,
      max_annotated_fps_, opencv_num_threads_);
    if (enable_depth_height_) {
      RCLCPP_INFO(
        get_logger(),
        "Height estimate depth_topic=%s camera_info=%s camera_fy_px=%.2f",
        depth_topic_.c_str(), camera_info_topic_.c_str(), camera_fy_px_);
    }
    if (overlay_skeleton_) {
      RCLCPP_INFO(
        get_logger(),
        "Skeleton overlay enabled: bodylist=%s bodyposture=%s output=%s",
        bodylist_topic_.c_str(), bodyposture_topic_.c_str(), annotated_topic_.c_str());
    }
  }

  ~TorsoTrtDemo() override
  {
    for (auto & buffer : buffers_) {
      if (buffer.device != nullptr) {
        cudaFree(buffer.device);
        buffer.device = nullptr;
      }
    }
    if (stream_ != nullptr) {
      cudaStreamDestroy(stream_);
      stream_ = nullptr;
    }
  }

private:
  bool cuda_ok(cudaError_t status, const char * what)
  {
    if (status == cudaSuccess) {
      return true;
    }
    RCLCPP_ERROR(get_logger(), "%s failed: %s", what, cudaGetErrorString(status));
    return false;
  }

  bool load_engine()
  {
    std::ifstream file(engine_path_, std::ios::binary);
    if (!file.good()) {
      RCLCPP_ERROR(get_logger(), "TensorRT engine file does not exist: %s", engine_path_.c_str());
      return false;
    }

    file.seekg(0, std::ifstream::end);
    const std::streamoff size = file.tellg();
    if (size <= 0) {
      RCLCPP_ERROR(get_logger(), "TensorRT engine file is empty: %s", engine_path_.c_str());
      return false;
    }
    file.seekg(0, std::ifstream::beg);

    std::vector<char> engine_data(static_cast<size_t>(size));
    file.read(engine_data.data(), size);
    if (!file.good()) {
      RCLCPP_ERROR(get_logger(), "Failed to read TensorRT engine: %s", engine_path_.c_str());
      return false;
    }

    initLibNvInferPlugins(&trt_logger_, "");
    runtime_.reset(nvinfer1::createInferRuntime(trt_logger_));
    if (!runtime_) {
      RCLCPP_ERROR(get_logger(), "Failed to create TensorRT runtime");
      return false;
    }

    engine_.reset(runtime_->deserializeCudaEngine(engine_data.data(), engine_data.size()));
    if (!engine_) {
      RCLCPP_ERROR(get_logger(), "Failed to deserialize TensorRT engine");
      return false;
    }

    context_.reset(engine_->createExecutionContext());
    if (!context_) {
      RCLCPP_ERROR(get_logger(), "Failed to create TensorRT execution context");
      return false;
    }

    if (!cuda_ok(cudaStreamCreate(&stream_), "cudaStreamCreate")) {
      return false;
    }

    if (!configure_input_shape()) {
      return false;
    }

    if (!allocate_buffers()) {
      return false;
    }

    RCLCPP_INFO(
      get_logger(),
      "Loaded TensorRT engine %s with input %s shape=%s",
      engine_path_.c_str(), input_name_.c_str(), shape_to_string(input_dims_).c_str());
    return true;
  }

  bool configure_input_shape()
  {
    const int tensor_count = engine_->getNbIOTensors();
    for (int i = 0; i < tensor_count; ++i) {
      const char * name = engine_->getIOTensorName(i);
      if (name == nullptr) {
        continue;
      }
      const auto mode = engine_->getTensorIOMode(name);
      if (mode == nvinfer1::TensorIOMode::kINPUT) {
        input_name_ = name;
        break;
      }
    }

    if (input_name_.empty()) {
      RCLCPP_ERROR(get_logger(), "No input tensor found in TensorRT engine");
      return false;
    }

    nvinfer1::Dims dims = engine_->getTensorShape(input_name_.c_str());
    if (has_dynamic_dim(dims)) {
      nvinfer1::Dims4 requested(1, 3, input_height_, input_width_);
      if (!context_->setInputShape(input_name_.c_str(), requested)) {
        RCLCPP_ERROR(
          get_logger(),
          "Failed to set dynamic input shape to [1, 3, %d, %d]",
          input_height_, input_width_);
        return false;
      }
    }

    input_dims_ = context_->getTensorShape(input_name_.c_str());
    if (input_dims_.nbDims != 4) {
      RCLCPP_ERROR(
        get_logger(),
        "Expected NCHW input tensor with 4 dims, got %s",
        shape_to_string(input_dims_).c_str());
      return false;
    }

    input_height_ = static_cast<int>(input_dims_.d[2]);
    input_width_ = static_cast<int>(input_dims_.d[3]);
    if (input_dims_.d[0] != 1 || input_dims_.d[1] != 3 || input_height_ <= 0 || input_width_ <= 0) {
      RCLCPP_ERROR(
        get_logger(),
        "Expected input shape [1, 3, H, W], got %s",
        shape_to_string(input_dims_).c_str());
      return false;
    }

    return true;
  }

  bool allocate_buffers()
  {
    const int tensor_count = engine_->getNbIOTensors();
    buffers_.clear();
    input_buffer_index_ = -1;

    for (int i = 0; i < tensor_count; ++i) {
      const char * tensor_name = engine_->getIOTensorName(i);
      if (tensor_name == nullptr) {
        continue;
      }

      TensorBuffer buffer;
      buffer.name = tensor_name;
      buffer.mode = engine_->getTensorIOMode(tensor_name);
      buffer.type = engine_->getTensorDataType(tensor_name);
      buffer.dims = context_->getTensorShape(tensor_name);
      buffer.elem_count = dims_volume(buffer.dims);
      buffer.byte_count = buffer.elem_count * data_type_size(buffer.type);

      if (buffer.elem_count == 0 || buffer.byte_count == 0) {
        RCLCPP_ERROR(
          get_logger(),
          "Invalid tensor %s shape=%s type=%d",
          buffer.name.c_str(), shape_to_string(buffer.dims).c_str(),
          static_cast<int>(buffer.type));
        return false;
      }

      if (!cuda_ok(cudaMalloc(&buffer.device, buffer.byte_count), "cudaMalloc")) {
        return false;
      }
      buffer.host.resize(buffer.byte_count);

      if (!context_->setTensorAddress(buffer.name.c_str(), buffer.device)) {
        RCLCPP_ERROR(get_logger(), "Failed to set TensorRT address for %s", buffer.name.c_str());
        return false;
      }

      if (buffer.mode == nvinfer1::TensorIOMode::kINPUT) {
        input_buffer_index_ = static_cast<int>(buffers_.size());
      }

      RCLCPP_INFO(
        get_logger(),
        "Tensor %s mode=%s shape=%s bytes=%zu",
        buffer.name.c_str(),
        buffer.mode == nvinfer1::TensorIOMode::kINPUT ? "input" : "output",
        shape_to_string(buffer.dims).c_str(), buffer.byte_count);

      buffers_.push_back(buffer);
    }

    if (input_buffer_index_ < 0) {
      RCLCPP_ERROR(get_logger(), "No input buffer allocated");
      return false;
    }
    return true;
  }

  LetterboxInfo preprocess(const cv::Mat & image, TensorBuffer & input)
  {
    LetterboxInfo info;
    info.original_width = image.cols;
    info.original_height = image.rows;
    info.scale = std::min(
      static_cast<float>(input_width_) / static_cast<float>(image.cols),
      static_cast<float>(input_height_) / static_cast<float>(image.rows));

    const int resized_w = static_cast<int>(std::round(image.cols * info.scale));
    const int resized_h = static_cast<int>(std::round(image.rows * info.scale));
    info.pad_x = static_cast<float>(input_width_ - resized_w) * 0.5f;
    info.pad_y = static_cast<float>(input_height_ - resized_h) * 0.5f;

    cv::Mat resized;
    cv::resize(image, resized, cv::Size(resized_w, resized_h));

    cv::Mat canvas(input_height_, input_width_, CV_8UC3, cv::Scalar(114, 114, 114));
    resized.copyTo(canvas(cv::Rect(
      static_cast<int>(std::round(info.pad_x)),
      static_cast<int>(std::round(info.pad_y)),
      resized_w, resized_h)));

    cv::Mat rgb;
    cv::cvtColor(canvas, rgb, cv::COLOR_BGR2RGB);

    const size_t channel_size = static_cast<size_t>(input_width_) * static_cast<size_t>(input_height_);
    if (input.type == nvinfer1::DataType::kHALF) {
      __half * dst = reinterpret_cast<__half *>(input.host.data());
      for (int c = 0; c < 3; ++c) {
        for (int y = 0; y < input_height_; ++y) {
          for (int x = 0; x < input_width_; ++x) {
            const float value = static_cast<float>(rgb.at<cv::Vec3b>(y, x)[c]) / 255.0f;
            dst[c * channel_size + static_cast<size_t>(y) * input_width_ + x] = __float2half(value);
          }
        }
      }
    } else {
      float * dst = reinterpret_cast<float *>(input.host.data());
      for (int c = 0; c < 3; ++c) {
        for (int y = 0; y < input_height_; ++y) {
          for (int x = 0; x < input_width_; ++x) {
            dst[c * channel_size + static_cast<size_t>(y) * input_width_ + x] =
              static_cast<float>(rgb.at<cv::Vec3b>(y, x)[c]) / 255.0f;
          }
        }
      }
    }

    return info;
  }

  bool infer()
  {
    TensorBuffer & input = buffers_[static_cast<size_t>(input_buffer_index_)];
    if (!cuda_ok(
        cudaMemcpyAsync(input.device, input.host.data(), input.byte_count, cudaMemcpyHostToDevice, stream_),
        "cudaMemcpyAsync input")) {
      return false;
    }

    if (!context_->enqueueV3(stream_)) {
      RCLCPP_ERROR(get_logger(), "TensorRT enqueueV3 failed");
      return false;
    }

    for (auto & buffer : buffers_) {
      if (buffer.mode != nvinfer1::TensorIOMode::kOUTPUT) {
        continue;
      }
      if (!cuda_ok(
          cudaMemcpyAsync(buffer.host.data(), buffer.device, buffer.byte_count, cudaMemcpyDeviceToHost, stream_),
          "cudaMemcpyAsync output")) {
        return false;
      }
    }

    return cuda_ok(cudaStreamSynchronize(stream_), "cudaStreamSynchronize");
  }

  std::vector<float> output_as_float(const TensorBuffer & buffer)
  {
    std::vector<float> values(buffer.elem_count, 0.0f);
    if (buffer.type == nvinfer1::DataType::kFLOAT) {
      const float * src = reinterpret_cast<const float *>(buffer.host.data());
      std::copy(src, src + buffer.elem_count, values.begin());
    } else if (buffer.type == nvinfer1::DataType::kHALF) {
      const __half * src = reinterpret_cast<const __half *>(buffer.host.data());
      for (size_t i = 0; i < buffer.elem_count; ++i) {
        values[i] = __half2float(src[i]);
      }
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Unsupported output tensor type %d for %s",
        static_cast<int>(buffer.type), buffer.name.c_str());
    }
    return values;
  }

  const TensorBuffer * primary_output() const
  {
    const TensorBuffer * best = nullptr;
    for (const auto & buffer : buffers_) {
      if (buffer.mode != nvinfer1::TensorIOMode::kOUTPUT) {
        continue;
      }
      if (best == nullptr || buffer.elem_count > best->elem_count) {
        best = &buffer;
      }
    }
    return best;
  }

  void append_xyxy_detection(
    std::vector<Detection> & detections,
    float x1, float y1, float x2, float y2,
    float score, int class_id,
    const LetterboxInfo & info) const
  {
    if (class_id != person_class_id_ || score < conf_threshold_) {
      return;
    }

    if (std::max(std::max(std::fabs(x1), std::fabs(y1)), std::max(std::fabs(x2), std::fabs(y2))) <= 2.0f) {
      x1 *= static_cast<float>(input_width_);
      x2 *= static_cast<float>(input_width_);
      y1 *= static_cast<float>(input_height_);
      y2 *= static_cast<float>(input_height_);
    }

    Detection det;
    det.x1 = clamp_float((x1 - info.pad_x) / info.scale, 0.0f, static_cast<float>(info.original_width - 1));
    det.y1 = clamp_float((y1 - info.pad_y) / info.scale, 0.0f, static_cast<float>(info.original_height - 1));
    det.x2 = clamp_float((x2 - info.pad_x) / info.scale, 0.0f, static_cast<float>(info.original_width - 1));
    det.y2 = clamp_float((y2 - info.pad_y) / info.scale, 0.0f, static_cast<float>(info.original_height - 1));
    det.score = score;
    det.class_id = class_id;

    if (det.x2 <= det.x1 || det.y2 <= det.y1) {
      return;
    }
    const float area = (det.x2 - det.x1) * (det.y2 - det.y1);
    if (area < min_person_area_px_) {
      return;
    }
    detections.push_back(det);
  }

  void append_xywh_detection(
    std::vector<Detection> & detections,
    float cx, float cy, float w, float h,
    float score, int class_id,
    const LetterboxInfo & info) const
  {
    if (std::max(std::max(std::fabs(cx), std::fabs(cy)), std::max(std::fabs(w), std::fabs(h))) <= 2.0f) {
      cx *= static_cast<float>(input_width_);
      w *= static_cast<float>(input_width_);
      cy *= static_cast<float>(input_height_);
      h *= static_cast<float>(input_height_);
    }
    append_xyxy_detection(
      detections,
      cx - w * 0.5f, cy - h * 0.5f,
      cx + w * 0.5f, cy + h * 0.5f,
      score, class_id, info);
  }

  std::vector<Detection> parse_yolo_output(
    const TensorBuffer & output,
    const LetterboxInfo & info)
  {
    const std::vector<float> data = output_as_float(output);
    const std::vector<int64_t> shape = dims_to_vector(output.dims);
    std::vector<Detection> detections;

    if (shape.size() < 2 || data.empty()) {
      return detections;
    }

    int64_t rows = 0;
    int64_t attrs = 0;
    bool channel_first = false;

    if (shape.size() == 3 && shape[0] == 1) {
      const int64_t a = shape[1];
      const int64_t b = shape[2];
      if (a <= 256 && b > a) {
        attrs = a;
        rows = b;
        channel_first = true;
      } else {
        rows = a;
        attrs = b;
      }
    } else if (shape.size() == 2) {
      const int64_t a = shape[0];
      const int64_t b = shape[1];
      if (a <= 256 && b > a) {
        attrs = a;
        rows = b;
        channel_first = true;
      } else {
        rows = a;
        attrs = b;
      }
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Unsupported YOLO output shape %s",
        shape_to_string(output.dims).c_str());
      return detections;
    }

    if (attrs == 6 || attrs == 7) {
      for (int64_t i = 0; i < rows; ++i) {
        const auto get = [&](int64_t attr) {
          if (channel_first) {
            return data[static_cast<size_t>(attr * rows + i)];
          }
          return data[static_cast<size_t>(i * attrs + attr)];
        };
        const float x1 = get(0);
        const float y1 = get(1);
        const float x2 = get(2);
        const float y2 = get(3);
        const float score = get(4);
        const int class_id = static_cast<int>(std::round(get(5)));
        append_xyxy_detection(detections, x1, y1, x2, y2, score, class_id, info);
      }
    } else if (attrs >= 6) {
      const bool has_objectness = attrs >= 85;
      const int class_offset = has_objectness ? 5 : 4;
      const int person_attr = class_offset + person_class_id_;
      if (person_attr >= attrs) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "person_class_id=%d is outside output attrs=%ld",
          person_class_id_, static_cast<long>(attrs));
        return detections;
      }

      for (int64_t i = 0; i < rows; ++i) {
        const auto get = [&](int64_t attr) {
          if (channel_first) {
            return data[static_cast<size_t>(attr * rows + i)];
          }
          return data[static_cast<size_t>(i * attrs + attr)];
        };
        float score = get(person_attr);
        if (has_objectness) {
          score *= get(4);
        }
        append_xywh_detection(
          detections, get(0), get(1), get(2), get(3),
          score, person_class_id_, info);
      }
    }

    return non_max_suppression(detections);
  }

  std::vector<Detection> non_max_suppression(std::vector<Detection> detections) const
  {
    std::sort(
      detections.begin(), detections.end(),
      [](const Detection & a, const Detection & b) { return a.score > b.score; });

    std::vector<Detection> kept;
    std::vector<bool> removed(detections.size(), false);
    for (size_t i = 0; i < detections.size(); ++i) {
      if (removed[i]) {
        continue;
      }
      kept.push_back(detections[i]);
      if (static_cast<int>(kept.size()) >= max_detections_) {
        break;
      }
      for (size_t j = i + 1; j < detections.size(); ++j) {
        if (!removed[j] && intersection_over_union(detections[i], detections[j]) > nms_iou_threshold_) {
          removed[j] = true;
        }
      }
    }
    return kept;
  }

  Detection torso_from_person(const Detection & person) const
  {
    const float w = person.x2 - person.x1;
    const float h = person.y2 - person.y1;
    Detection torso = person;
    torso.x1 = person.x1 + w * static_cast<float>(torso_side_crop_ratio_);
    torso.x2 = person.x2 - w * static_cast<float>(torso_side_crop_ratio_);
    torso.y1 = person.y1 + h * static_cast<float>(torso_top_ratio_);
    torso.y2 = person.y1 + h * static_cast<float>(torso_bottom_ratio_);
    return torso;
  }

  Detection region_from_body_band(
    const Detection & person,
    double side_crop_ratio,
    double band_top_ratio,
    double band_bottom_ratio,
    double inner_top_ratio,
    double inner_bottom_ratio) const
  {
    const float w = person.x2 - person.x1;
    const float h = person.y2 - person.y1;
    const float side = clamp_float(static_cast<float>(side_crop_ratio), 0.0f, 0.45f);
    const float band_top = clamp_float(static_cast<float>(band_top_ratio), 0.0f, 1.0f);
    const float band_bottom =
      clamp_float(static_cast<float>(band_bottom_ratio), band_top + 0.02f, 1.0f);
    const float inner_top = clamp_float(static_cast<float>(inner_top_ratio), 0.0f, 1.0f);
    const float inner_bottom =
      clamp_float(static_cast<float>(inner_bottom_ratio), inner_top + 0.02f, 1.0f);
    const float band_height = band_bottom - band_top;
    const float top = band_top + band_height * inner_top;
    const float bottom = band_top + band_height * inner_bottom;

    Detection region = person;
    region.x1 = person.x1 + w * side;
    region.x2 = person.x2 - w * side;
    region.y1 = person.y1 + h * top;
    region.y2 = person.y1 + h * bottom;
    return region;
  }

  Detection upper_color_region_from_person(const Detection & person) const
  {
    return region_from_body_band(
      person, upper_color_side_crop_ratio_,
      upper_color_band_top_ratio_, upper_color_band_bottom_ratio_,
      upper_color_inner_top_ratio_, upper_color_inner_bottom_ratio_);
  }

  Detection lower_color_region_from_person(const Detection & person) const
  {
    return region_from_body_band(
      person, lower_color_side_crop_ratio_,
      lower_color_band_top_ratio_, lower_color_band_bottom_ratio_,
      lower_color_inner_top_ratio_, lower_color_inner_bottom_ratio_);
  }

  Detection lower_body_from_person(const Detection & person) const
  {
    const float w = person.x2 - person.x1;
    const float h = person.y2 - person.y1;
    Detection lower = person;
    lower.x1 = person.x1 + w * 0.20f;
    lower.x2 = person.x2 - w * 0.20f;
    lower.y1 = person.y1 + h * 0.55f;
    lower.y2 = person.y1 + h * 0.95f;
    return lower;
  }

  std::array<float, kHueBins> compute_hue_histogram(const cv::Mat & hsv_roi) const
  {
    std::array<float, kHueBins> hist{};
    float weight_sum = 0.0f;
    for (int y = 0; y < hsv_roi.rows; ++y) {
      const cv::Vec3b * row = hsv_roi.ptr<cv::Vec3b>(y);
      for (int x = 0; x < hsv_roi.cols; ++x) {
        const int hue = row[x][0];
        const int saturation = row[x][1];
        const int value = row[x][2];
        if (saturation < color_saturation_min_ || value < color_value_min_) {
          continue;
        }
        const int bin = std::max(0, std::min(kHueBins - 1, hue * kHueBins / 180));
        const float sat_norm = static_cast<float>(saturation) / 255.0f;
        const float value_norm = static_cast<float>(value) / 255.0f;
        const float weight = sat_norm * sat_norm * std::sqrt(value_norm);
        hist[static_cast<size_t>(bin)] += weight;
        weight_sum += weight;
      }
    }

    if (weight_sum > 1e-5f) {
      for (float & value : hist) {
        value /= weight_sum;
      }
    }
    return hist;
  }

  cv::Scalar weighted_hsv_mean(const cv::Mat & hsv_roi) const
  {
    double hue_x = 0.0;
    double hue_y = 0.0;
    double sat_sum = 0.0;
    double value_sum = 0.0;
    double weight_sum = 0.0;

    for (int y = 0; y < hsv_roi.rows; ++y) {
      const cv::Vec3b * row = hsv_roi.ptr<cv::Vec3b>(y);
      for (int x = 0; x < hsv_roi.cols; ++x) {
        const double hue = row[x][0];
        const double saturation = row[x][1];
        const double value = row[x][2];
        if (saturation < color_saturation_min_ || value < color_value_min_) {
          continue;
        }

        const double sat_norm = saturation / 255.0;
        const double value_norm = value / 255.0;
        const double weight = sat_norm * sat_norm * std::sqrt(value_norm);
        const double angle = hue * 2.0 * kPi / 180.0;
        hue_x += std::cos(angle) * weight;
        hue_y += std::sin(angle) * weight;
        sat_sum += saturation * weight;
        value_sum += value * weight;
        weight_sum += weight;
      }
    }

    if (weight_sum <= 1e-5) {
      return cv::mean(hsv_roi);
    }

    double hue = std::atan2(hue_y, hue_x) * 180.0 / (2.0 * kPi);
    if (hue < 0.0) {
      hue += 180.0;
    }
    return cv::Scalar(hue, sat_sum / weight_sum, value_sum / weight_sum, 0.0);
  }

  size_t dominant_hue_index(const std::array<float, kHueBins> & hue_hist) const
  {
    size_t max_index = 0;
    for (size_t i = 1; i < hue_hist.size(); ++i) {
      if (hue_hist[i] > hue_hist[max_index]) {
        max_index = i;
      }
    }
    return max_index;
  }

  float dominant_hue_value(const std::array<float, kHueBins> & hue_hist) const
  {
    const size_t index = dominant_hue_index(hue_hist);
    if (hue_hist[index] <= 1e-5f) {
      return -1.0f;
    }
    return static_cast<float>(
      (static_cast<double>(index) + 0.5) * (180.0 / static_cast<double>(kHueBins)));
  }

  float histogram_distance(
    const std::array<float, kHueBins> & a,
    const std::array<float, kHueBins> & b) const
  {
    float sum_a = 0.0f;
    float sum_b = 0.0f;
    float distance = 0.0f;
    for (size_t i = 0; i < a.size(); ++i) {
      sum_a += a[i];
      sum_b += b[i];
      distance += std::fabs(a[i] - b[i]);
    }
    if (sum_a <= 1e-5f && sum_b <= 1e-5f) {
      return 0.0f;
    }
    if (sum_a <= 1e-5f || sum_b <= 1e-5f) {
      return 1.0f;
    }
    return std::min(1.0f, distance * 0.5f);
  }

  bool has_purple_bgr_bias(
    const cv::Scalar & mean_bgr,
    double saturation,
    double value) const
  {
    if (saturation < 28.0 || value < 50.0) {
      return false;
    }

    const double blue = mean_bgr[0];
    const double green = mean_bgr[1];
    const double red = mean_bgr[2];
    const double red_blue_average = (red + blue) * 0.5;

    return red >= 55.0 &&
           blue >= 55.0 &&
           red >= green * 0.82 &&
           blue >= green * 0.90 &&
           red_blue_average >= green + 12.0;
  }

  std::string color_name_from_hue(
    double hue,
    double saturation,
    double value,
    bool purple_bgr_bias) const
  {
    const bool pastel = value >= 140.0 && saturation <= 125.0;
    const bool dark = value <= 95.0;

    if (hue < 4.0 || hue >= 176.0) {
      return pastel ? "pink_red" : "red";
    }
    if (hue < 11.0) {
      return "red_orange";
    }
    if (hue < 22.0) {
      return dark ? "brown_orange" : "orange";
    }
    if (hue < 34.0) {
      return pastel ? "light_yellow" : "yellow";
    }
    if (hue < 48.0) {
      return "yellow_green";
    }
    if (hue < 78.0) {
      return dark ? "dark_green" : "green";
    }
    if (hue < 88.0) {
      return "teal";
    }
    if (hue < 98.0) {
      return pastel ? "light_cyan" : "cyan";
    }
    if (hue < 113.0) {
      if (purple_bgr_bias) {
        return pastel ? "lavender_blue" : "blue_violet";
      }
      return pastel ? "light_blue" : "sky_blue";
    }
    if (hue < 124.0) {
      if (purple_bgr_bias) {
        return pastel ? "lavender_blue" : "blue_violet";
      }
      return dark ? "navy_blue" : "blue";
    }
    if (hue < 135.0) {
      return pastel ? "lavender_blue" : "blue_violet";
    }
    if (hue < 154.0) {
      return pastel ? "lavender" : "purple";
    }
    if (hue < 166.0) {
      return pastel ? "pink" : "magenta";
    }
    if (hue < 176.0) {
      return pastel ? "pink" : "rose";
    }
    return "red";
  }

  std::string dominant_color_name(
    const std::array<float, kHueBins> & hue_hist,
    const cv::Scalar & mean_hsv,
    const cv::Scalar & mean_bgr) const
  {
    const double saturation = mean_hsv[1];
    const double value = mean_hsv[2];
    if (value < 42.0) {
      return "black";
    }
    if (saturation < 18.0) {
      return value > 190.0 ? "white" : "gray";
    }

    double hue = dominant_hue_value(hue_hist);
    if (hue < 0.0) {
      hue = mean_hsv[0];
    }
    const bool purple_bgr_bias = has_purple_bgr_bias(mean_bgr, saturation, value);
    return color_name_from_hue(hue, saturation, value, purple_bgr_bias);
  }

  float histogram_confidence(const std::array<float, kHueBins> & hue_hist) const
  {
    return hue_hist[dominant_hue_index(hue_hist)];
  }

  std::string simplified_color_name(const std::string & color) const
  {
    if (color.find("purple") != std::string::npos ||
      color.find("violet") != std::string::npos ||
      color.find("lavender") != std::string::npos)
    {
      return "purple";
    }
    if (color.find("blue") != std::string::npos || color.find("cyan") != std::string::npos) {
      return "blue";
    }
    if (color.find("green") != std::string::npos || color == "teal") {
      return "green";
    }
    if (color.find("yellow") != std::string::npos || color.find("orange") != std::string::npos ||
      color.find("brown") != std::string::npos)
    {
      return "warm";
    }
    if (color.find("pink") != std::string::npos || color == "magenta" || color == "rose") {
      return "red_pink";
    }
    return color;
  }

  void extract_color_feature(
    const cv::Mat & image,
    const Detection & region,
    cv::Scalar & mean_bgr,
    cv::Scalar & stddev_bgr,
    cv::Scalar & mean_hsv,
    std::array<float, kHueBins> & hue_hist,
    std::string & dominant_color,
    float & dominant_hue,
    float & color_confidence) const
  {
    const cv::Rect rect = detection_to_rect(region, image.cols, image.rows);
    if (rect.width < 4 || rect.height < 4) {
      return;
    }

    const cv::Mat roi = image(rect);
    cv::Mat hsv_roi;
    cv::cvtColor(roi, hsv_roi, cv::COLOR_BGR2HSV);
    cv::meanStdDev(roi, mean_bgr, stddev_bgr);
    mean_hsv = weighted_hsv_mean(hsv_roi);
    hue_hist = compute_hue_histogram(hsv_roi);
    dominant_hue = dominant_hue_value(hue_hist);
    if (dominant_hue < 0.0f) {
      dominant_hue = static_cast<float>(mean_hsv[0]);
    }
    color_confidence = histogram_confidence(hue_hist);
    dominant_color = dominant_color_name(hue_hist, mean_hsv, mean_bgr);
  }

  PersonFeature extract_feature(
    const cv::Mat & image,
    const Detection & person) const
  {
    PersonFeature feature;
    feature.person = person;
    feature.torso = torso_from_person(person);
    feature.lower_body = lower_body_from_person(person);
    feature.upper_color_region = upper_color_region_from_person(person);
    feature.lower_color_region = lower_color_region_from_person(person);
    feature.center_x = (person.x1 + person.x2) * 0.5f;
    feature.center_y = (person.y1 + person.y2) * 0.5f;

    const float frame_area = std::max(1.0f, static_cast<float>(image.cols * image.rows));
    feature.person_height_px = detection_height(person);
    feature.person_height_px_smooth = feature.person_height_px;
    feature.person_height_ratio =
      feature.person_height_px / std::max(1.0f, static_cast<float>(image.rows));
    feature.person_height_ratio_smooth = feature.person_height_ratio;
    feature.person_area_ratio = detection_area(person) / frame_area;
    feature.torso_area_ratio = detection_area(feature.torso) / frame_area;
    feature.aspect_ratio = detection_width(person) / std::max(1.0f, detection_height(person));

    extract_color_feature(
      image, feature.upper_color_region, feature.mean_bgr, feature.stddev_bgr,
      feature.mean_hsv, feature.hue_hist, feature.dominant_color,
      feature.dominant_hue, feature.color_confidence);
    extract_color_feature(
      image, feature.lower_color_region, feature.lower_mean_bgr, feature.lower_stddev_bgr,
      feature.lower_mean_hsv, feature.lower_hue_hist, feature.lower_dominant_color,
      feature.lower_dominant_hue, feature.lower_color_confidence);

    return feature;
  }

  std::vector<PersonFeature> extract_features(
    const cv::Mat & image,
    const std::vector<Detection> & persons) const
  {
    std::vector<PersonFeature> features;
    features.reserve(persons.size());
    for (const auto & person : persons) {
      features.push_back(extract_feature(image, person));
    }
    return features;
  }

  double normalized_center_distance(
    const PersonFeature & feature,
    const TrackState & track,
    int image_width,
    int image_height) const
  {
    const double dx =
      static_cast<double>(feature.center_x - track.center_x) /
      std::max(1.0, static_cast<double>(image_width));
    const double dy =
      static_cast<double>(feature.center_y - track.center_y) /
      std::max(1.0, static_cast<double>(image_height));
    return std::min(1.0, std::sqrt(dx * dx + dy * dy) * 2.0);
  }

  double track_match_score(
    const PersonFeature & feature,
    const TrackState & track,
    int image_width,
    int image_height) const
  {
    const double iou = intersection_over_union(feature.person, track.person);
    const double appearance =
      0.45 * histogram_distance(feature.hue_hist, track.hue_hist) +
      0.55 * histogram_distance(feature.lower_hue_hist, track.lower_hue_hist);
    const double size =
      std::min(1.0, static_cast<double>(
        std::fabs(feature.person_height_ratio - track.person_height_ratio)) * 3.0);
    const double center = normalized_center_distance(feature, track, image_width, image_height);

    return track_iou_weight_ * (1.0 - iou) +
           track_appearance_weight_ * appearance +
           track_size_weight_ * size +
           track_center_weight_ * center;
  }

  void update_track_from_feature(TrackState & track, PersonFeature & feature)
  {
    const float color_old_weight = track.missed == 0 ? 0.70f : 0.35f;
    const float color_new_weight = 1.0f - color_old_weight;
    const float height_alpha =
      static_cast<float>(clamp_float(
        static_cast<float>(height_smoothing_alpha_), 0.01f, 1.0f));
    feature.track_id = track.id;
    track.person = feature.person;
    track.center_x = feature.center_x;
    track.center_y = feature.center_y;
    if (track.smoothed_height_px <= 1e-3f) {
      track.smoothed_height_px = feature.person_height_px;
    } else {
      track.smoothed_height_px =
        (1.0f - height_alpha) * track.smoothed_height_px +
        height_alpha * feature.person_height_px;
    }
    track.smoothed_height_ratio =
      (1.0f - height_alpha) * track.smoothed_height_ratio +
      height_alpha * feature.person_height_ratio;
    feature.person_height_px_smooth = track.smoothed_height_px;
    feature.person_height_ratio_smooth = track.smoothed_height_ratio;
    track.person_height_ratio =
      color_old_weight * track.person_height_ratio + color_new_weight * feature.person_height_ratio;
    track.mean_hsv =
      track.mean_hsv * static_cast<double>(color_old_weight) +
      feature.mean_hsv * static_cast<double>(color_new_weight);
    track.lower_mean_hsv =
      track.lower_mean_hsv * static_cast<double>(color_old_weight) +
      feature.lower_mean_hsv * static_cast<double>(color_new_weight);
    for (size_t i = 0; i < track.hue_hist.size(); ++i) {
      track.hue_hist[i] =
        color_old_weight * track.hue_hist[i] + color_new_weight * feature.hue_hist[i];
      track.lower_hue_hist[i] =
        color_old_weight * track.lower_hue_hist[i] + color_new_weight * feature.lower_hue_hist[i];
    }
    track.missed = 0;
  }

  void create_track_from_feature(PersonFeature & feature)
  {
    TrackState track;
    track.id = next_track_id_++;
    track.person = feature.person;
    track.center_x = feature.center_x;
    track.center_y = feature.center_y;
    track.person_height_ratio = feature.person_height_ratio;
    track.smoothed_height_px = feature.person_height_px;
    track.smoothed_height_ratio = feature.person_height_ratio;
    feature.person_height_px_smooth = feature.person_height_px;
    feature.person_height_ratio_smooth = feature.person_height_ratio;
    track.mean_hsv = feature.mean_hsv;
    track.hue_hist = feature.hue_hist;
    track.lower_mean_hsv = feature.lower_mean_hsv;
    track.lower_hue_hist = feature.lower_hue_hist;
    track.missed = 0;
    feature.track_id = track.id;
    tracks_.push_back(track);
  }

  void update_tracks(std::vector<PersonFeature> & features, int image_width, int image_height)
  {
    std::vector<bool> feature_used(features.size(), false);
    std::vector<bool> track_used(tracks_.size(), false);
    std::vector<TrackCandidate> candidates;

    for (size_t feature_index = 0; feature_index < features.size(); ++feature_index) {
      for (size_t track_index = 0; track_index < tracks_.size(); ++track_index) {
        TrackCandidate candidate;
        candidate.feature_index = feature_index;
        candidate.track_index = track_index;
        candidate.score =
          track_match_score(features[feature_index], tracks_[track_index], image_width, image_height);
        candidates.push_back(candidate);
      }
    }

    std::sort(
      candidates.begin(), candidates.end(),
      [](const TrackCandidate & a, const TrackCandidate & b) {
        return a.score < b.score;
      });

    for (const auto & candidate : candidates) {
      if (candidate.score > track_match_threshold_) {
        break;
      }
      if (feature_used[candidate.feature_index] || track_used[candidate.track_index]) {
        continue;
      }
      update_track_from_feature(tracks_[candidate.track_index], features[candidate.feature_index]);
      feature_used[candidate.feature_index] = true;
      track_used[candidate.track_index] = true;
    }

    for (size_t track_index = 0; track_index < tracks_.size(); ++track_index) {
      if (!track_used[track_index]) {
        tracks_[track_index].missed++;
      }
    }

    for (size_t feature_index = 0; feature_index < features.size(); ++feature_index) {
      if (!feature_used[feature_index]) {
        create_track_from_feature(features[feature_index]);
      }
    }

    const int base_max_missed = std::max(1, track_max_missed_);
    const int locked_max_missed = base_max_missed * 4;
    std::vector<TrackState> kept_tracks;
    kept_tracks.reserve(tracks_.size());
    bool locked_track_kept = locked_track_id_ < 0;
    for (const auto & track : tracks_) {
      const int allowed_missed = track.id == locked_track_id_ ? locked_max_missed : base_max_missed;
      if (track.missed <= allowed_missed) {
        if (track.id == locked_track_id_) {
          locked_track_kept = true;
        }
        kept_tracks.push_back(track);
      }
    }
    tracks_.swap(kept_tracks);
    if (!locked_track_kept) {
      locked_track_id_ = -1;
    }
  }

  int select_center_track(
    const std::vector<PersonFeature> & features,
    int image_width,
    int image_height) const
  {
    if (features.empty()) {
      return -1;
    }

    const double center_x = static_cast<double>(image_width) * 0.5;
    const double center_y = static_cast<double>(image_height) * 0.5;
    const double norm = std::max(1.0, std::hypot(center_x, center_y));
    double best_score = std::numeric_limits<double>::max();
    int best_id = -1;
    for (const auto & feature : features) {
      const double dx = static_cast<double>(feature.center_x) - center_x;
      const double dy = static_cast<double>(feature.center_y) - center_y;
      const double score =
        std::hypot(dx, dy) / norm -
        static_cast<double>(feature.person_area_ratio) * 0.35 -
        static_cast<double>(feature.person.score) * 0.10;
      if (score < best_score) {
        best_score = score;
        best_id = feature.track_id;
      }
    }
    return best_id;
  }

  int select_largest_track(const std::vector<PersonFeature> & features) const
  {
    float best_area = -1.0f;
    int best_id = -1;
    for (const auto & feature : features) {
      const float area = detection_area(feature.person);
      if (area > best_area) {
        best_area = area;
        best_id = feature.track_id;
      }
    }
    return best_id;
  }

  int parse_track_id_command(const std::string & command) const
  {
    std::string value = command;
    if (value.find("track:") == 0) {
      value = value.substr(6);
    } else if (value.find("id:") == 0) {
      value = value.substr(3);
    }

    char * end = nullptr;
    const long parsed = std::strtol(value.c_str(), &end, 10);
    if (end != nullptr && *end == '\0' && parsed > 0 && parsed < std::numeric_limits<int>::max()) {
      return static_cast<int>(parsed);
    }
    return -1;
  }

  void handle_lock_command(
    const std::vector<PersonFeature> & features,
    int image_width,
    int image_height)
  {
    if (has_pending_lock_command_) {
      const std::string command = lower_copy(trim_copy(pending_lock_command_));
      has_pending_lock_command_ = false;

      if (command == "clear" || command == "unlock" || command == "none") {
        locked_track_id_ = -1;
        RCLCPP_INFO(get_logger(), "Torso target lock cleared");
      } else if (command == "center" || command == "lock_center") {
        locked_track_id_ = select_center_track(features, image_width, image_height);
        RCLCPP_INFO(get_logger(), "Torso target locked by center: id=%d", locked_track_id_);
      } else if (command == "largest" || command == "lock_largest") {
        locked_track_id_ = select_largest_track(features);
        RCLCPP_INFO(get_logger(), "Torso target locked by largest area: id=%d", locked_track_id_);
      } else {
        const int parsed_id = parse_track_id_command(command);
        if (parsed_id > 0) {
          locked_track_id_ = parsed_id;
          RCLCPP_INFO(get_logger(), "Torso target locked by explicit id=%d", locked_track_id_);
        } else {
          RCLCPP_WARN(get_logger(), "Unknown torso lock command: %s", command.c_str());
        }
      }
    }

    if (auto_lock_center_ && locked_track_id_ < 0 && !features.empty()) {
      locked_track_id_ = select_center_track(features, image_width, image_height);
      RCLCPP_INFO(get_logger(), "Torso target auto-locked by center: id=%d", locked_track_id_);
    }
  }

  const PersonFeature * find_locked_feature(const std::vector<PersonFeature> & features) const
  {
    if (locked_track_id_ < 0) {
      return nullptr;
    }
    for (const auto & feature : features) {
      if (feature.track_id == locked_track_id_) {
        return &feature;
      }
    }
    return nullptr;
  }

  void append_torso_detection(
    vision_msgs::msg::Detection2DArray & msg,
    const std_msgs::msg::Header & header,
    const PersonFeature & feature,
    bool locked) const
  {
    vision_msgs::msg::Detection2D det;
    det.header = header;
    det.bbox.center.position.x = (feature.torso.x1 + feature.torso.x2) * 0.5;
    det.bbox.center.position.y = (feature.torso.y1 + feature.torso.y2) * 0.5;
    det.bbox.center.theta = 0.0;
    det.bbox.size_x = std::max(0.0f, feature.torso.x2 - feature.torso.x1);
    det.bbox.size_y = std::max(0.0f, feature.torso.y2 - feature.torso.y1);

    vision_msgs::msg::ObjectHypothesisWithPose hyp;
    hyp.hypothesis.class_id = locked ? "locked_torso" : "torso";
    hyp.hypothesis.score = feature.person.score;
    det.results.push_back(hyp);
    msg.detections.push_back(det);
  }

  void publish_detections(
    const std_msgs::msg::Header & header,
    const std::vector<PersonFeature> & features)
  {
    vision_msgs::msg::Detection2DArray msg;
    msg.header = header;

    for (const auto & feature : features) {
      append_torso_detection(msg, header, feature, false);
    }

    detections_pub_->publish(msg);
  }

  void publish_locked_target(
    const std_msgs::msg::Header & header,
    const std::vector<PersonFeature> & features)
  {
    vision_msgs::msg::Detection2DArray msg;
    msg.header = header;
    const PersonFeature * locked_feature = find_locked_feature(features);
    if (locked_feature != nullptr) {
      append_torso_detection(msg, header, *locked_feature, true);
    }
    locked_pub_->publish(msg);
  }

  void append_histogram_json(
    std::ostringstream & out,
    const std::array<float, kHueBins> & histogram) const
  {
    out << "[";
    for (size_t i = 0; i < histogram.size(); ++i) {
      if (i > 0) {
        out << ",";
      }
      out << histogram[i];
    }
    out << "]";
  }

  void append_box_json(
    std::ostringstream & out,
    const Detection & detection,
    double scale_x = 1.0,
    double scale_y = 1.0) const
  {
    out << "["
        << detection.x1 * scale_x << ","
        << detection.y1 * scale_y << ","
        << detection.x2 * scale_x << ","
        << detection.y2 * scale_y << "]";
  }

  std::string hue_peaks_string(const std::array<float, kHueBins> & histogram, int max_count) const
  {
    std::vector<size_t> indexes(histogram.size());
    std::iota(indexes.begin(), indexes.end(), 0);
    std::sort(
      indexes.begin(), indexes.end(),
      [&](size_t a, size_t b) {
        return histogram[a] > histogram[b];
      });

    std::ostringstream out;
    out << "[";
    const int count = std::min(max_count, static_cast<int>(indexes.size()));
    for (int i = 0; i < count; ++i) {
      const size_t index = indexes[static_cast<size_t>(i)];
      if (histogram[index] <= 1e-5f) {
        break;
      }
      if (i > 0) {
        out << ",";
      }
      const double hue =
        (static_cast<double>(index) + 0.5) * (180.0 / static_cast<double>(kHueBins));
      out << std::fixed << std::setprecision(1) << hue << ":" << std::setprecision(2) << histogram[index];
    }
    out << "]";
    return out.str();
  }

  void publish_features(
    const std_msgs::msg::Header & header,
    const std::vector<PersonFeature> & features,
    int image_width,
    int image_height)
  {
    const PersonFeature * locked_feature = find_locked_feature(features);
    const double feature_scale_x =
      image_width > 0 ? skeleton_base_width_ / static_cast<double>(image_width) : 1.0;
    const double feature_scale_y =
      image_height > 0 ? skeleton_base_height_ / static_cast<double>(image_height) : 1.0;
    std::ostringstream out;
    out << std::fixed << std::setprecision(3);
    out << "{";
    out << "\"stamp\":{\"sec\":" << header.stamp.sec
        << ",\"nanosec\":" << header.stamp.nanosec << "},";
    out << "\"locked_track_id\":" << locked_track_id_ << ",";
    out << "\"locked_visible\":" << (locked_feature != nullptr ? "true" : "false") << ",";
    out << "\"targets\":[";
    for (size_t i = 0; i < features.size(); ++i) {
      const auto & feature = features[i];
      if (i > 0) {
        out << ",";
      }
      out << "{";
      out << "\"track_id\":" << feature.track_id << ",";
      out << "\"locked\":" << (feature.track_id == locked_track_id_ ? "true" : "false") << ",";
      out << "\"score\":" << feature.person.score << ",";
      out << "\"dominant_color\":\"" << feature.dominant_color << "\",";
      out << "\"dominant_color_group\":\"" << simplified_color_name(feature.dominant_color) << "\",";
      out << "\"lower_dominant_color\":\"" << feature.lower_dominant_color << "\",";
      out << "\"lower_dominant_color_group\":\"" << simplified_color_name(feature.lower_dominant_color) << "\",";
      out << "\"appearance_label\":\"upper:" << feature.dominant_color
          << ",lower:" << feature.lower_dominant_color << "\",";
      out << "\"dominant_hue\":" << feature.dominant_hue << ",";
      out << "\"lower_dominant_hue\":" << feature.lower_dominant_hue << ",";
      out << "\"color_confidence\":" << feature.color_confidence << ",";
      out << "\"lower_color_confidence\":" << feature.lower_color_confidence << ",";
      out << "\"hue_peaks\":\"" << hue_peaks_string(feature.hue_hist, 3) << "\",";
      out << "\"lower_hue_peaks\":\"" << hue_peaks_string(feature.lower_hue_hist, 3) << "\",";
      out << "\"mean_bgr\":["
          << feature.mean_bgr[0] << ","
          << feature.mean_bgr[1] << ","
          << feature.mean_bgr[2] << "],";
      out << "\"lower_mean_bgr\":["
          << feature.lower_mean_bgr[0] << ","
          << feature.lower_mean_bgr[1] << ","
          << feature.lower_mean_bgr[2] << "],";
      out << "\"mean_hsv\":["
          << feature.mean_hsv[0] << ","
          << feature.mean_hsv[1] << ","
          << feature.mean_hsv[2] << "],";
      out << "\"lower_mean_hsv\":["
          << feature.lower_mean_hsv[0] << ","
          << feature.lower_mean_hsv[1] << ","
          << feature.lower_mean_hsv[2] << "],";
      out << "\"hue_hist\":";
      append_histogram_json(out, feature.hue_hist);
      out << ",";
      out << "\"lower_hue_hist\":";
      append_histogram_json(out, feature.lower_hue_hist);
      out << ",";
      out << "\"height_px\":" << feature.person_height_px * feature_scale_y << ",";
      out << "\"height_px_smooth\":"
          << feature.person_height_px_smooth * feature_scale_y << ",";
      out << "\"height_ratio\":" << feature.person_height_ratio << ",";
      out << "\"height_ratio_smooth\":" << feature.person_height_ratio_smooth << ",";
      out << "\"depth_m\":" << feature.depth_m << ",";
      out << "\"height_m\":" << feature.height_m << ",";
      out << "\"height_source\":\"" << feature.height_source << "\",";
      out << "\"person_area_ratio\":" << feature.person_area_ratio << ",";
      out << "\"torso_area_ratio\":" << feature.torso_area_ratio << ",";
      out << "\"aspect_ratio\":" << feature.aspect_ratio << ",";
      out << "\"center\":["
          << feature.center_x * feature_scale_x << ","
          << feature.center_y * feature_scale_y << "],";
      out << "\"person_bbox\":";
      append_box_json(out, feature.person, feature_scale_x, feature_scale_y);
      out << ",\"torso_bbox\":";
      append_box_json(out, feature.torso, feature_scale_x, feature_scale_y);
      out << ",\"lower_bbox\":";
      append_box_json(out, feature.lower_body, feature_scale_x, feature_scale_y);
      out << ",\"upper_color_bbox\":";
      append_box_json(out, feature.upper_color_region, feature_scale_x, feature_scale_y);
      out << ",\"lower_color_bbox\":";
      append_box_json(out, feature.lower_color_region, feature_scale_x, feature_scale_y);
      out << "}";
    }
    out << "]}";

    std_msgs::msg::String msg;
    msg.data = out.str();
    features_pub_->publish(msg);
  }

  void bodylist_callback(const bodyreader_msg::msg::Bodylist::SharedPtr msg)
  {
    latest_bodylist_ = *msg;
    has_bodylist_ = true;
    last_bodylist_time_ = now();
  }

  void bodyposture_callback(const bodyreader_msg::msg::Bodyposture::SharedPtr msg)
  {
    locked_body_id_ = msg->bodyid;
    skeleton_lock_status_ = msg->lock_status;
  }

  bool project_skeleton_joint(
    const bodyreader_msg::msg::Joint & joint,
    int image_width,
    int image_height,
    cv::Point & point) const
  {
    if (skeleton_base_width_ <= 1.0 || skeleton_base_height_ <= 1.0) {
      return false;
    }

    if (joint.depthposition.x <= 0.1f || joint.depthposition.y <= 0.1f) {
      return false;
    }

    const double sx = static_cast<double>(image_width) / skeleton_base_width_;
    const double sy = static_cast<double>(image_height) / skeleton_base_height_;
    const int x = static_cast<int>(std::round(static_cast<double>(joint.depthposition.x) * sx));
    const int y = static_cast<int>(std::round(static_cast<double>(joint.depthposition.y) * sy));
    if (x < 0 || x >= image_width || y < 0 || y >= image_height) {
      return false;
    }

    point = cv::Point(x, y);
    return true;
  }

  void draw_skeleton_line(
    cv::Mat & image,
    const std::array<cv::Point, 19> & points,
    const std::array<bool, 19> & visible,
    int a,
    int b,
    const cv::Scalar & color,
    int thickness) const
  {
    if (a < 0 || b < 0 || a >= 19 || b >= 19 || !visible[static_cast<size_t>(a)] ||
      !visible[static_cast<size_t>(b)])
    {
      return;
    }
    cv::line(
      image,
      points[static_cast<size_t>(a)],
      points[static_cast<size_t>(b)],
      color,
      thickness,
      cv::LINE_AA);
  }

  void draw_skeleton_overlay(cv::Mat & image)
  {
    if (!overlay_skeleton_ || !has_bodylist_) {
      return;
    }

    const double age = (now() - last_bodylist_time_).seconds();
    if (skeleton_max_age_sec_ > 0.0 && age > skeleton_max_age_sec_) {
      cv::putText(
        image, "skeleton stale",
        cv::Point(12, std::max(24, image.rows - 14)),
        cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 180, 255), 1, cv::LINE_AA);
      return;
    }

    const int count = std::min(6, std::max(0, static_cast<int>(latest_bodylist_.count)));
    for (int i = 0; i < count; ++i) {
      const auto & body = latest_bodylist_.bodies[static_cast<size_t>(i)];
      const bool locked = body.bodyid == locked_body_id_ && skeleton_lock_status_ > 0;
      const cv::Scalar line_color = locked ? cv::Scalar(0, 255, 255) : cv::Scalar(245, 245, 245);
      const cv::Scalar joint_color = locked ? cv::Scalar(0, 220, 255) : cv::Scalar(0, 255, 0);
      const int line_thickness = locked ? 3 : 2;
      const int joint_radius = locked ? 5 : 4;

      std::array<cv::Point, 19> points{};
      std::array<bool, 19> visible{};
      for (size_t j = 0; j < points.size(); ++j) {
        visible[j] = project_skeleton_joint(
          body.joints[j], image.cols, image.rows, points[j]);
      }

      for (int a : {0, 1, 2, 3, 5, 6, 8, 9, 10, 11, 13, 14}) {
        draw_skeleton_line(image, points, visible, a, a + 1, line_color, line_thickness);
      }
      draw_skeleton_line(image, points, visible, 1, 5, line_color, line_thickness);
      draw_skeleton_line(image, points, visible, 1, 8, line_color, line_thickness);
      draw_skeleton_line(image, points, visible, 9, 13, line_color, line_thickness);

      for (size_t j = 0; j < points.size(); ++j) {
        if (visible[j]) {
          cv::circle(image, points[j], joint_radius, joint_color, -1, cv::LINE_AA);
        }
      }

      const cv::Point label_anchor =
        visible[1] ? points[1] : (visible[8] ? points[8] : cv::Point(12, 28 + i * 18));
      char label[96];
      std::snprintf(
        label, sizeof(label), "skel:%d%s z:%.2fm",
        body.bodyid,
        locked ? " LOCK" : "",
        static_cast<double>(body.centerofmass.z) * 0.001);
      cv::putText(
        image,
        label,
        cv::Point(std::max(4, label_anchor.x + 6), std::max(16, label_anchor.y - 8)),
        cv::FONT_HERSHEY_SIMPLEX,
        0.5,
        joint_color,
        1,
        cv::LINE_AA);
    }
  }

  void publish_annotated_image(
    const std_msgs::msg::Header & header,
    const cv::Mat & image,
    const std::vector<PersonFeature> & features)
  {
    if (!publish_annotated_) {
      return;
    }
    if (
      annotated_only_if_subscribed_ &&
      annotated_pub_->get_subscription_count() == 0)
    {
      return;
    }

    const auto wall_now = std::chrono::steady_clock::now();
    if (max_annotated_fps_ > 0.0 && last_annotated_wall_time_ != SteadyTimePoint{}) {
      const double elapsed =
        std::chrono::duration<double>(wall_now - last_annotated_wall_time_).count();
      if (elapsed < 1.0 / max_annotated_fps_) {
        return;
      }
    }
    last_annotated_wall_time_ = wall_now;

    cv::Mat annotated = image.clone();
    for (const auto & feature : features) {
      const bool locked = feature.track_id == locked_track_id_;
      const int thickness = locked ? 3 : 2;
      const cv::Scalar person_color = locked ? cv::Scalar(0, 220, 255) : cv::Scalar(0, 180, 0);
      const cv::Scalar torso_color = locked ? cv::Scalar(255, 0, 255) : cv::Scalar(0, 0, 255);
      const cv::Scalar lower_color = locked ? cv::Scalar(255, 255, 0) : cv::Scalar(255, 140, 0);
      const cv::Rect person_rect = detection_to_rect(feature.person, annotated.cols, annotated.rows);
      const cv::Rect torso_rect = detection_to_rect(feature.torso, annotated.cols, annotated.rows);
      const cv::Rect lower_rect = detection_to_rect(feature.lower_body, annotated.cols, annotated.rows);
      const cv::Rect upper_color_rect =
        detection_to_rect(feature.upper_color_region, annotated.cols, annotated.rows);
      const cv::Rect lower_color_rect =
        detection_to_rect(feature.lower_color_region, annotated.cols, annotated.rows);
      cv::rectangle(annotated, person_rect, person_color, thickness);
      cv::rectangle(annotated, torso_rect, torso_color, thickness);
      cv::rectangle(annotated, lower_rect, lower_color, 1);
      cv::rectangle(annotated, upper_color_rect, cv::Scalar(180, 0, 180), 1);
      cv::rectangle(annotated, lower_color_rect, cv::Scalar(255, 180, 0), 1);

      char label[128];
      std::snprintf(
        label, sizeof(label), "id:%d %s/%s %.2f h:%.2f",
        feature.track_id, feature.dominant_color.c_str(), feature.lower_dominant_color.c_str(),
        feature.person.score, feature.person_height_ratio_smooth);
      cv::putText(
        annotated, label,
        cv::Point(torso_rect.x, std::max(12, torso_rect.y - 6)),
        cv::FONT_HERSHEY_SIMPLEX, 0.5, torso_color, 1);
    }

    draw_skeleton_overlay(annotated);
    cv::putText(
      annotated,
      "YOLO + skeleton",
      cv::Point(12, 22),
      cv::FONT_HERSHEY_SIMPLEX,
      0.6,
      cv::Scalar(255, 255, 255),
      1,
      cv::LINE_AA);

    auto out = cv_bridge::CvImage(header, "bgr8", annotated).toImageMsg();
    annotated_pub_->publish(*out);
  }

  void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    latest_depth_msg_ = msg;
  }

  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    if (msg->k[4] > 1.0) {
      camera_fy_px_ = msg->k[4];
    }
  }

  double depth_pixel_to_meters(const cv::Mat & depth, int x, int y, const std::string & encoding) const
  {
    if (x < 0 || y < 0 || x >= depth.cols || y >= depth.rows) {
      return 0.0;
    }

    double depth_m = 0.0;
    if (encoding == "16UC1" || encoding == "mono16") {
      depth_m = static_cast<double>(depth.at<uint16_t>(y, x)) * depth_unit_scale_;
    } else if (encoding == "32FC1") {
      depth_m = static_cast<double>(depth.at<float>(y, x));
    } else if (encoding == "64FC1") {
      depth_m = depth.at<double>(y, x);
    } else if (depth.type() == CV_16UC1) {
      depth_m = static_cast<double>(depth.at<uint16_t>(y, x)) * depth_unit_scale_;
    } else if (depth.type() == CV_32FC1) {
      depth_m = static_cast<double>(depth.at<float>(y, x));
    }

    if (!std::isfinite(depth_m) || depth_m < min_valid_depth_m_ || depth_m > max_valid_depth_m_) {
      return 0.0;
    }
    return depth_m;
  }

  double median_depth_for_region(
    const Detection & region,
    int image_width,
    int image_height,
    const builtin_interfaces::msg::Time & image_stamp) const
  {
    if (!enable_depth_height_ || !latest_depth_msg_) {
      return 0.0;
    }

    if (depth_max_age_sec_ > 0.0) {
      const double age =
        std::fabs((rclcpp::Time(image_stamp) - rclcpp::Time(latest_depth_msg_->header.stamp)).seconds());
      if (age > depth_max_age_sec_) {
        return 0.0;
      }
    }

    cv_bridge::CvImageConstPtr depth_ptr;
    try {
      depth_ptr = cv_bridge::toCvShare(latest_depth_msg_);
    } catch (const cv_bridge::Exception &) {
      return 0.0;
    }

    const cv::Mat & depth = depth_ptr->image;
    if (depth.empty()) {
      return 0.0;
    }

    const cv::Rect color_rect = detection_to_rect(region, image_width, image_height);
    const double sx = static_cast<double>(depth.cols) / std::max(1.0, static_cast<double>(image_width));
    const double sy = static_cast<double>(depth.rows) / std::max(1.0, static_cast<double>(image_height));
    cv::Rect depth_rect(
      std::max(0, static_cast<int>(std::round(color_rect.x * sx))),
      std::max(0, static_cast<int>(std::round(color_rect.y * sy))),
      std::max(1, static_cast<int>(std::round(color_rect.width * sx))),
      std::max(1, static_cast<int>(std::round(color_rect.height * sy))));
    depth_rect &= cv::Rect(0, 0, depth.cols, depth.rows);
    if (depth_rect.width < 2 || depth_rect.height < 2) {
      return 0.0;
    }

    const int x0 = depth_rect.x + depth_rect.width / 4;
    const int x1 = depth_rect.x + depth_rect.width * 3 / 4;
    const int y0 = depth_rect.y + depth_rect.height / 4;
    const int y1 = depth_rect.y + depth_rect.height * 3 / 4;
    const int stride = std::max(1, std::min(depth_rect.width, depth_rect.height) / 24);

    std::vector<double> samples;
    samples.reserve(static_cast<size_t>((x1 - x0 + 1) * (y1 - y0 + 1) / (stride * stride) + 1));
    for (int y = y0; y <= y1; y += stride) {
      for (int x = x0; x <= x1; x += stride) {
        const double depth_m = depth_pixel_to_meters(depth, x, y, latest_depth_msg_->encoding);
        if (depth_m > 0.0) {
          samples.push_back(depth_m);
        }
      }
    }

    if (samples.size() < 5) {
      return 0.0;
    }

    const size_t middle = samples.size() / 2;
    std::nth_element(samples.begin(), samples.begin() + static_cast<long>(middle), samples.end());
    return samples[middle];
  }

  void update_depth_estimates(
    std::vector<PersonFeature> & features,
    int image_width,
    int image_height,
    const builtin_interfaces::msg::Time & image_stamp) const
  {
    for (auto & feature : features) {
      feature.depth_m = static_cast<float>(
        median_depth_for_region(feature.torso, image_width, image_height, image_stamp));
    }
  }

  void update_height_estimates(std::vector<PersonFeature> & features) const
  {
    for (auto & feature : features) {
      feature.height_m = 0.0f;
      feature.height_source = "image_only";

      if (feature.depth_m > 0.0f && camera_fy_px_ > 1.0) {
        feature.height_m =
          static_cast<float>(
            static_cast<double>(feature.person_height_px_smooth) *
            static_cast<double>(feature.depth_m) / camera_fy_px_);
        feature.height_source = "depth_pinhole";
      } else if (height_reference_m_ > 0.0 && height_reference_px_ > 1.0) {
        feature.height_m =
          static_cast<float>(
            static_cast<double>(feature.person_height_px_smooth) *
            height_reference_m_ / height_reference_px_);
        feature.height_source = "reference_px";
      }
    }
  }

  std::string visual_height_label(const PersonFeature & feature) const
  {
    if (feature.person_height_ratio_smooth >= 0.70f) {
      return "very_tall_or_close";
    }
    if (feature.person_height_ratio_smooth >= 0.50f) {
      return "tall_or_near";
    }
    if (feature.person_height_ratio_smooth >= 0.30f) {
      return "medium";
    }
    return "short_or_far";
  }

  void log_image_features(
    const cv::Mat & image,
    const std::vector<PersonFeature> & features,
    double fps) const
  {
    if (!log_image_features_) {
      return;
    }

    const int logged_count =
      std::min(static_cast<int>(features.size()), std::max(0, max_logged_targets_));
    std::ostringstream out;
    out << std::fixed << std::setprecision(2);
    out << "image_features image=" << image.cols << "x" << image.rows
        << " fps=" << fps
        << " targets=" << features.size()
        << " tracks=" << tracks_.size()
        << " locked=" << locked_track_id_;

    if (features.empty()) {
      out << " no_person";
    }

    for (int i = 0; i < logged_count; ++i) {
      const PersonFeature & feature = features[static_cast<size_t>(i)];
      out << " | id=" << feature.track_id
          << (feature.track_id == locked_track_id_ ? "*" : "")
          << " score=" << feature.person.score
          << " upper=" << feature.dominant_color
          << "(h=" << feature.dominant_hue
          << ",s=" << feature.mean_hsv[1]
          << ",v=" << feature.mean_hsv[2]
          << ",conf=" << feature.color_confidence << ")"
          << " upper_peaks=" << hue_peaks_string(feature.hue_hist, 3)
          << " lower=" << feature.lower_dominant_color
          << "(h=" << feature.lower_dominant_hue
          << ",s=" << feature.lower_mean_hsv[1]
          << ",v=" << feature.lower_mean_hsv[2]
          << ",conf=" << feature.lower_color_confidence << ")"
          << " lower_peaks=" << hue_peaks_string(feature.lower_hue_hist, 3)
          << " height_px=" << feature.person_height_px
          << " height_px_smooth=" << feature.person_height_px_smooth
          << " height_ratio=" << feature.person_height_ratio
          << " height_ratio_smooth=" << feature.person_height_ratio_smooth
          << " height_label=" << visual_height_label(feature)
          << " depth_m=" << feature.depth_m
          << " height_m=" << feature.height_m
          << " height_source=" << feature.height_source
          << " area_ratio=" << feature.person_area_ratio
          << " center=(" << feature.center_x << "," << feature.center_y << ")"
          << " bbox=[" << feature.person.x1 << "," << feature.person.y1
          << "," << feature.person.x2 << "," << feature.person.y2 << "]"
          << " upper_color_bbox=[" << feature.upper_color_region.x1 << ","
          << feature.upper_color_region.y1 << ","
          << feature.upper_color_region.x2 << ","
          << feature.upper_color_region.y2 << "]"
          << " lower_color_bbox=[" << feature.lower_color_region.x1 << ","
          << feature.lower_color_region.y1 << ","
          << feature.lower_color_region.x2 << ","
          << feature.lower_color_region.y2 << "]";
    }

    if (static_cast<int>(features.size()) > logged_count) {
      out << " | more_targets=" << (static_cast<int>(features.size()) - logged_count);
    }

    RCLCPP_INFO(get_logger(), "%s", out.str().c_str());
  }

  void mode_callback(const std_msgs::msg::Int8::SharedPtr msg)
  {
    const int next_mode = static_cast<int>(msg->data);
    if (next_mode == current_mode_) {
      return;
    }
    current_mode_ = next_mode;
    if (current_mode_ == mode_required_) {
      tracks_.clear();
      locked_track_id_ = -1;
      locked_body_id_ = -1;
      RCLCPP_INFO(get_logger(), "Torso tracks reset for new follow session");
    }
    last_inference_wall_time_ = SteadyTimePoint{};
    last_annotated_wall_time_ = SteadyTimePoint{};
    RCLCPP_INFO(
      get_logger(), "TensorRT inference %s for /mode=%d",
      (!respect_mode_topic_ || current_mode_ == mode_required_) ? "active" : "paused",
      current_mode_);
  }

  bool inference_due()
  {
    if (respect_mode_topic_ && current_mode_ != mode_required_) {
      return false;
    }

    const auto wall_now = std::chrono::steady_clock::now();
    if (max_inference_fps_ > 0.0 && last_inference_wall_time_ != SteadyTimePoint{}) {
      const double elapsed =
        std::chrono::duration<double>(wall_now - last_inference_wall_time_).count();
      if (elapsed < 1.0 / max_inference_fps_) {
        return false;
      }
    }
    last_inference_wall_time_ = wall_now;
    return true;
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!ready_ || !inference_due()) {
      return;
    }

    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    } catch (const cv_bridge::Exception & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "cv_bridge conversion failed: %s", ex.what());
      return;
    }

    TensorBuffer & input = buffers_[static_cast<size_t>(input_buffer_index_)];
    const LetterboxInfo info = preprocess(cv_ptr->image, input);
    if (!infer()) {
      publish_state("infer_failed");
      return;
    }

    const TensorBuffer * output = primary_output();
    if (output == nullptr) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "No TensorRT output tensor");
      return;
    }

    const std::vector<Detection> persons = parse_yolo_output(*output, info);
    std::vector<PersonFeature> features = extract_features(cv_ptr->image, persons);
    update_depth_estimates(features, cv_ptr->image.cols, cv_ptr->image.rows, msg->header.stamp);
    update_tracks(features, cv_ptr->image.cols, cv_ptr->image.rows);
    update_height_estimates(features);
    handle_lock_command(features, cv_ptr->image.cols, cv_ptr->image.rows);
    publish_detections(msg->header, features);
    publish_locked_target(msg->header, features);
    publish_features(msg->header, features, cv_ptr->image.cols, cv_ptr->image.rows);
    publish_annotated_image(msg->header, cv_ptr->image, features);

    frame_count_++;
    const auto now = this->now();
    const double elapsed = (now - last_fps_time_).seconds();
    const double fps_for_log =
      elapsed > 1e-6 ? static_cast<double>(frame_count_) / elapsed : 0.0;
    if (elapsed >= 1.0) {
      frame_count_ = 0;
      last_fps_time_ = now;
      RCLCPP_INFO(
        get_logger(),
        "TensorRT torso demo fps=%.1f persons=%zu tracks=%zu locked=%d",
        fps_for_log, features.size(), tracks_.size(), locked_track_id_);
    }

    const double feature_log_elapsed = (now - last_feature_log_time_).seconds();
    if (feature_log_period_sec_ <= 0.0 || feature_log_elapsed >= feature_log_period_sec_) {
      last_feature_log_time_ = now;
      log_image_features(cv_ptr->image, features, fps_for_log);
    }
  }

  void lock_command_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    pending_lock_command_ = msg->data;
    has_pending_lock_command_ = true;
  }

  void publish_state(const std::string & state)
  {
    std_msgs::msg::String msg;
    msg.data = state;
    state_pub_->publish(msg);
  }

  using SteadyTimePoint = std::chrono::steady_clock::time_point;

  std::string engine_path_;
  std::string image_topic_;
  std::string annotated_topic_;
  std::string detections_topic_;
  std::string state_topic_;
  std::string features_topic_;
  std::string locked_topic_;
  std::string lock_command_topic_;
  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string bodylist_topic_;
  std::string bodyposture_topic_;
  std::string input_name_;
  std::string pending_lock_command_;

  int input_width_{640};
  int input_height_{640};
  int person_class_id_{0};
  int max_detections_{20};
  int input_buffer_index_{-1};
  int frame_count_{0};
  int next_track_id_{1};
  int locked_track_id_{-1};
  int locked_body_id_{-1};
  int skeleton_lock_status_{0};
  int track_max_missed_{12};
  int max_logged_targets_{6};
  int mode_required_{2};
  int current_mode_{1};
  int opencv_num_threads_{0};
  double conf_threshold_{0.45};
  double nms_iou_threshold_{0.45};
  double min_person_area_px_{600.0};
  double torso_side_crop_ratio_{0.18};
  double torso_top_ratio_{0.22};
  double torso_bottom_ratio_{0.68};
  double upper_color_side_crop_ratio_{0.28};
  double upper_color_band_top_ratio_{0.00};
  double upper_color_band_bottom_ratio_{0.50};
  double upper_color_inner_top_ratio_{0.34};
  double upper_color_inner_bottom_ratio_{0.84};
  double lower_color_side_crop_ratio_{0.26};
  double lower_color_band_top_ratio_{0.50};
  double lower_color_band_bottom_ratio_{1.00};
  double lower_color_inner_top_ratio_{0.08};
  double lower_color_inner_bottom_ratio_{0.44};
  double feature_log_period_sec_{1.0};
  double color_saturation_min_{22.0};
  double color_value_min_{35.0};
  double height_smoothing_alpha_{0.35};
  double camera_fy_px_{0.0};
  double depth_max_age_sec_{0.5};
  double depth_unit_scale_{0.001};
  double min_valid_depth_m_{0.20};
  double max_valid_depth_m_{8.0};
  double height_reference_m_{0.0};
  double height_reference_px_{0.0};
  double track_match_threshold_{0.82};
  double track_iou_weight_{0.35};
  double track_appearance_weight_{0.35};
  double track_size_weight_{0.15};
  double track_center_weight_{0.15};
  double skeleton_max_age_sec_{0.5};
  double skeleton_base_width_{640.0};
  double skeleton_base_height_{480.0};
  double max_inference_fps_{0.0};
  double max_annotated_fps_{0.0};
  bool publish_annotated_{true};
  bool annotated_only_if_subscribed_{false};
  bool log_image_features_{true};
  bool enable_depth_height_{true};
  bool auto_lock_center_{false};
  bool overlay_skeleton_{true};
  bool respect_mode_topic_{false};
  bool has_pending_lock_command_{false};
  bool has_bodylist_{false};
  bool ready_{false};
  SteadyTimePoint last_inference_wall_time_{};
  SteadyTimePoint last_annotated_wall_time_{};

  TrtLogger trt_logger_;
  TrtUniquePtr<nvinfer1::IRuntime> runtime_{nullptr};
  TrtUniquePtr<nvinfer1::ICudaEngine> engine_{nullptr};
  TrtUniquePtr<nvinfer1::IExecutionContext> context_{nullptr};
  nvinfer1::Dims input_dims_{};
  std::vector<TensorBuffer> buffers_;
  std::vector<TrackState> tracks_;
  cudaStream_t stream_{nullptr};
  sensor_msgs::msg::Image::SharedPtr latest_depth_msg_;
  bodyreader_msg::msg::Bodylist latest_bodylist_;
  rclcpp::Time last_fps_time_{this->now()};
  rclcpp::Time last_feature_log_time_{this->now()};
  rclcpp::Time last_bodylist_time_{this->now()};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr lock_command_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr mode_sub_;
  rclcpp::Subscription<bodyreader_msg::msg::Bodylist>::SharedPtr bodylist_sub_;
  rclcpp::Subscription<bodyreader_msg::msg::Bodyposture>::SharedPtr bodyposture_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_pub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detections_pub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr locked_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr features_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  auto node = std::make_shared<TorsoTrtDemo>();
  if (!g_shutdown_requested && rclcpp::ok()) {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (!g_shutdown_requested && rclcpp::ok()) {
      executor.spin_once(std::chrono::milliseconds(100));
    }
    executor.remove_node(node);
  }

  node.reset();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
