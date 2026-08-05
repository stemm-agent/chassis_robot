// This file is part of the Orbbec Astra SDK [https://orbbec3d.com]
// Copyright (c) 2015-2017 Orbbec 3D
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// 
// Be excellent to each other.
#include <astra/capi/astra.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <thread>
#include <unordered_map>
#include <key_handler.h>
#include <rclcpp/rclcpp.hpp>
#include "bodyreader_msg/msg/bodylist.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/int8.hpp"
#include <vector>

rclcpp::Publisher<bodyreader_msg::msg::Bodylist>::SharedPtr bodylist_Pub;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_Pub;

namespace
{
constexpr int kJointCount = 19;
constexpr int kJointStatusTracked = 2;
constexpr int kMidSpine = 8;
constexpr int kBaseSpine = 9;
constexpr int kLeftShoulder = 2;
constexpr int kRightShoulder = 5;
constexpr int kNeck = 18;
constexpr int kFrameWaitTimeoutMs = 50;
constexpr int kIdleSleepMs = 20;

int g_min_tracked_joints = 8;
int g_min_tracked_core_joints = 3;
float g_min_body_distance_mm = 800.0f;
float g_max_body_distance_mm = 4000.0f;
float g_max_abs_center_ratio = 0.35f;
int g_min_stable_frames = 3;
bool g_body_filter_diagnostics_enabled = true;
double g_body_filter_diagnostics_period_s = 2.0;
int g_mode_required = 2;

std::atomic<int> g_current_mode{-1};
std::atomic<bool> g_restart_body_stream_requested{false};

std::unordered_map<int, int> g_valid_body_streaks;

struct BodyFilterEvaluation
{
    float center_z_mm = 0.0f;
    float center_ratio = std::numeric_limits<float>::infinity();
    int tracked_joint_count = 0;
    int tracked_core_joint_count = 0;
    bool distance_ok = false;
    bool center_ok = false;
    bool joints_ok = false;
    bool core_joints_ok = false;

    bool accepted() const
    {
        return distance_ok && center_ok && joints_ok && core_joints_ok;
    }
};

struct BodyFilterDiagnostics
{
    uint64_t frames = 0;
    uint64_t raw_bodies = 0;
    uint64_t accepted_bodies = 0;
    uint64_t no_raw_body_frames = 0;
    uint64_t consecutive_no_raw_body_frames = 0;
    uint64_t body_list_errors = 0;
    uint64_t frame_wait_failures = 0;
    uint64_t consecutive_frame_wait_failures = 0;
    int last_frame_wait_status = 0;
    uint64_t body_frame_errors = 0;
    int last_body_frame_status = 0;
    uint64_t rejected_distance = 0;
    uint64_t rejected_center = 0;
    uint64_t rejected_joints = 0;
    uint64_t rejected_core_joints = 0;
    uint64_t rejected_unstable = 0;
    BodyFilterEvaluation last_candidate;
    bool has_last_candidate = false;
    bool was_active = false;
    std::chrono::steady_clock::time_point window_start =
        std::chrono::steady_clock::now();
};

BodyFilterDiagnostics g_body_filter_diagnostics;

bool is_core_joint(int joint_index)
{
    return joint_index == kNeck ||
           joint_index == kMidSpine ||
           joint_index == kBaseSpine ||
           joint_index == kLeftShoulder ||
           joint_index == kRightShoulder;
}

BodyFilterEvaluation evaluate_body_filter(const astra_body_t& body)
{
    BodyFilterEvaluation result;
    const float center_z = body.centerOfMass.z;
    result.center_z_mm = center_z;
    result.distance_ok =
        center_z > 1.0f &&
        center_z >= g_min_body_distance_mm &&
        center_z <= g_max_body_distance_mm;
    if (center_z > 1.0f)
    {
        result.center_ratio = std::fabs(body.centerOfMass.x / center_z);
    }
    result.center_ok = result.center_ratio <= g_max_abs_center_ratio;

    for (int joint_index = 0; joint_index < kJointCount; ++joint_index)
    {
        const astra_joint_t& joint = body.joints[joint_index];
        if (joint.status != kJointStatusTracked)
        {
            continue;
        }

        ++result.tracked_joint_count;
        if (is_core_joint(joint_index))
        {
            ++result.tracked_core_joint_count;
        }
    }

    result.joints_ok = result.tracked_joint_count >= g_min_tracked_joints;
    result.core_joints_ok =
        result.tracked_core_joint_count >= g_min_tracked_core_joints;
    return result;
}

void reset_body_filter_diagnostics(bool active)
{
    g_body_filter_diagnostics = BodyFilterDiagnostics{};
    g_body_filter_diagnostics.was_active = active;
}

void maybe_log_body_filter_diagnostics()
{
    if (!g_body_filter_diagnostics_enabled)
    {
        return;
    }

    const bool active =
        g_current_mode.load(std::memory_order_relaxed) == g_mode_required;
    if (!active)
    {
        if (g_body_filter_diagnostics.was_active)
        {
            reset_body_filter_diagnostics(false);
        }
        return;
    }

    if (!g_body_filter_diagnostics.was_active)
    {
        reset_body_filter_diagnostics(true);
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    const double elapsed_s = std::chrono::duration<double>(
        now - g_body_filter_diagnostics.window_start).count();
    if (elapsed_s < g_body_filter_diagnostics_period_s)
    {
        return;
    }

    const auto& d = g_body_filter_diagnostics;
    if (d.frame_wait_failures > 0 || d.body_frame_errors > 0)
    {
        RCLCPP_WARN(
            rclcpp::get_logger("body_main"),
            "ASTRA_DIAG %.1fs frame_wait_failures=%llu consecutive_wait_failures=%llu "
            "last_wait_status=%d body_frame_errors=%llu last_body_frame_status=%d",
            elapsed_s,
            static_cast<unsigned long long>(d.frame_wait_failures),
            static_cast<unsigned long long>(d.consecutive_frame_wait_failures),
            d.last_frame_wait_status,
            static_cast<unsigned long long>(d.body_frame_errors),
            d.last_body_frame_status);
    }
    if (d.accepted_bodies > 0)
    {
        RCLCPP_INFO(
            rclcpp::get_logger("body_main"),
            "BODY_FILTER %.1fs frames=%llu raw=%llu accepted=%llu "
            "rejected[distance=%llu center=%llu joints=%llu core=%llu unstable=%llu]",
            elapsed_s,
            static_cast<unsigned long long>(d.frames),
            static_cast<unsigned long long>(d.raw_bodies),
            static_cast<unsigned long long>(d.accepted_bodies),
            static_cast<unsigned long long>(d.rejected_distance),
            static_cast<unsigned long long>(d.rejected_center),
            static_cast<unsigned long long>(d.rejected_joints),
            static_cast<unsigned long long>(d.rejected_core_joints),
            static_cast<unsigned long long>(d.rejected_unstable));
    }
    else if (d.has_last_candidate)
    {
        RCLCPP_WARN(
            rclcpp::get_logger("body_main"),
            "BODY_FILTER %.1fs frames=%llu raw=%llu accepted=%llu no_raw_frames=%llu "
            "list_errors=%llu rejected[distance=%llu center=%llu joints=%llu "
            "core=%llu unstable=%llu] last[z=%.0fmm ratio=%.3f joints=%d core=%d] "
            "limits[z=%.0f..%.0fmm ratio<=%.3f joints>=%d core>=%d stable>=%d]",
            elapsed_s,
            static_cast<unsigned long long>(d.frames),
            static_cast<unsigned long long>(d.raw_bodies),
            static_cast<unsigned long long>(d.accepted_bodies),
            static_cast<unsigned long long>(d.no_raw_body_frames),
            static_cast<unsigned long long>(d.body_list_errors),
            static_cast<unsigned long long>(d.rejected_distance),
            static_cast<unsigned long long>(d.rejected_center),
            static_cast<unsigned long long>(d.rejected_joints),
            static_cast<unsigned long long>(d.rejected_core_joints),
            static_cast<unsigned long long>(d.rejected_unstable),
            d.last_candidate.center_z_mm,
            d.last_candidate.center_ratio,
            d.last_candidate.tracked_joint_count,
            d.last_candidate.tracked_core_joint_count,
            g_min_body_distance_mm,
            g_max_body_distance_mm,
            g_max_abs_center_ratio,
            g_min_tracked_joints,
            g_min_tracked_core_joints,
            g_min_stable_frames);
    }
    else
    {
        RCLCPP_WARN(
            rclcpp::get_logger("body_main"),
            "BODY_FILTER %.1fs frames=%llu raw=0 accepted=0 no_raw_frames=%llu "
            "consecutive_no_raw_frames=%llu list_errors=%llu reason=no_astra_body",
            elapsed_s,
            static_cast<unsigned long long>(d.frames),
            static_cast<unsigned long long>(d.no_raw_body_frames),
            static_cast<unsigned long long>(d.consecutive_no_raw_body_frames),
            static_cast<unsigned long long>(d.body_list_errors));
    }

    reset_body_filter_diagnostics(true);
}
}

void print_color(astra_colorframe_t colorFrame)
{
    astra_image_metadata_t metadata;
    astra_rgb_pixel_t* color_data = nullptr;
    uint32_t color_byte_length = 0;

    if (astra_colorframe_get_data_rgb_ptr(
            colorFrame, &color_data, &color_byte_length) != ASTRA_STATUS_SUCCESS ||
        astra_colorframe_get_metadata(colorFrame, &metadata) != ASTRA_STATUS_SUCCESS ||
        color_data == nullptr)
    {
        return;
    }

    sensor_msgs::msg::Image image_msg;
    image_msg.height = metadata.height / 2;
    image_msg.width = metadata.width / 2;
    image_msg.encoding = "rgb8";
    image_msg.is_bigendian = 0;
    image_msg.step = image_msg.width * 3;
    image_msg.header.frame_id = "cam";
    image_msg.data.resize(
        static_cast<size_t>(image_msg.step) * image_msg.height);

    for (uint32_t output_y = 0; output_y < image_msg.height; ++output_y)
    {
        const uint32_t source_y = output_y * 2;
        for (uint32_t output_x = 0; output_x < image_msg.width; ++output_x)
        {
            const uint32_t source_x = output_x * 2;
            const auto& pixel =
                color_data[source_y * metadata.width + source_x];
            const size_t output_index =
                (static_cast<size_t>(output_y) * image_msg.width + output_x) * 3;
            image_msg.data[output_index] = pixel.r;
            image_msg.data[output_index + 1] = pixel.g;
            image_msg.data[output_index + 2] = pixel.b;
        }
    }

    image_Pub->publish(image_msg);
}


void output_bodyframe_info(astra_bodyframe_t bodyFrame)
{
    astra_bodyframe_info_t info;

    const astra_status_t rc = astra_bodyframe_info(bodyFrame, &info);
    if (rc != ASTRA_STATUS_SUCCESS)
    {
        return;
    }
}


void output_joint(const int32_t bodyId, const astra_joint_t* joint)
{
    // jointType is one of ASTRA_JOINT_* which exists for each joint type
    const astra_joint_type_t jointType = joint->type;

    // jointStatus is one of:
    // ASTRA_JOINT_STATUS_NOT_TRACKED = 0,
    // ASTRA_JOINT_STATUS_LOW_CONFIDENCE = 1,
    // ASTRA_JOINT_STATUS_TRACKED = 2,
    const astra_joint_status_t jointStatus = joint->status;

    const astra_vector3f_t* worldPos = &joint->worldPosition;

    // depthPosition is in pixels from 0 to width and 0 to height
    // where width and height are member of astra_bodyframe_info_t
    // which is obtained from astra_bodyframe_info().
    const astra_vector2f_t* depthPos = &joint->depthPosition;

    (void)bodyId;
    (void)jointType;
    (void)jointStatus;
    (void)worldPos;
    (void)depthPos;
}


void output_bodies(astra_bodyframe_t bodyFrame)
{
    astra_body_list_t bodyList;
    bodyreader_msg::msg::Bodylist bodylist_msg;
    const astra_status_t rc = astra_bodyframe_body_list(bodyFrame, &bodyList);
    if (rc != ASTRA_STATUS_SUCCESS)
    {
        ++g_body_filter_diagnostics.body_list_errors;
        maybe_log_body_filter_diagnostics();
        return;
    }

    ++g_body_filter_diagnostics.frames;
    g_body_filter_diagnostics.raw_bodies +=
        static_cast<uint64_t>(std::max(0, bodyList.count));
    if (bodyList.count <= 0)
    {
        ++g_body_filter_diagnostics.no_raw_body_frames;
        ++g_body_filter_diagnostics.consecutive_no_raw_body_frames;
    }
    else
    {
        g_body_filter_diagnostics.consecutive_no_raw_body_frames = 0;
    }

    std::unordered_map<int, int> next_valid_body_streaks;
    int filtered_body_count = 0;
    for (int i = 0; i < bodyList.count && filtered_body_count < 6; ++i)
    {   
        astra_body_t* body = &bodyList.bodies[i];
        const BodyFilterEvaluation evaluation = evaluate_body_filter(*body);
        g_body_filter_diagnostics.last_candidate = evaluation;
        g_body_filter_diagnostics.has_last_candidate = true;
        if (!evaluation.distance_ok)
        {
            ++g_body_filter_diagnostics.rejected_distance;
        }
        if (!evaluation.center_ok)
        {
            ++g_body_filter_diagnostics.rejected_center;
        }
        if (!evaluation.joints_ok)
        {
            ++g_body_filter_diagnostics.rejected_joints;
        }
        if (!evaluation.core_joints_ok)
        {
            ++g_body_filter_diagnostics.rejected_core_joints;
        }
        if (!evaluation.accepted())
        {
            continue;
        }

        const int stable_frames = g_valid_body_streaks[body->id] + 1;
        next_valid_body_streaks[body->id] = stable_frames;
        if (stable_frames < g_min_stable_frames)
        {
            ++g_body_filter_diagnostics.rejected_unstable;
            continue;
        }

        // Pixels in the body mask with the same value as bodyId are
        // from the same body.
        bodylist_msg.bodies[filtered_body_count].bodyid = body->id;
        //printf("+++++++++++++++bodyId = %d\n", bodyId);
        const astra_vector3f_t* centerofmass = &body->centerOfMass;
        bodylist_msg.bodies[filtered_body_count].centerofmass.x = centerofmass->x;
        bodylist_msg.bodies[filtered_body_count].centerofmass.y = centerofmass->y;
        bodylist_msg.bodies[filtered_body_count].centerofmass.z = centerofmass->z;
        for (int j = 0; j < kJointCount; ++j)
        { 
          const astra_joint_t* joint = &body->joints[j];
          bodylist_msg.bodies[filtered_body_count].joints[j].type = joint->type ;
          bodylist_msg.bodies[filtered_body_count].joints[j].status = joint->status ;
          const astra_vector2f_t* depthPosition = &joint->depthPosition;
          bodylist_msg.bodies[filtered_body_count].joints[j].depthposition.x = depthPosition->x ;
          bodylist_msg.bodies[filtered_body_count].joints[j].depthposition.y = depthPosition->y ;
          const astra_vector3f_t* worldPosition = &joint->worldPosition;
          bodylist_msg.bodies[filtered_body_count].joints[j].worldposition.x = worldPosition->x ;
          bodylist_msg.bodies[filtered_body_count].joints[j].worldposition.y = worldPosition->y ;
          bodylist_msg.bodies[filtered_body_count].joints[j].worldposition.z = worldPosition->z ;
        }

        ++filtered_body_count;
        ++g_body_filter_diagnostics.accepted_bodies;

    }

    g_valid_body_streaks = std::move(next_valid_body_streaks);
    bodylist_msg.count = filtered_body_count;
    bodylist_Pub->publish(bodylist_msg);
    maybe_log_body_filter_diagnostics();
}

void output_bodyframe(astra_bodyframe_t bodyFrame)
{
    //output_bodyframe_info(bodyFrame);
    output_bodies(bodyFrame);
}

int main(int argc, char* argv[])
{
    bool body_stream;
    bool rgb_stream;
    bool mode_gated_body_stream;
    int mode_required;
    double max_processing_rate_hz;

    // Keep stderr available so ROS diagnostics reach the service journal.
    (void)freopen("/dev/null", "w", stdout);

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("main");

    node->declare_parameter<bool>("rgb_stream", false);
    node->declare_parameter<bool>("body_stream", true);
    node->declare_parameter<bool>("mode_gated_body_stream", false);
    node->declare_parameter<int>("mode_required", 2);
    node->declare_parameter<double>("max_processing_rate_hz", 0.0);
    node->declare_parameter<int>("min_tracked_joints", 8);
    node->declare_parameter<int>("min_tracked_core_joints", 3);
    node->declare_parameter<double>("min_body_distance_mm", 800.0);
    node->declare_parameter<double>("max_body_distance_mm", 4000.0);
    node->declare_parameter<double>("max_abs_center_ratio", 0.35);
    node->declare_parameter<int>("min_stable_frames", 3);
    node->declare_parameter<bool>("body_filter_diagnostics_enabled", true);
    node->declare_parameter<double>("body_filter_diagnostics_period_s", 2.0);
    node->get_parameter("rgb_stream", rgb_stream);
    node->get_parameter("body_stream", body_stream);
    node->get_parameter("mode_gated_body_stream", mode_gated_body_stream);
    node->get_parameter("mode_required", mode_required);
    node->get_parameter("max_processing_rate_hz", max_processing_rate_hz);
    node->get_parameter("min_tracked_joints", g_min_tracked_joints);
    node->get_parameter("min_tracked_core_joints", g_min_tracked_core_joints);

    double min_body_distance_mm = g_min_body_distance_mm;
    double max_body_distance_mm = g_max_body_distance_mm;
    double max_abs_center_ratio = g_max_abs_center_ratio;
    node->get_parameter("min_body_distance_mm", min_body_distance_mm);
    node->get_parameter("max_body_distance_mm", max_body_distance_mm);
    node->get_parameter("max_abs_center_ratio", max_abs_center_ratio);
    node->get_parameter("min_stable_frames", g_min_stable_frames);
    node->get_parameter(
        "body_filter_diagnostics_enabled", g_body_filter_diagnostics_enabled);
    node->get_parameter(
        "body_filter_diagnostics_period_s", g_body_filter_diagnostics_period_s);

    g_min_body_distance_mm = static_cast<float>(min_body_distance_mm);
    g_max_body_distance_mm = static_cast<float>(max_body_distance_mm);
    g_max_abs_center_ratio = static_cast<float>(max_abs_center_ratio);

    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr mode_sub;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr body_stream_restart_sub;
    g_mode_required = mode_required;
    g_body_filter_diagnostics_period_s =
        std::max(0.5, g_body_filter_diagnostics_period_s);
    mode_sub = node->create_subscription<std_msgs::msg::Int8>(
        "/mode",
        rclcpp::QoS(1).best_effort(),
        [](const std_msgs::msg::Int8::SharedPtr msg)
        {
            g_current_mode.store(msg->data, std::memory_order_relaxed);
        });
    body_stream_restart_sub = node->create_subscription<std_msgs::msg::Empty>(
        "/body_main/restart_body_stream",
        1,
        [](const std_msgs::msg::Empty::SharedPtr)
        {
            g_restart_body_stream_requested.store(true, std::memory_order_relaxed);
        });

    set_key_handler();

    astra_initialize();

    const char* licenseString = "<INSERT LICENSE KEY HERE>";
    

    orbbec_body_tracking_set_license(licenseString);

    astra_streamsetconnection_t sensor;

    astra_streamset_open("device/default", &sensor);

    astra_reader_t reader;
    astra_reader_create(sensor, &reader);

    astra_bodystream_t bodyStream;
    astra_reader_get_bodystream(reader, &bodyStream);

    astra_colorstream_t colorStream;
    astra_reader_get_colorstream(reader, &colorStream);

    bool body_stream_running = false;
    if (body_stream)
    {
        bodylist_Pub =
            node->create_publisher<bodyreader_msg::msg::Bodylist>("/bodylist", 1);
        if (!mode_gated_body_stream)
        {
            body_stream_running =
                astra_stream_start(bodyStream) == ASTRA_STATUS_SUCCESS;
        }
    }

    if (rgb_stream)
    {
        image_Pub = node->create_publisher<sensor_msgs::msg::Image>("/image_raw", 1);
        astra_stream_start(colorStream);
    }

    using SteadyClock = std::chrono::steady_clock;
    const bool rate_limited = max_processing_rate_hz > 0.0;
    const auto processing_period = rate_limited
        ? std::chrono::duration_cast<SteadyClock::duration>(
              std::chrono::duration<double>(1.0 / max_processing_rate_hz))
        : SteadyClock::duration::zero();
    auto next_processing_time = SteadyClock::now();

    do
    {
        rclcpp::spin_some(node);

        if (body_stream &&
            g_restart_body_stream_requested.exchange(false, std::memory_order_relaxed))
        {
            g_valid_body_streaks.clear();
            bodyreader_msg::msg::Bodylist empty_bodylist;
            empty_bodylist.count = 0;
            bodylist_Pub->publish(empty_bodylist);

            RCLCPP_WARN(
                node->get_logger(),
                "Astra watchdog recycling body_main for full SDK reinitialization");
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            break;
        }

        if (mode_gated_body_stream && body_stream)
        {
            const int requested_mode =
                g_current_mode.load(std::memory_order_relaxed);
            if (requested_mode >= 0)
            {
                const bool should_run = requested_mode == mode_required;
                if (should_run && !body_stream_running)
                {
                    g_valid_body_streaks.clear();
                    body_stream_running =
                        astra_stream_start(bodyStream) == ASTRA_STATUS_SUCCESS;
                }
                else if (!should_run && body_stream_running)
                {
                    astra_stream_stop(bodyStream);
                    body_stream_running = false;
                    g_valid_body_streaks.clear();

                    bodyreader_msg::msg::Bodylist empty_bodylist;
                    empty_bodylist.count = 0;
                    bodylist_Pub->publish(empty_bodylist);
                }
            }
        }

        if (!body_stream_running && !rgb_stream)
        {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(kIdleSleepMs));
            continue;
        }

        astra_update();

        astra_reader_frame_t frame;
        const astra_status_t rc =
            astra_reader_open_frame(reader, kFrameWaitTimeoutMs, &frame);

        if (rc == ASTRA_STATUS_SUCCESS)
        {
            g_body_filter_diagnostics.consecutive_frame_wait_failures = 0;
            const auto now = SteadyClock::now();
            const bool process_frame =
                !rate_limited || now >= next_processing_time;

            if (process_frame)
            {
                if (body_stream_running)
                {
                    astra_bodyframe_t bodyFrame;
                    const astra_status_t body_frame_rc =
                        astra_frame_get_bodyframe(frame, &bodyFrame);
                    if (body_frame_rc == ASTRA_STATUS_SUCCESS)
                    {
                        output_bodyframe(bodyFrame);
                    }
                    else
                    {
                        ++g_body_filter_diagnostics.body_frame_errors;
                        g_body_filter_diagnostics.last_body_frame_status =
                            static_cast<int>(body_frame_rc);
                    }
                }

                if (rgb_stream)
                {
                    astra_colorframe_t colorFrame;
                    astra_frame_get_colorframe(frame, &colorFrame);
                    print_color(colorFrame);
                }

                if (rate_limited)
                {
                    do
                    {
                        next_processing_time += processing_period;
                    } while (next_processing_time <= now);
                }
            }

            astra_reader_close_frame(&frame);
        }
        else
        {
            ++g_body_filter_diagnostics.frame_wait_failures;
            ++g_body_filter_diagnostics.consecutive_frame_wait_failures;
            g_body_filter_diagnostics.last_frame_wait_status =
                static_cast<int>(rc);
        }

        maybe_log_body_filter_diagnostics();

    } while (shouldContinue && rclcpp::ok());

    if (body_stream_running)
    {
        astra_stream_stop(bodyStream);
    }
    if (rgb_stream)
    {
        astra_stream_stop(colorStream);
    }

    astra_reader_destroy(&reader);
    astra_streamset_close(&sensor);

    astra_terminate();

    bodylist_Pub.reset();
    image_Pub.reset();
    body_stream_restart_sub.reset();
    mode_sub.reset();
    node.reset();

    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }

    return 0;
}
