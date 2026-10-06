/**
 * @file follower_no_avoider.cpp
 * @brief 人体跟随 PD 控制节点 — 无避障版本
 *
 * 与 follower.cpp 功能完全一致，唯一区别：给无避障链路预留安全中间层，发布到 /cmd_vel_raw。
 * 适用场景：无激光雷达 / 不需要 VFH 避障 / 纯跟随测试。
 *
 * 架构（无避障）：
 *   body_posture → follower_no_avoider (PD控制) → /cmd_vel_raw → collision_guard → /cmd_vel
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <bodyreader_msg/msg/bodyposture.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int8.hpp>
#include <cmath>      // fabs

using namespace std;

rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_Pub;

float target_x_angle = 0;
float target_distance = 2000;
float x_p = 0;
float x_d = 0;
float z_p = 0;
float z_d = 0;
int mode  = 2;   //1:sleep  2:follow

void bodyposture_Callback(bodyreader_msg::msg::Bodyposture msg)
{
	if (mode != 2) return;

	// ============================================================
	// 稳定状态变量 (static)
	// ============================================================
	static float filtered_x_angle = 0.0f;     // 低通滤波后的角度
	static float last_error_x      = 0.0f;     // 上一帧角度误差 (用于D项)
	static float last_error_d      = 0.0f;     // 上一帧距离误差 (用于D项)
	static int   lost_frames       = 0;        // 连续丢帧计数
	static bool  filter_init       = false;    // 滤波器是否已初始化

	// ============================================================
	// 1. 丢帧检测: 防止 D 项突变 + 停止保护
	// ============================================================
	if (msg.centerofmass_x == 0 && msg.centerofmass_y == 0 && msg.centerofmass_z == 0)
	{
		lost_frames++;
		// 连续丢失 > 15 帧 (约 0.5s@30Hz): 确认为彻底丢失
		if (lost_frames > 15)
		{
			geometry_msgs::msg::Twist stop;
			cmd_vel_Pub->publish(stop);
			filter_init   = false;
			last_error_x  = 0.0f;
			last_error_d  = 0.0f;
		}
		// ★ 关键: 丢失期间不更新 last_error, 避免下一帧 D 项突变
		return;
	}
	lost_frames = 0;

	// ============================================================
	// 2. 计算原始测量值
	// ============================================================
	float raw_x_angle = msg.centerofmass_x / msg.centerofmass_z;
	float distance    = msg.centerofmass_z;

	// ============================================================
	// 3. 低通滤波: 抑制骨架跟踪 jitter (α=0.35)
	//    filtered = α·raw + (1-α)·filtered_prev
	// ============================================================
	const float alpha = 0.35f;
	if (!filter_init)
	{
		filtered_x_angle = raw_x_angle;
		filter_init      = true;
	}
	else
	{
		filtered_x_angle = alpha * raw_x_angle + (1.0f - alpha) * filtered_x_angle;
	}

	// ============================================================
	// 4. 误差计算 + 误差死区 (抑制骨架噪声)
	// ============================================================
	float error_x_angle  = filtered_x_angle - target_x_angle;
	float error_distance = distance - target_distance;

	// 角度误差死区: < 0.025rad (≈1.4°) 视为无偏,清空误差
	// 这过滤掉了人体站立时的自然微摆和骨架抖动
	if (fabs(error_x_angle) < 0.025f)
		error_x_angle = 0.0f;

	// 距离误差死区: ±80mm
	if (fabs(error_distance) < 80.0f)
		error_distance = 0.0f;

	// ============================================================
	// 5. PD 控制
	//    angular = error_angle·Kp + Δerror_angle·Kd
	//    linear  = error_dist·Kp/1000 + Δerror_dist·Kd/1000
	// ============================================================
	geometry_msgs::msg::Twist cmd_vel_msg;

	cmd_vel_msg.linear.x  = error_distance * x_p  / 1000.0f
	                      + (error_distance - last_error_d) * x_d / 1000.0f;
	cmd_vel_msg.angular.z = error_x_angle * z_p
	                      + (error_x_angle - last_error_x) * z_d;

	// 输出死区 (二次保护)
	if (fabs(cmd_vel_msg.linear.x)  < 0.02f) cmd_vel_msg.linear.x  = 0.0f;
	if (fabs(cmd_vel_msg.angular.z) < 0.02f) cmd_vel_msg.angular.z = 0.0f;

	// 限幅 (C++14 兼容)
	if (cmd_vel_msg.linear.x > 0.5f)  cmd_vel_msg.linear.x = 0.5f;
	if (cmd_vel_msg.linear.x < -0.5f) cmd_vel_msg.linear.x = -0.5f;
	if (cmd_vel_msg.angular.z > 1.0f)  cmd_vel_msg.angular.z = 1.0f;
	if (cmd_vel_msg.angular.z < -1.0f) cmd_vel_msg.angular.z = -1.0f;

	cmd_vel_Pub->publish(cmd_vel_msg);

	// ============================================================
	// 6. 更新上一帧误差 (仅在有效帧更新)
	// ============================================================
	last_error_x = error_x_angle;
	last_error_d = error_distance;
}


void mode_Callback(std_msgs::msg::Int8 msg)
{
	mode = msg.data;
}


int main(int argc, char *argv[])
{

	rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("follower_no_avoider");

	// 无避障链路也先发 /cmd_vel_raw，再由 collision_guard 做后退碰撞急停。
	cmd_vel_Pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_raw", 1);
	auto bodyposture_sub = node->create_subscription<bodyreader_msg::msg::Bodyposture>("/body_posture", 1, bodyposture_Callback);
	auto mode_sub = node->create_subscription<std_msgs::msg::Int8>("/mode", 1, mode_Callback);

	node->declare_parameter<float>("bodyfollow_x_p", 0.01);
  	node->declare_parameter<float>("bodyfollow_x_d", 0.01);
    node->declare_parameter<float>("bodyfollow_z_p", 0.01);
  	node->declare_parameter<float>("bodyfollow_z_d", 0.01);
  	node->declare_parameter<int>("mode", 2);
  	node->get_parameter("bodyfollow_x_p", x_p);
  	node->get_parameter("bodyfollow_x_d", x_d);
  	node->get_parameter("bodyfollow_z_p", z_p);
  	node->get_parameter("bodyfollow_z_d", z_d);
  	node->get_parameter("mode", mode);

	double rate = 10;    //频率10Hz
	rclcpp::Rate loopRate(rate);

/*	printf("bodyfollow_x_p = %f\n", x_p);
	printf("bodyfollow_x_d = %f\n", x_d);
	printf("bodyfollow_z_p = %f\n", z_p);
	printf("bodyfollow_z_d = %f\n", z_d);*/
	
	while(rclcpp::ok())
	{

		rclcpp::spin_some(node->get_node_base_interface());
		loopRate.sleep();

	}

}
