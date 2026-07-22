#include <rclcpp/rclcpp.hpp>
#include <array>
#include <initializer_list>
#include <cmath>
#include <bodyreader_msg/msg/bodyposture.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <bodyreader_msg/msg/bodylist.hpp>
#include <bodyreader_msg/msg/body.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int8.hpp>
#include <geometry_msgs/msg/twist.hpp>

#define HEAD 0
#define SHOULDER_SPINE 1
#define LEFT_SHOULDER 2
#define LEFT_ELBOW 3
#define LEFT_HAND 4
#define RIGHT_SHOULDER 5
#define RIGHT_ELBOW 6
#define RIGHT_HAND 7
#define MID_SPINE 8
#define BASE_SPINE 9
#define LEFT_HIP 10
#define LEFT_KNEE 11
#define LEFT_FOOT 12
#define RIGHT_HIP 13
#define RIGHT_KNEE 14
#define RIGHT_FOOT 15
#define LEFT_WRIST 16
#define RIGHT_WRIST 17
#define NECK 18
#define UNKNOWN 255

#define JOINT_STATUS_TRACKED 2
 
 
int lock_body_id = 0;
int lock_status = 0;		//0-nobody  1-no lock    2-locked
int last_lock_status = 0;
//int ii=0  ; 

bool open_switch = false;
bool require_akimbo_lock = true;
bool auto_lock_first_body = false;
bool body_locked_once = false;
int initial_mode = 1;

rclcpp::Publisher<bodyreader_msg::msg::Bodyposture>::SharedPtr bodyposture_Pub;
bodyreader_msg::msg::Bodyposture bodyposture_msg;

rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr mode_Pub;
std_msgs::msg::Int8 mode_msg;

rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_Pub;

int pose_min_joint_status = JOINT_STATUS_TRACKED;
float arm_horizontal_y_tol_mm = 100.0f;
float arm_horizontal_forearm_y_tol_mm = 150.0f;
float arm_horizontal_segment_min_mm = 200.0f;
float hand_raise_elbow_hand_min_mm = 130.0f;
float hand_raise_shoulder_elbow_min_mm = 80.0f;
float akimbo_hand_above_spine_min_mm = 50.0f;
float akimbo_shoulder_hand_x_tol_mm = 100.0f;
float foot_up_delta_mm = 120.0f;
float fall_knee_spine_min_mm = -10.0f;

bool joints_tracked(const bodyreader_msg::msg::Body& body, std::initializer_list<int> joints)
{
	for (int joint_index : joints)
	{
		if (body.joints[joint_index].status < pose_min_joint_status)
		{
			return false;
		}
	}

	return true;
}

void judge_pose(bodyreader_msg::msg::Body body)
{	
	if(joints_tracked(body, {LEFT_SHOULDER, LEFT_ELBOW, LEFT_HAND})
		&& std::fabs(body.joints[LEFT_SHOULDER].worldposition.y - body.joints[LEFT_ELBOW].worldposition.y) < arm_horizontal_y_tol_mm
		&& std::fabs(body.joints[LEFT_ELBOW].worldposition.y - body.joints[LEFT_HAND].worldposition.y) < arm_horizontal_forearm_y_tol_mm
		&& (body.joints[LEFT_SHOULDER].worldposition.x - body.joints[LEFT_ELBOW].worldposition.x) > arm_horizontal_segment_min_mm
		&& (body.joints[LEFT_ELBOW].worldposition.x - body.joints[LEFT_HAND].worldposition.x) > arm_horizontal_segment_min_mm)
    {
		if(body.bodyid == lock_body_id)
		{
			bodyposture_msg.left_arm_out = 1;
		}
	}

	if(joints_tracked(body, {RIGHT_SHOULDER, RIGHT_ELBOW, RIGHT_HAND})
		&& std::fabs(body.joints[RIGHT_SHOULDER].worldposition.y - body.joints[RIGHT_ELBOW].worldposition.y) < arm_horizontal_y_tol_mm
		&& std::fabs(body.joints[RIGHT_ELBOW].worldposition.y - body.joints[RIGHT_HAND].worldposition.y) < arm_horizontal_forearm_y_tol_mm
		&& (body.joints[RIGHT_ELBOW].worldposition.x - body.joints[RIGHT_SHOULDER].worldposition.x) > arm_horizontal_segment_min_mm
		&& (body.joints[RIGHT_HAND].worldposition.x - body.joints[RIGHT_ELBOW].worldposition.x) > arm_horizontal_segment_min_mm)
    {
		if(body.bodyid == lock_body_id)
		{
			bodyposture_msg.right_arm_out = 1;
		}
	}

	if(joints_tracked(body, {LEFT_SHOULDER, LEFT_ELBOW, LEFT_HAND})
	         && (body.joints[LEFT_HAND].worldposition.y - body.joints[LEFT_ELBOW].worldposition.y) > hand_raise_elbow_hand_min_mm
	         && (body.joints[LEFT_SHOULDER].worldposition.y - body.joints[LEFT_ELBOW].worldposition.y) > hand_raise_shoulder_elbow_min_mm)
    {
		if(body.bodyid == lock_body_id)
		{
			bodyposture_msg.left_hand_raised = 1;
		}
	}

	if(joints_tracked(body, {RIGHT_SHOULDER, RIGHT_ELBOW, RIGHT_HAND})
	         && (body.joints[RIGHT_HAND].worldposition.y - body.joints[RIGHT_ELBOW].worldposition.y) > hand_raise_elbow_hand_min_mm
	         && (body.joints[RIGHT_SHOULDER].worldposition.y - body.joints[RIGHT_ELBOW].worldposition.y) > hand_raise_shoulder_elbow_min_mm)
    {
		if(body.bodyid == lock_body_id)
		{
			bodyposture_msg.right_hand_raised = 1;
		}
	}

	if(joints_tracked(body, {LEFT_HAND, RIGHT_HAND, LEFT_SHOULDER, RIGHT_SHOULDER, BASE_SPINE})
		&& (body.joints[LEFT_HAND].worldposition.y - body.joints[BASE_SPINE].worldposition.y) > akimbo_hand_above_spine_min_mm
		&& (body.joints[RIGHT_HAND].worldposition.y - body.joints[BASE_SPINE].worldposition.y) > akimbo_hand_above_spine_min_mm
		&& std::fabs(body.joints[LEFT_SHOULDER].worldposition.x - body.joints[LEFT_HAND].worldposition.x) < akimbo_shoulder_hand_x_tol_mm
		//&& (body.joints[LEFT_HAND].worldposition.x - body.joints[LEFT_ELBOW].worldposition.x) > 100
		//&& (body.joints[RIGHT_SHOULDER].worldposition.x - body.joints[RIGHT_ELBOW].worldposition.x) < -100
		//&& (body.joints[RIGHT_HAND].worldposition.x - body.joints[RIGHT_ELBOW].worldposition.x) < -100
		&& std::fabs(body.joints[RIGHT_SHOULDER].worldposition.x - body.joints[RIGHT_HAND].worldposition.x) < akimbo_shoulder_hand_x_tol_mm
		&& (body.joints[RIGHT_SHOULDER].worldposition.y - body.joints[RIGHT_HAND].worldposition.y) > akimbo_hand_above_spine_min_mm
		&& (body.joints[LEFT_SHOULDER].worldposition.y - body.joints[LEFT_HAND].worldposition.y) > akimbo_hand_above_spine_min_mm
            )
    {
		if (require_akimbo_lock)
		{
			lock_body_id = body.bodyid;
			body_locked_once = true;
			bodyposture_msg.akimibo = 1;
		}
	}

	if(joints_tracked(body, {LEFT_FOOT, RIGHT_FOOT, BASE_SPINE})
		      && (body.joints[LEFT_FOOT].worldposition.y - body.joints[RIGHT_FOOT].worldposition.y) > foot_up_delta_mm  
              && (body.joints[BASE_SPINE].worldposition.y - body.joints[LEFT_FOOT].worldposition.y) > 200
              && (body.joints[BASE_SPINE].worldposition.y - body.joints[RIGHT_FOOT].worldposition.y) > 500)
      {
		if(body.bodyid == lock_body_id)
		{
			bodyposture_msg.left_foot_up = 1;
		}
	}

	if(joints_tracked(body, {LEFT_FOOT, RIGHT_FOOT, BASE_SPINE})
		      && (body.joints[RIGHT_FOOT].worldposition.y - body.joints[LEFT_FOOT].worldposition.y) > foot_up_delta_mm  
              && (body.joints[BASE_SPINE].worldposition.y - body.joints[LEFT_FOOT].worldposition.y) > 500
              && (body.joints[BASE_SPINE].worldposition.y - body.joints[RIGHT_FOOT].worldposition.y) > 200)
    {
		if(body.bodyid == lock_body_id)
		{
			bodyposture_msg.right_foot_up = 1;
		}
	}


	if(joints_tracked(body, {LEFT_KNEE, RIGHT_KNEE, BASE_SPINE})
		&& (body.joints[LEFT_KNEE].worldposition.y - body.joints[BASE_SPINE].worldposition.y) > fall_knee_spine_min_mm 
		&& (body.joints[RIGHT_KNEE].worldposition.y - body.joints[BASE_SPINE].worldposition.y) > -10
              )
    {
		if(body.bodyid == lock_body_id)
		{
			bodyposture_msg.fall = 1;
		}
	}


	if (open_switch){
		if(joints_tracked(body, {LEFT_HAND, RIGHT_HAND, BASE_SPINE})
			&& (body.joints[LEFT_HAND].worldposition.x - body.joints[BASE_SPINE].worldposition.x) > 0 
			&& (body.joints[BASE_SPINE].worldposition.x - body.joints[RIGHT_HAND].worldposition.x) > 0
	              ) 
	    {
			if(body.bodyid == lock_body_id)
			{
				
				if (mode_msg.data == 1) mode_msg.data = 2;
				else if (mode_msg.data == 2) mode_msg.data = 1;
				mode_Pub->publish(mode_msg);
				system("aplay -D plughw:CARD=Device,DEV=0 ~/wheeltec_ros2/src/bodyreader/audio/mode_switch.wav");
			}
		}
	}
}

void bodylist_Callback(bodyreader_msg::msg::Bodylist body_list)
{
	bodyposture_msg.bodyid = 0;
	bodyposture_msg.centerofmass_x = 0;
    bodyposture_msg.centerofmass_y = 0;
    bodyposture_msg.centerofmass_z = 0;
    bodyposture_msg.left_arm_out = 0;
    bodyposture_msg.right_arm_out = 0;
    bodyposture_msg.left_hand_raised = 0;
    bodyposture_msg.right_hand_raised = 0;
    bodyposture_msg.akimibo = 0;
    bodyposture_msg.left_foot_up = 0;
    bodyposture_msg.right_foot_up = 0;
    bodyposture_msg.fall = 0;
    bodyposture_msg.tips =0;
    bodyposture_msg.lock_status =0;

	if(body_list.count !=0) lock_status=1;
    else lock_status=0;

	if(auto_lock_first_body && body_list.count != 0)
	{
		bool locked_body_seen = false;
		for(int i = 0; i < body_list.count; ++i)
		{
			if(body_list.bodies[i].bodyid == lock_body_id)
			{
				locked_body_seen = true;
				break;
			}
		}

		if(!body_locked_once || !locked_body_seen)
		{
			lock_body_id = body_list.bodies[0].bodyid;
			body_locked_once = true;
		}
	}

	for(int i = 0; i < body_list.count; ++i)
    {
    	bodyreader_msg::msg::Body body = body_list.bodies[i];

    	judge_pose(body);

    	if(body.bodyid == lock_body_id)
		{
			bodyposture_msg.centerofmass_x = body.centerofmass.x;
			bodyposture_msg.centerofmass_y = body.centerofmass.y;
			bodyposture_msg.centerofmass_z = body.centerofmass.z;

			lock_status = 2;
			bodyposture_msg.bodyid = lock_body_id;
		}
		
    }
    if(lock_status == 1 && last_lock_status != lock_status) bodyposture_msg.tips = 1;
    last_lock_status = lock_status ;


    if(lock_status == 2) 
    {
       if((bodyposture_msg.centerofmass_x / bodyposture_msg.centerofmass_z > 0.35)
           ||(bodyposture_msg.centerofmass_x / bodyposture_msg.centerofmass_z < -0.35)
           ||(bodyposture_msg.centerofmass_z < 1400))
       {
              bodyposture_msg.left_arm_out = 0;
              bodyposture_msg.right_arm_out = 0;
              bodyposture_msg.left_hand_raised = 0;
              bodyposture_msg.right_hand_raised = 0;
              bodyposture_msg.akimibo = 0;
              bodyposture_msg.left_foot_up = 0;
              bodyposture_msg.right_foot_up = 0;
              bodyposture_msg.fall = 0;
       }
    }
    else 
    {
    	cmd_vel_Pub->publish(geometry_msgs::msg::Twist());
    }
	bodyposture_msg.lock_status = lock_status;
    bodyposture_Pub->publish(bodyposture_msg);

}

void recoveryid_Callback(std_msgs::msg::Int16 recoveryid)
{
	lock_body_id = recoveryid.data;
	body_locked_once = true;
	bodyposture_msg.tips = 2;
}

int main(int argc, char *argv[])
{
  	rclcpp::init(argc, argv);
  	auto node = rclcpp::Node::make_shared("body_process");

	bodyposture_Pub = node->create_publisher<bodyreader_msg::msg::Bodyposture>("/body_posture", 1);
	auto bodylist_sub = node->create_subscription<bodyreader_msg::msg::Bodylist>("/bodylist", 1, bodylist_Callback);
	auto recoveryid_sub = node->create_subscription<std_msgs::msg::Int16>("/recoveryid", 1, recoveryid_Callback);

	mode_Pub = node->create_publisher<std_msgs::msg::Int8>("/mode", 1);
	mode_msg.data = 1;

	cmd_vel_Pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

	node->declare_parameter<bool>("open_switch", false);
	node->declare_parameter<bool>("require_akimbo_lock", true);
	node->declare_parameter<bool>("auto_lock_first_body", false);
	node->declare_parameter<int>("initial_mode", 1);
	node->declare_parameter<int>("pose_min_joint_status", JOINT_STATUS_TRACKED);
	node->declare_parameter<double>("arm_horizontal_y_tol_mm", 100.0);
	node->declare_parameter<double>("arm_horizontal_forearm_y_tol_mm", 150.0);
	node->declare_parameter<double>("arm_horizontal_segment_min_mm", 200.0);
	node->declare_parameter<double>("hand_raise_elbow_hand_min_mm", 130.0);
	node->declare_parameter<double>("hand_raise_shoulder_elbow_min_mm", 80.0);
	node->declare_parameter<double>("akimbo_hand_above_spine_min_mm", 50.0);
	node->declare_parameter<double>("akimbo_shoulder_hand_x_tol_mm", 100.0);
	node->declare_parameter<double>("foot_up_delta_mm", 120.0);
	node->declare_parameter<double>("fall_knee_spine_min_mm", -10.0);
  	node->get_parameter("open_switch", open_switch);
	node->get_parameter("require_akimbo_lock", require_akimbo_lock);
	node->get_parameter("auto_lock_first_body", auto_lock_first_body);
	node->get_parameter("initial_mode", initial_mode);
	node->get_parameter("pose_min_joint_status", pose_min_joint_status);

	if(initial_mode != 1 && initial_mode != 2)
	{
		RCLCPP_WARN(node->get_logger(), "initial_mode must be 1 or 2, fallback to 1");
		initial_mode = 1;
	}
	mode_msg.data = initial_mode;
	mode_Pub->publish(mode_msg);

	double arm_horizontal_y_tol_mm_param = arm_horizontal_y_tol_mm;
	double arm_horizontal_forearm_y_tol_mm_param = arm_horizontal_forearm_y_tol_mm;
	double arm_horizontal_segment_min_mm_param = arm_horizontal_segment_min_mm;
	double hand_raise_elbow_hand_min_mm_param = hand_raise_elbow_hand_min_mm;
	double hand_raise_shoulder_elbow_min_mm_param = hand_raise_shoulder_elbow_min_mm;
	double akimbo_hand_above_spine_min_mm_param = akimbo_hand_above_spine_min_mm;
	double akimbo_shoulder_hand_x_tol_mm_param = akimbo_shoulder_hand_x_tol_mm;
	double foot_up_delta_mm_param = foot_up_delta_mm;
	double fall_knee_spine_min_mm_param = fall_knee_spine_min_mm;

	node->get_parameter("arm_horizontal_y_tol_mm", arm_horizontal_y_tol_mm_param);
	node->get_parameter("arm_horizontal_forearm_y_tol_mm", arm_horizontal_forearm_y_tol_mm_param);
	node->get_parameter("arm_horizontal_segment_min_mm", arm_horizontal_segment_min_mm_param);
	node->get_parameter("hand_raise_elbow_hand_min_mm", hand_raise_elbow_hand_min_mm_param);
	node->get_parameter("hand_raise_shoulder_elbow_min_mm", hand_raise_shoulder_elbow_min_mm_param);
	node->get_parameter("akimbo_hand_above_spine_min_mm", akimbo_hand_above_spine_min_mm_param);
	node->get_parameter("akimbo_shoulder_hand_x_tol_mm", akimbo_shoulder_hand_x_tol_mm_param);
	node->get_parameter("foot_up_delta_mm", foot_up_delta_mm_param);
	node->get_parameter("fall_knee_spine_min_mm", fall_knee_spine_min_mm_param);

	arm_horizontal_y_tol_mm = static_cast<float>(arm_horizontal_y_tol_mm_param);
	arm_horizontal_forearm_y_tol_mm = static_cast<float>(arm_horizontal_forearm_y_tol_mm_param);
	arm_horizontal_segment_min_mm = static_cast<float>(arm_horizontal_segment_min_mm_param);
	hand_raise_elbow_hand_min_mm = static_cast<float>(hand_raise_elbow_hand_min_mm_param);
	hand_raise_shoulder_elbow_min_mm = static_cast<float>(hand_raise_shoulder_elbow_min_mm_param);
	akimbo_hand_above_spine_min_mm = static_cast<float>(akimbo_hand_above_spine_min_mm_param);
	akimbo_shoulder_hand_x_tol_mm = static_cast<float>(akimbo_shoulder_hand_x_tol_mm_param);
	foot_up_delta_mm = static_cast<float>(foot_up_delta_mm_param);
	fall_knee_spine_min_mm = static_cast<float>(fall_knee_spine_min_mm_param);

	rclcpp::spin(node);

	bodylist_sub.reset();
	recoveryid_sub.reset();
	bodyposture_Pub.reset();
	mode_Pub.reset();
	cmd_vel_Pub.reset();
	node.reset();

	if (rclcpp::ok())
	{
		rclcpp::shutdown();
	}

	return 0;
}
