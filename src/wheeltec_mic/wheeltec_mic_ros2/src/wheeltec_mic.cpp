/****************************************************************/
/* Copyright (c) 2025 WHEELTEC Technology, Inc   				*/
/* function:Serial port analysis							    */
/* 功能：串口解析										            */
/****************************************************************/
#include "wheeltec_mic.h"

#include <algorithm>
#include <numeric>

/**************************************
Function: Get the serial port return field
功能: 获取串口返回字段
***************************************/
bool Wheeltec_Mic::UnPackMsgPacket(const std::string &content, MsgPacket &data)
{
	if (content.size() < 7 || ((unsigned char)content.at(0) != FRAME_HEADER))
		return false;

	data.uid  = content[1] & 0xff;
	data.type = content[2] & 0xff;
	data.size = (content[3] & 0xff) | ((content[4] << 8 & 0xff00));
	data.sid  = (content[5] & 0xff) | ((content[6] << 8 & 0xff00));

	switch((MsgType)data.type)
	{
		case MsgType::AIUI_MSG:
		{
			std::string info = content.substr(7,data.size);
			data.bytes  = info;
			return true;
		}
			break;
		case MsgType::CONTROL:
		{
			std::string info = content.substr(7,data.size);
			data.bytes  = info;
			return true;
		}
			break;
		default:
        	break;
	}
    return false;
}

/**************************************
Function: Verify serial port data and parse information
功能: 校验串口数据并解析信息
***************************************/
int Wheeltec_Mic::process_data(const unsigned char *buf, int len)
{
  if (buf == nullptr || len < 8)
  {
    return -1;
  }

  int sum = std::accumulate(buf, buf + len - 1, 0);
  if (((~sum + 1) & 0xff) != buf[len - 1])
  {
    return -1;
  }

  const MsgType frame_type = static_cast<MsgType>(buf[2]);
  if (frame_type == MsgType::Shake || frame_type == MsgType::CONFIRM)
  {
    protocol_ready = true;
    if (protocol_keepalive && frame_type == MsgType::Shake)
    {
      send_protocol_frame(MsgType::CONFIRM);
    }
    maybe_send_startup_config();
    return 1;
  }

  if (frame_type == MsgType::AIUI_MSG || frame_type == MsgType::CONTROL)
  {
    protocol_ready = true;
  }

  if (UnPackMsgPacket(std::string(reinterpret_cast<const char *>(buf), len), MsgPkg))
  {
    Json::Reader reader;
    if (static_cast<MsgType>(MsgPkg.type) == MsgType::AIUI_MSG)
    {
      Json::Value Aiui_Msg;
      Json::Value value_iwv;
      if (reader.parse(MsgPkg.bytes, Aiui_Msg))
      {
        if (Aiui_Msg["type"].asString() == "aiui_event")
        {
          Json::Value content = Aiui_Msg["content"];
          if (content["eventType"].asString() == "4")
          {
            std_msgs::msg::Int8 awake_msg;
            awake_msg.data = 1;
            awake_flag_pub->publish(awake_msg);

            std::string iwv_msg = content["info"].asString();
            if (reader.parse(iwv_msg, value_iwv))
            {
              angle = value_iwv["ivw"]["angle"].asInt();
              std_msgs::msg::UInt32 angle_msg;
              angle_msg.data = angle;
              angle_pub->publish(angle_msg);
              std_msgs::msg::String msg;
              msg.data = "小车唤醒";
              voice_words_pub->publish(msg);
              std::cout << ">>>>>唤醒 角度为: " << angle << "°" << std::endl;
            }
            else
            {
              std::cout << "reader json fail!" << std::endl;
            }
          }
        }
        else
        {
          device_message = Aiui_Msg["content"].asString();
          process_result = true;
        }
      }
      maybe_send_startup_config();
      return 1;
    }
  }
  maybe_send_startup_config();
  return -1;
}

/**************************************
Function: Receive and filter data
功能: 过滤数据
***************************************/
int Wheeltec_Mic::uart_analyse(unsigned char buffer)
{
	static int count=0, frame_len=0;
	Receive_Data[count] = buffer;
    if(Receive_Data[0] != FRAME_HEADER || (count == 1 && Receive_Data[1] != USER_ID))  
      count = 0,frame_len = 0;
    else 
      count++;
	if (count == 7){  
      frame_len = (Receive_Data[4]<<8 | Receive_Data[3]) + 7 + 1;
      if (frame_len > 1024) {
          //RCLCPP_ERROR(this->get_logger(), "Frame length exceeds buffer size");
          count = frame_len = 0;
          memset(Receive_Data, 0, sizeof(Receive_Data));
          return 0;
      }
	}
	if(count == frame_len && frame_len > 0){
		int ret = process_data(Receive_Data,frame_len);
		count = 0,frame_len = 0;
		memset(Receive_Data, 0, 1024);
		return ret;
	}
	return 0;
}

/**************************************
Function: Receive the information sent by the device
功能: 接收下位机发送的信息
***************************************/
bool Wheeltec_Mic::Get_Serial_Data()
{
  if (!serial_initialized)
  {
    return false;
  }

  try
  {
    size_t available = MicArr_Serial.available();
    if (available == 0)
    {
      return false;
    }
    const size_t max_read_size = 128;
    size_t to_read = std::min(available, max_read_size);
    unsigned char buffer[max_read_size];
    size_t bytes_read = MicArr_Serial.read(buffer, to_read);
    bool result = false;

    for (size_t i = 0; i < bytes_read; i++)
    {
      if (uart_analyse(buffer[i]))
      {
        result = true;
      }
    }
    return result;
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "Serial read error: %s", e.what());
    handle_serial_error();
  }
  return false;
}

/**************************************
Function: Handle serial port errors
功能: 处理串口异常
***************************************/
void Wheeltec_Mic::handle_serial_error()
{
  const bool was_connected = serial_initialized;
  serial_initialized = false;
  protocol_ready = false;
  startup_config_sent = false;

  try
  {
    if (MicArr_Serial.isOpen())
    {
      MicArr_Serial.close();
    }
  }
  catch (const std::exception &e)
  {
    RCLCPP_WARN(this->get_logger(), "Error while closing serial port: %s", e.what());
  }

  if (was_connected)
  {
    publish_voice_flag(0);
  }
  last_reconnect_attempt = std::chrono::steady_clock::now() -
    std::chrono::milliseconds(serial_retry_interval_ms);
  RCLCPP_WARN(this->get_logger(),
    "Serial connection lost; retrying %s every %d ms",
    usart_port_name.c_str(), serial_retry_interval_ms);
}

/**************************************
Function: Serial port data processing callback
功能: 串口数据处理回调函数
***************************************/
void Wheeltec_Mic::serial_read_callback()
{
  const auto now = std::chrono::steady_clock::now();
  if (!serial_initialized)
  {
    if (now - last_reconnect_attempt >= std::chrono::milliseconds(serial_retry_interval_ms))
    {
      last_reconnect_attempt = now;
      initialize_serial();
    }
    return;
  }

  maybe_send_keepalive();
  Get_Serial_Data();
  maybe_send_startup_config();
}

/**************************************
Function: Loop access to the lower computer data and issue topics
功能: 循环获取下位机数据与发布话题
***************************************/
void Wheeltec_Mic::run()
{
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&Wheeltec_Mic::serial_read_callback, this));

  if (!serial_initialized)
  {
    RCLCPP_WARN(this->get_logger(),
      "Serial port is not ready; node will keep retrying in the background");
  }
  rclcpp::spin(shared_from_this());
}

/**************************************
Function: Initialize serial port with retry
功能: 串口初始化
***************************************/
void Wheeltec_Mic::initialize_serial()
{
  serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
  for (int retry = 0; retry < serial_open_retries; ++retry)
  {
    try
    {
      if (MicArr_Serial.isOpen())
      {
        MicArr_Serial.close();
      }
      MicArr_Serial.setPort(usart_port_name);
      MicArr_Serial.setBaudrate(serial_baud_rate);
      MicArr_Serial.setTimeout(timeout);
      MicArr_Serial.open();
      if (MicArr_Serial.isOpen())
      {
        MicArr_Serial.flush();
        serial_initialized = true;
        protocol_ready = false;
        startup_config_sent = false;
        last_keepalive_tx = std::chrono::steady_clock::now() -
          std::chrono::milliseconds(keepalive_interval_ms);
        publish_voice_flag(1);
        RCLCPP_INFO(get_logger(), "Serial port initialized: %s at %d baud",
          usart_port_name.c_str(), serial_baud_rate);
        std::cout << ">>>>>成功打开麦克风设备" << std::endl;
        std::cout << ">>>>>以降噪板设置的唤醒词为准[默认:小微小微] " << std::endl;
        return;
      }
    }
    catch (const std::exception &e)
    {
      RCLCPP_WARN(get_logger(), "Serial open attempt %d/%d failed: %s",
        retry + 1, serial_open_retries, e.what());
    }
    if (retry + 1 < serial_open_retries)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(serial_retry_interval_ms));
    }
  }
  serial_initialized = false;
  RCLCPP_WARN(get_logger(),
    "Unable to open serial port %s; background retry remains active",
    usart_port_name.c_str());
}

bool Wheeltec_Mic::send_protocol_frame(MsgType type)
{
  if (!serial_initialized)
  {
    return false;
  }
  const std::string payload("\xA5\x00\x00\x00", 4);
  try
  {
    MicArr_Serial.write(MakeMsgPacket(0, type, payload));
    return true;
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "Serial protocol write failed: %s", e.what());
    handle_serial_error();
    return false;
  }
}

void Wheeltec_Mic::maybe_send_keepalive()
{
  if (!protocol_keepalive || !serial_initialized)
  {
    return;
  }
  const auto now = std::chrono::steady_clock::now();
  if (now - last_keepalive_tx < std::chrono::milliseconds(keepalive_interval_ms))
  {
    return;
  }
  last_keepalive_tx = now;
  send_protocol_frame(MsgType::Shake);
}

void Wheeltec_Mic::maybe_send_startup_config()
{
  if (!serial_initialized || !protocol_ready || startup_config_sent || startup_mic_type.empty())
  {
    return;
  }
  ServicePacket startup_pkg;
  startup_pkg.type = "switch_mic";
  startup_pkg.sid = 0;
  startup_pkg.msgType = ServiceType::SWITCH_MIC;
  startup_pkg.mic_type = startup_mic_type;
  if (Send_Serial_Data(startup_pkg))
  {
    startup_config_sent = true;
    RCLCPP_INFO(this->get_logger(), "Microphone topology selected: %s",
      startup_mic_type.c_str());
  }
}

void Wheeltec_Mic::publish_voice_flag(int value)
{
  if (!voice_flag_pub)
  {
    return;
  }
  std_msgs::msg::Int8 flag_msg;
  flag_msg.data = value;
  voice_flag_pub->publish(flag_msg);
}

/**************************************
Function: Constructor, executed only once, for initialization
功能: 构造函数, 用于初始化
***************************************/
Wheeltec_Mic::Wheeltec_Mic(const std::string &node_name)
: rclcpp::Node(node_name),
  process_result(false),
  serial_initialized(false),
  protocol_keepalive(false),
  protocol_ready(false),
  startup_config_sent(false)
{
  memset(&Receive_Data, 0, sizeof(Receive_Data));

  setupMicArrayServices();

  this->declare_parameter<std::string>("usart_port_name", "/dev/ttyCH343USB0");
  this->declare_parameter<int>("serial_baud_rate", 115200);
  this->declare_parameter<bool>("protocol_keepalive", false);
  this->declare_parameter<int>("keepalive_interval_ms", 200);
  this->declare_parameter<int>("serial_retry_interval_ms", 1000);
  this->declare_parameter<int>("serial_open_retries", 3);
  this->declare_parameter<std::string>("startup_mic_type", "");

  this->get_parameter("usart_port_name", usart_port_name);
  this->get_parameter("serial_baud_rate", serial_baud_rate);
  this->get_parameter("protocol_keepalive", protocol_keepalive);
  this->get_parameter("keepalive_interval_ms", keepalive_interval_ms);
  this->get_parameter("serial_retry_interval_ms", serial_retry_interval_ms);
  this->get_parameter("serial_open_retries", serial_open_retries);
  this->get_parameter("startup_mic_type", startup_mic_type);

  keepalive_interval_ms = std::max(50, keepalive_interval_ms);
  serial_retry_interval_ms = std::max(100, serial_retry_interval_ms);
  serial_open_retries = std::max(1, serial_open_retries);
  last_reconnect_attempt = std::chrono::steady_clock::now();

  awake_flag_pub = this->create_publisher<std_msgs::msg::Int8>("awake_flag", 10);
  voice_flag_pub = this->create_publisher<std_msgs::msg::Int8>("voice_flag", 10);
  angle_pub = this->create_publisher<std_msgs::msg::UInt32>("awake_angle", 10);
  voice_words_pub = this->create_publisher<std_msgs::msg::String>("voice_words", 10);

  RCLCPP_INFO(this->get_logger(),
    "Mic serial configuration: port=%s baud=%d keepalive=%s",
    usart_port_name.c_str(), serial_baud_rate, protocol_keepalive ? "on" : "off");
  initialize_serial();
}

Wheeltec_Mic::~Wheeltec_Mic()
{
	RCLCPP_INFO(this->get_logger(),"wheeltec_mic_node over!\n");

	if (timer_) {
		timer_->cancel();
	}

	if (MicArr_Serial.isOpen()) {
		MicArr_Serial.close();
	}
}

int main(int argc,char **argv)
{
	rclcpp::init(argc,argv);
	auto mic = std::make_shared<Wheeltec_Mic>("wheeltec_mic");
  	mic->run();
  	rclcpp::shutdown();
	return 0;
}
