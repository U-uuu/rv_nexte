// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#include <tf2/LinearMath/Quaternion.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// C++ system
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp> 

#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"
#include "rm_serial_driver/rm_serial_driver.hpp"

namespace rm_serial_driver
{
RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions & options)
: Node("rm_serial_driver", options),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
  RCLCPP_INFO(get_logger(), "Start RMSerialDriver!");

  getParams();

  // TF broadcaster
  timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Create Publisher
  latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("/latency", 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/aiming_point", 10);

  recieveYaw_pub_ = this->create_publisher<std_msgs::msg::Float64>("/recieveYaw", 10);
  sendYaw_pub_ = this->create_publisher<std_msgs::msg::Float64>("/sendYaw", 10);
  recievePitch_pub_ = this->create_publisher<std_msgs::msg::Float64>("/recievePitch", 10);
  sendPitch_pub_ = this->create_publisher<std_msgs::msg::Float64>("/sendPitch", 10);

  // Detect parameter client
  detector_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "armor_detector");

  // Tracker reset service client
  reset_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("/tracker/reset");

  try {
    serial_driver_->init_port(device_name_, *device_config_);
    if (!serial_driver_->port()->is_open()) {
      serial_driver_->port()->open();
      receive_thread_ = std::thread(&RMSerialDriver::receiveData, this);
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
    throw ex;
  }

  aiming_point_.header.frame_id = "odom";
  aiming_point_.ns = "aiming_point";
  aiming_point_.type = visualization_msgs::msg::Marker::SPHERE;
  aiming_point_.action = visualization_msgs::msg::Marker::ADD;
  aiming_point_.scale.x = aiming_point_.scale.y = aiming_point_.scale.z = 0.12;
  aiming_point_.color.r = 1.0;
  aiming_point_.color.g = 1.0;
  aiming_point_.color.b = 1.0;
  aiming_point_.color.a = 1.0;
  aiming_point_.lifetime = rclcpp::Duration::from_seconds(0.1);

  // Create Subscription
  target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
    "/tracker/target", rclcpp::SensorDataQoS(),
    std::bind(&RMSerialDriver::sendData, this, std::placeholders::_1));
}

RMSerialDriver::~RMSerialDriver()
{
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

  if (serial_driver_->port()->is_open()) {
    serial_driver_->port()->close();
  }

  if (owned_ctx_) {
    owned_ctx_->waitForExit();
  }
}

void RMSerialDriver::receiveData()
{
  std::vector<uint8_t> header(1);
  std::vector<uint8_t> data;
  data.resize(sizeof(ReceivePacket));

  while (rclcpp::ok()) {
    try {
      serial_driver_->port()->receive(data);

      // if (header[0] == 0x5A) {
        if (data[0] == 0xA5 && data[1] == 0x00) {
        ReceivePacket packet = fromVector(data);

        // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "reciece_header: %d",packet.header);
        // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "reciece_mode: %d",packet.mode);
        // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "reciece_color: %d",packet.detect_color);
        // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "reciece_yaw: %05.2f",packet.yaw);
        // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "reciece_pitch: %05.2f",packet.pitch);
        // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "reciece_speed: %05.2f",packet.speed);



        // bool crc_ok =
        //   crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
        // if (crc_ok) {

          shootspeed = packet.speed;
          recieve_pitch = packet.pitch;
          recieve_yaw = packet.yaw;
          if (!initial_set_param_ || packet.detect_color != previous_receive_color_) {
            setParam(rclcpp::Parameter("detect_color", packet.detect_color));
            previous_receive_color_ = packet.detect_color;
          }

          // if (packet.reset_tracker) {
          //   resetTracker();
          // }

          geometry_msgs::msg::TransformStamped t;
          timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
          t.header.stamp = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
          t.header.frame_id = "odom";
          t.child_frame_id = "gimbal_link";
          tf2::Quaternion q;
          q.setRPY(0, -packet.pitch/180.0*CV_PI, packet.yaw/180.0*CV_PI);
          t.transform.rotation = tf2::toMsg(q);
          tf_broadcaster_->sendTransform(t);

          // if (abs(packet.aim_x) > 0.01) {
          //   aiming_point_.header.stamp = this->now();
          //   aiming_point_.pose.position.x = packet.aim_x;
          //   aiming_point_.pose.position.y = packet.aim_y;
          //   aiming_point_.pose.position.z = packet.aim_z;
          //   marker_pub_->publish(aiming_point_);
          // }
          
          // by:Promise 
          // if (abs(target_armor_x) > 0.01) {
          //   aiming_point_.header.stamp = this->now();
          //   aiming_point_.pose.position.x = target_armor_x;
          //   aiming_point_.pose.position.y = target_armor_y;
          //   aiming_point_.pose.position.z = target_armor_z;
          //   marker_pub_->publish(aiming_point_);
          // }

        // } else {
        //   RCLCPP_ERROR(get_logger(), "CRC error!");
        // }
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "Invalid header: %02X", header[0]);
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
      reopenPort();
    }
  }
}

void RMSerialDriver::sendData(const auto_aim_interfaces::msg::Target::SharedPtr msg)
{
  const static std::map<std::string, uint8_t> id_unit8_map{
    {"", 0},  {"outpost", 0}, {"1", 1}, {"1", 1},     {"2", 2},
    {"3", 3}, {"4", 4},       {"5", 5}, {"guard", 6}, {"base", 7}};

  try {
    SendPacket packet;
    //是否在跟踪上
    bool istracking = msg->tracking;                  

    //对方机器人的中心xyz
    float detect_x = msg->position.x;
    float detect_y = msg->position.y;
    float detect_z = msg->position.z;

    //当前识别到的装甲板的相对敌方机器人中心的yaw
    float detect_yaw = msg->yaw;

    //对方机器人中心的xyz平移速度
    float detect_x_v = msg->velocity.x;
    float detect_y_v = msg->velocity.y;
    // float detect_z_v = msg->velocity.z;

    //当前识别到的装甲板的相对敌方机器人中心的yaw的旋转速度
    float detect_yaw_v = msg->v_yaw;

    //当前识别到的装甲板的半径radius_1
    float detect_radius_now = msg->radius_1;

    //上一次识别到的装甲板的半径radius_2
    float detect_radius_last = msg->radius_2;

    //装甲板的高度差
    // float detect_height_diff = msg->dz;

    //距离 = 机器人中心的距离- 装甲板半径的均值 
    distance = sqrt(pow(detect_x,2)+pow(detect_y,2)+pow(detect_z,2))-(detect_radius_now+detect_radius_last)/2;

    //延迟时间 = 飞行延迟 + 各种延迟
    // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "delayTime: %02f", delaytime);

    //对击打机器人进行预测
    float target_robot_x = detect_x + delayTime * detect_x_v;
    float target_robot_y = detect_y + delayTime * detect_y_v;
    float target_robot_z = detect_z;
    // float target_robot_z = detect_z + delayTime * detect_z_v;
    float target_robot_yaw = detect_yaw + delayTime * detect_yaw_v;

    // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "1.1: %02f", detect_x);
    // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "1.2: %02f", detect_x_v);
    // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "2.1: %02f", target_robot_x);
    // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "2: %02f", cos(target_robot_yaw));


    //对击打装甲板进行预测
    target_armor_x = target_robot_x - detect_radius_now * cos(target_robot_yaw);
    target_armor_y = target_robot_y - detect_radius_now * sin(target_robot_yaw);
    target_armor_z = target_robot_z ;

    aiming_point_.header.stamp = this->now();
    aiming_point_.pose.position.x = target_armor_x;
    aiming_point_.pose.position.y = target_armor_y;
    aiming_point_.pose.position.z = target_armor_z;
    marker_pub_->publish(aiming_point_);

    // 坐标系转换
    float target_myyaw = atan2(target_armor_y,target_armor_x);
    // float target_mypitch = atan2(target_armor_z,sqrt(target_armor_x*target_armor_x + target_armor_y*target_armor_y));

    // 弹道补偿
    float x = sqrt(target_armor_x*target_armor_x + target_armor_y*target_armor_y);
    float y = target_armor_z;
    float target_increase_y = increase(delayTime,shootspeed,x,y);
    float target_increase_mypitch = atan2(target_increase_y,sqrt(target_armor_x*target_armor_x + target_armor_y*target_armor_y));

    // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "y: %02f", target_increase_y);
    // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "inc_pitch: %02f",target_increase_mypitch/CV_PI*180.0);
    // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "pitch_increase: %02f", abs(target_mypitch/CV_PI*180.0-target_increase_mypitch/CV_PI*180.0));


    packet.yaw = target_myyaw/CV_PI*180.0;
    if(packet.yaw<0){
      packet.yaw+=360;
    }
    packet.pitch = target_increase_mypitch/CV_PI*180.0 + 0.0;
    // packet.pitch = target_mypitch/CV_PI*180.0 + 0.0;


    // float diff = abs(packet.yaw - recieve_yaw);
    // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "packet.yaw: %02f", packet.yaw );
    // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "recieve_yaw: %02f", recieve_yaw);
    // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "diff: %02f", diff);

    // float target_yaw = target_robot_yaw<0.0 ? target_robot_yaw+2*CV_PI : target_robot_yaw;
    // float deltaYaw=abs(target_yaw / CV_PI * 180.0 - recieve_yaw);
    // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "target_yaw: %02f", target_yaw);
    // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "deltaYaw: %02f", deltaYaw);






    if (istracking){
      // By: Promise 
      // if(deltaYaw<30.0){
        // if(abs(recieve_yaw - last_send_yaw) < 4){
          // packet.fire = 1;
        // }
        // if(abs(recieve_yaw-packet.yaw)<0.5){
          packet.fire = 1;
        // }
        // else{
        //   packet.fire = 0;
        // }
      }
      else{
        packet.fire = 0;
        packet.yaw = recieve_yaw;
        packet.pitch  = recieve_pitch;
        delayTime = 0;
      }
    // }
    
    // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "send_yaw: %02f", packet.yaw);
    // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "send_pitch: %02f", packet.pitch);
    // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "send_fire: %02d", packet.fire);
    // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "send_distance: %02f",distance);


    // crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));

    std::vector<uint8_t> data = toVector(packet);

    serial_driver_->port()->send(data);

    std_msgs::msg::Float64  recieveYaw,sendYaw;
    recieveYaw.data = recieve_yaw;
    sendYaw.data = packet.yaw;
    recieveYaw_pub_->publish(recieveYaw);
    sendYaw_pub_->publish(sendYaw);

    std_msgs::msg::Float64  recievePitch,sendPitch;
    recievePitch.data = recieve_pitch;
    sendPitch.data = packet.pitch;
    recievePitch_pub_->publish(recievePitch);
    sendPitch_pub_->publish(sendPitch);
    
    std_msgs::msg::Float64 latency;
    latency.data = (this->now() - msg->header.stamp).seconds() * 1000.0;
    RCLCPP_DEBUG_STREAM(get_logger(), "Total latency: " + std::to_string(latency.data) + "ms");
    latency_pub_->publish(latency);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
    reopenPort();
  }
}

void RMSerialDriver::getParams()
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  try{
    errorTime = declare_parameter<float>("errorTime",0.15);
    k = declare_parameter<float>("k",0.03);
  }catch(rclcpp::ParameterTypeException & ex){
    RCLCPP_ERROR(get_logger(), "The delaytime and k provided were failed");
    throw ex;
  }

  try {
    device_name_ = declare_parameter<std::string>("device_name", "");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    baud_rate = declare_parameter<int>("baud_rate", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  try {
    const auto fc_string = declare_parameter<std::string>("flow_control", "");

    if (fc_string == "none") {
      fc = FlowControl::NONE;
    } else if (fc_string == "hardware") {
      fc = FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      fc = FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
        "The flow_control parameter must be one of: none, software, or hardware."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  try {
    const auto pt_string = declare_parameter<std::string>("parity", "");

    if (pt_string == "none") {
      pt = Parity::NONE;
    } else if (pt_string == "odd") {
      pt = Parity::ODD;
    } else if (pt_string == "even") {
      pt = Parity::EVEN;
    } else {
      throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;
  }

  try {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");

    if (sb_string == "1" || sb_string == "1.0") {
      sb = StopBits::ONE;
    } else if (sb_string == "1.5") {
      sb = StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = StopBits::TWO;
    } else {
      throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }

  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}

void RMSerialDriver::reopenPort()
{
  RCLCPP_WARN(get_logger(), "Attempting to reopen port");
  try {
    if (serial_driver_->port()->is_open()) {
      serial_driver_->port()->close();
    }
    serial_driver_->port()->open();
    RCLCPP_INFO(get_logger(), "Successfully reopened port");
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
    if (rclcpp::ok()) {
      rclcpp::sleep_for(std::chrono::seconds(1));
      reopenPort();
    }
  }
}

void RMSerialDriver::setParam(const rclcpp::Parameter & param)
{
  if (!detector_param_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping parameter set");
    return;
  }

  if (
    !set_param_future_.valid() ||
    set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    RCLCPP_INFO(get_logger(), "Setting detect_color to %ld...", param.as_int());
    set_param_future_ = detector_param_client_->set_parameters(
      {param}, [this, param](const ResultFuturePtr & results) {
        for (const auto & result : results.get()) {
          if (!result.successful) {
            RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
            return;
          }
        }
        RCLCPP_INFO(get_logger(), "Successfully set detect_color to %ld!", param.as_int());
        initial_set_param_ = true;
      });
  }
}

void RMSerialDriver::resetTracker()
{
  if (!reset_tracker_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping tracker reset");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  reset_tracker_client_->async_send_request(request);
  // RCLCPP_INFO(get_logger(), "Reset tracker!");
}

float RMSerialDriver::increase(float &time_output,float speed,float x,float y){
    float gravity=9.7;
    float aim_y = y;
    float real_y,theta,time;
    for(size_t i = 0;i<20;i++){
      theta=atan2(aim_y,x);
      time=(exp(k * x) - 1) / (k * speed * cos(theta));
      real_y = speed * sin(theta) * time - gravity * (time*time) / 2;
      aim_y += (y-real_y);
      if(abs(real_y-y)<0.001){
        time_output = time + errorTime;
        return aim_y;
      }
    }
    return aim_y;
}


}  // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)
