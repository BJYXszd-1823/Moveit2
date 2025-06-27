// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.
#include <tf2/LinearMath/Quaternion.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

// C++ system
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"
#include "rm_serial_driver/rm_serial_driver.hpp"

// #include "rm_utils/logger/log.hpp"
// #include "rm_utils/math/utils.hpp"

namespace rm_serial_driver
{
RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions & options)
: Node("rm_serial_driver", options),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
  RCLCPP_INFO(get_logger(), "Start RMSerialDriver!");

  getParams();

  setStartPose();

  // TF broadcaster
  timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Create Publisher
  // latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("/latency", 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/aiming_point", 10);

  // Param client

  auto client1 = this->create_client<rm_interfaces::srv::SetMode>("rune_detector/set_mode");
  set_mode_clients_.emplace(client1->get_service_name(), client1);
  auto client2 = this->create_client<rm_interfaces::srv::SetMode>("rune_solver/set_mode");
  set_mode_clients_.emplace(client2->get_service_name(), client2);

  // 创建客户端
  auto client = this->create_client<rm_interfaces::srv::ArmData>("arm_data");
  // arm_data_clients_.emplace(client->get_service_name(), client);
  // arm_data_clients_.emplace(std::string(client->get_service_name()), client);
  rm_serial_driver::RMSerialDriver::SetArmClient set_arm_client(client);
  std::string service_name = client->get_service_name();
  arm_data_clients_.emplace(std::move(service_name), std::move(set_arm_client));



  // 创建ArmData服务服务器
  arm_data_server_ = create_service<rm_interfaces::srv::ArmData>(
    "arm_data",
    std::bind(&RMSerialDriver::handleArmData, this, std::placeholders::_1, std::placeholders::_2));

  // 正式代码需要
  try {
    serial_driver_->init_port(device_name_, *device_config_);
    if (!serial_driver_->port()->is_open()) 
    {
      serial_driver_->port()->open();
      receive_thread_ = std::thread(&RMSerialDriver::receiveData, this);
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
    throw ex;
  }

  // // 手动启动接收线程
  // receive_thread_ = std::thread(&RMSerialDriver::receiveData, this);
  
  // // 延迟初始化MoveGroup
  // is_move_group_initialized_ = false;
  // is_fully_constructed_ = false; // 表示对象是否完全构造好
  


  // MoveIt 初始化，指定规划组名称为 test
  move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "test");
  move_group_->setPlanningTime(5.0);
  

  // // 创建下位机数据接收方
  // LowCom_sub = this->create_subscription<rm_interfaces::msg::LowComputer>(
  //   "rune_solver/LowComputer",rclcpp::SensorDataQoS(),
  //   std::bind(&RMSerialDriver::sendData, this, std::placeholders::_1));

  // Create Subscription
  target_sub_ = this->create_subscription<rm_interfaces::msg::GimbalCmd>(
    "rune_solver/cmd_gimbal", rclcpp::SensorDataQoS(),
    std::bind(&RMSerialDriver::sendData, this, std::placeholders::_1));

  // subscriptions_ = this->getSubscriptions(this->shared_from_this());
  // for (auto sub : subscriptions_) {
  //   FYT_INFO("serial_driver", "Subscribe to topic: {}", sub->get_topic_name());
  // }

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

// 设置起始位姿
void RMSerialDriver::setStartPose()
{
  // 根据需求需要设置起始位姿的具体值（机械测量）
  start_pose_.position.x = 0.0;
  start_pose_.position.y = 0.0;
  start_pose_.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, 0.0);
  start_pose_.orientation = tf2::toMsg(q);

  current_pose_ = start_pose_;
}

// // 添加初始化函数
// void RMSerialDriver::initMoveGroup()
// {
//     // 使用互斥锁保证线程安全
//     std::lock_guard<std::mutex> lock(init_mutex_);
    
//     if (!is_move_group_initialized_) {
//         // 确保对象已经完全构造好
//         while (!is_fully_constructed_) {
//             std::this_thread::sleep_for(std::chrono::milliseconds(10));
//         }
        
//         move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "test");
//         move_group_->setPlanningTime(5.0);
//         is_move_group_initialized_ = true;
//         RCLCPP_INFO(get_logger(), "MoveGroup initialized successfully!");
//     }
// }


// 处理ArmData服务请求的回调函数
void RMSerialDriver::handleArmData(
  const std::shared_ptr<rm_interfaces::srv::ArmData::Request> request,
  std::shared_ptr<rm_interfaces::srv::ArmData::Response> response)
{
  // 处理接收到的关节角度
  const auto & joint_angles = request->joint_angles;
  RCLCPP_INFO(get_logger(), "Received joint angles: %f, %f, %f, %f, %f, %f",
    joint_angles[0], joint_angles[1], joint_angles[2],
    joint_angles[3], joint_angles[4], joint_angles[5]);

  // 在这里可以添加对关节角度的处理逻辑
  // ...

  // 设置响应数据（使用最新接收到的位置和欧拉角）
  response->x = latest_packet_.x;
  response->y = latest_packet_.y;
  response->z = latest_packet_.z;
  response->roll = latest_packet_.roll;
  response->pitch = latest_packet_.pitch;
  response->yaw = latest_packet_.yaw;
}

// 新增：发送数据并调用服务的函数
void RMSerialDriver::sendDataExchange(rm_interfaces::msg::GimbalCmd::SharedPtr msg)
{
  if (arm_data_clients_.empty()) {
    RCLCPP_ERROR(get_logger(), "No data exchange clients available");
    return;
  }

  // 使用第一个客户端
  auto & client = arm_data_clients_.begin()->second;
  
  if (client.on_waiting.load()) {
    RCLCPP_WARN(get_logger(), "Previous request still pending, skipping");
    return;
  }

  // 创建请求
  auto request = std::make_shared<rm_interfaces::srv::ArmData::Request>();
  if (msg->joint_angles.size() >= 6) {
    request->joint_angles = {
      static_cast<float>(msg->joint_angles[0]),
      static_cast<float>(msg->joint_angles[1]),
      static_cast<float>(msg->joint_angles[2]),
      static_cast<float>(msg->joint_angles[3]),
      static_cast<float>(msg->joint_angles[4]),
      static_cast<float>(msg->joint_angles[5])
    };
  } else {
    RCLCPP_ERROR(this->get_logger(), "Insufficient joint angles (expected 6, got %zu)", 
      msg->joint_angles.size());
    request->joint_angles = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  }

  // 发送异步请求
  client.on_waiting.store(true);
  auto result = client.ptr->async_send_request(
    request,
    [this, &client](rclcpp::Client<rm_interfaces::srv::ArmData>::SharedFuture future) {
      client.on_waiting.store(false);
      try {
        auto response = future.get();
        RCLCPP_INFO(get_logger(), "Received position: x=%.3f, y=%.3f, z=%.3f", 
                    response->x, response->y, response->z);
        RCLCPP_INFO(get_logger(), "Received Euler angles: roll=%.3f, pitch=%.3f, yaw=%.3f", 
                    response->roll, response->pitch, response->yaw);
        
        // 在这里可以添加对接收到的数据的处理逻辑
        // ...
        
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Exception while handling service response: %s", e.what());
      }
    });

  if (!result.valid()) {
    client.on_waiting.store(false);
    RCLCPP_ERROR(get_logger(), "Service request not valid");
  }
}

void RMSerialDriver::receiveData()
{
  std::vector<uint8_t> header(1);
  std::vector<uint8_t> data;
  data.reserve(sizeof(ReceivePacket));

  while (rclcpp::ok()) {
    //  // 模拟接收到的数据
    //  ReceivePacket packet;
    //  packet.header = 0x5A;
    //  packet.detect_color = 0;
    //  packet.reset_tracker = false;
    //  packet.reserved = 1; // 启用 MoveIt
    //  packet.roll = 0.1;
    //  packet.pitch = 0.2;
    //  packet.yaw = 0.3;
    //  packet.x = 0.4;
    //  packet.y = 0.5;
    //  packet.z = 0.6;
    try {
      serial_driver_->port()->receive(header);

      if (header[0] == 0x5A) {
        data.resize(sizeof(ReceivePacket) - 1);
        serial_driver_->port()->receive(data);

        data.insert(data.begin(), header[0]);
        ReceivePacket packet = fromVector(data);

        bool crc_ok =
          crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
        if (crc_ok) 
        {

          // RCLCPP_INFO(get_logger(), "CRC OK!");
          // if(packet.reserved == 0)
          // {
          //   int ret = system("gnome-terminal -- /home/xianyu/kill_buff.sh");
          //   if(ret == 0 ){}
          // }

          // RCLCPP_INFO(get_logger(), "[Receive] pitch %f!", packet.pitch);
          // RCLCPP_INFO(get_logger(), "[Receive] yaw %f!", packet.yaw);
          latest_packet_ = packet;
          RCLCPP_INFO(get_logger(), "[Receive] reserved %d!", packet.reserved);

          // 当packet.reserved为1时表示启用MoveIt
          moveit_enabled_ = (packet.reserved == 1);

          if (moveit_enabled_) {

            // // 初始化MoveIt
            // if (!is_move_group_initialized_) {
            //   initMoveGroup();
            // }

            // 启用状态下，接收下位机的末端位姿
            geometry_msgs::msg::Pose target_pose;
            target_pose.position.x = packet.x;
            target_pose.position.y = packet.y;
            target_pose.position.z = packet.z;

            tf2::Quaternion q;
            q.setRPY(packet.roll, packet.pitch, packet.yaw);
            target_pose.orientation = tf2::toMsg(q);

            // 以上一次位姿作为起始位置进行规划
            move_group_->setStartStateToCurrentState();
            move_group_->setStartState(*move_group_->getCurrentState());
            move_group_->setPoseTarget(target_pose);

            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            moveit::core::MoveItErrorCode error_code = move_group_->plan(my_plan);

            if (error_code == moveit::core::MoveItErrorCode::SUCCESS) {
              RCLCPP_INFO(get_logger(), "Plan found!");
              if (!my_plan.trajectory_.joint_trajectory.points.empty()) {
                // 执行规划
                move_group_->execute(my_plan);
                // 更新当前位姿
                current_pose_ = target_pose;
              } else {
                RCLCPP_ERROR(get_logger(), "Generated trajectory is empty!");
              }
            } 
            else {
              RCLCPP_ERROR(get_logger(), "Planning failed! Error code: %d", error_code.val);
            }
          } else {
            // 非启用状态下，将起始位置恢复成指定位姿
            setStartPose();
          }

          for (auto &[service_name, client] : set_mode_clients_) 
          {
            if (client.mode.load() != packet.reserved && !client.on_waiting.load()) 
            {
              setMode(client, packet.reserved);
            }
          }

          geometry_msgs::msg::TransformStamped t;
          // 个人认为这个参数是下位机发送数据到上位机的时延(单位:s)
          timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
          t.header.stamp = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
          t.header.frame_id = "odom";
          t.child_frame_id = "gimbal_link";
          tf2::Quaternion q;
          q.setRPY(packet.roll, packet.pitch, packet.yaw);
          t.transform.rotation = tf2::toMsg(q);
          tf_broadcaster_->sendTransform(t);

        } else 
        {
          // RCLCPP_INFO(get_logger(), "[Receive] reserved %d!", packet.reserved);
          
          // LOG [Receive] [Receive] rpy
          // RCLCPP_INFO(get_logger(), "[Receive] roll %f!", packet.roll);
          // RCLCPP_INFO(get_logger(), "[Receive] pitch %f!", packet.pitch);
          // RCLCPP_INFO(get_logger(), "[Receive] yaw %f!", packet.yaw);
          RCLCPP_ERROR(get_logger(), "CRC error!");
        }
      } else 
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "Invalid header: %02X", header[0]);
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
      reopenPort();
    }
  }
}

void RMSerialDriver::processReceivedData(const ReceivePacket& packet, rm_interfaces::msg::GimbalCmd::SharedPtr msg)
{
  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = packet.x;
  target_pose.position.y = packet.y;
  target_pose.position.z = packet.z;

  tf2::Quaternion q;
  q.setRPY(packet.roll, packet.pitch, packet.yaw);
  target_pose.orientation = tf2::toMsg(q);

  std::vector<double> joint_angles = planTrajectory(target_pose);
  if (!joint_angles.empty()) {
    sendData(msg);
  }
}

std::vector<double> RMSerialDriver::planTrajectory(const geometry_msgs::msg::Pose& target_pose)
{
  move_group_->setPoseTarget(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::core::MoveItErrorCode error_code = move_group_->plan(my_plan);

  if (error_code == moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_INFO(get_logger(), "Plan found!");
    // 使用MoveIt 2的正确方式获取关节轨迹
    if (!my_plan.trajectory_.joint_trajectory.points.empty()) {
      return my_plan.trajectory_.joint_trajectory.points.back().positions;
    } else {
      RCLCPP_ERROR(get_logger(), "Generated trajectory is empty!");
      return std::vector<double>();
    }
  } else {
    RCLCPP_ERROR(get_logger(), "Planning failed! Error code: %d", error_code.val);
    return std::vector<double>();
  }
}

void RMSerialDriver::sendData(rm_interfaces::msg::GimbalCmd::SharedPtr msg)
{
  // 实际运用
  try {
    SendPacket packet;
    packet.start = 0xA5;
    packet.tracking = 0; // 假设tracking标志为1表示正在跟踪目标
    
    // 确保关节角度数量足够
    if (msg->joint_angles.size() >= 6) {
      // 直接从vector中提取关节角度并映射到packet字段
      packet.joint1 = msg->joint_angles[0];
      packet.joint2 = msg->joint_angles[1];
      packet.joint3 = msg->joint_angles[2];
      packet.joint4 = msg->joint_angles[3];
      packet.joint5 = msg->joint_angles[4];
      packet.joint6 = msg->joint_angles[5];
    } else {
      RCLCPP_ERROR(this->get_logger(), "Insufficient joint angles (expected 6, got %zu)", 
      msg->joint_angles.size());
      // 使用默认值或重置关节
      packet.joint1 = 0.0;
      packet.joint2 = 0.0;
      packet.joint3 = 0.0;
      packet.joint4 = 0.0;
      packet.joint5 = 0.0;
      packet.joint6 = 0.0;
    }
    
    // 计算CRC校验
    crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t*>(&packet), sizeof(packet));
    
    // 发送数据
    std::vector<uint8_t> data = toVector(packet);
    serial_driver_->port()->send(data);
    
  } catch (const std::exception& ex) {
    RCLCPP_ERROR(this->get_logger(), "Error while sending data: %s", ex.what());
    reopenPort();
  }
}




// void RMSerialDriver::sendData(const rm_interfaces::msg::GimbalCmd::SharedPtr msg)
// {
//   // const static std::map<std::string, uint8_t> id_unit8_map{
//   //   {"", 0},  {"outpost", 0}, {"1", 1}, {"1", 1},     {"2", 2},
//   //   {"3", 3}, {"4", 4},       {"5", 5}, {"guard", 6}, {"base", 7}};

//   try {
//     SendPacket packet;
//     packet.start = 0xA5;
//     packet.tracking = 0; //标志位,下位机通过该标志位执行解算操作

//     // packet.T_x = msg->position.x;
//     // packet.T_y = msg->position.y;
//     // packet.T_z = msg->position.z;

//     // packet.R_x = msg->r_tag.x;
//     // packet.R_y = msg->r_tag.y;
//     // packet.R_z = msg->r_tag.z;

//     // packet.direction = msg->direction;
//     // packet.b_a = msg->b_a;
//     // packet.b_omega = msg->b_omega;
//     // packet.b_t0 = msg->b_t0;
//     // packet.b_b = msg->b_b;
//     // packet.b_c = msg->b_c;

//     packet.id = 0;
//     packet.armors_num = 0;
//     packet.reserved = 1;
//     packet.x = 0;
//     packet.y = 0;
//     packet.z = 0;
//     packet.yaw = 0;
//     packet.vx = 0;
//     packet.vy = 0;
//     packet.vz = 0;
//     packet.v_yaw = 0;
//     packet.r1 = msg->yaw_diff;

//     // std::cout << "aim_pitch:                    " << msg->pitch << std::endl;
//     // std::cout << "aim_pitch_diff:                    " << msg->pitch_diff << std::endl;
//     // std::cout << "aim_yaw:                    " << msg->yaw << std::endl;
//     // std::cout << "aim_yaw_diff:                    " << msg->yaw_diff << std::endl;

//     packet.r2 = msg->pitch_diff;
//     packet.dz = msg->distance;
//     packet.checksum =0;
//     crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));

//     std::vector<uint8_t> data = toVector(packet);

//     serial_driver_->port()->send(data);

//   } catch (const std::exception & ex) {
//     RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
//     reopenPort();
//   }
// }

void RMSerialDriver::getParams()
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

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

// void RMSerialDriver::setParam(const rclcpp::Parameter & param)
// {
//   // if (!detector_param_client_->service_is_ready()) {
//   //   RCLCPP_WARN(get_logger(), "Service not ready, skipping parameter set");
//   //   return;
//   // }

//   if (
//     !set_param_future_.valid() ||
//     set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
//     RCLCPP_INFO(get_logger(), "Setting detect_color to %ld...", param.as_int());
//     set_param_future_ = detector_param_client_->set_parameters(
//       {param}, [this, param](const ResultFuturePtr & results) {
//         for (const auto & result : results.get()) {
//           if (!result.successful) {
//             RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
//             return;
//           }
//         }
//         RCLCPP_INFO(get_logger(), "Successfully set detect_color to %ld!", param.as_int());
//         initial_set_param_ = true;
//       });
//   }
// }

// 设置模式
void RMSerialDriver::setMode(SetModeClient &client, const uint8_t mode) {
  using namespace std::chrono_literals;

  std::string service_name = client.ptr->get_service_name();
  // Wait for service
  while (!client.ptr->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      // FYT_ERROR(
      //   "serial_driver", "Interrupted while waiting for the service {}. Exiting.", service_name);
      return;
    }
    // FYT_INFO("serial_driver", "Service {} not available, waiting again...", service_name);
  }
  if (!client.ptr->service_is_ready()) {
    // FYT_WARN("serial_driver", "Service: {} is not available!", service_name);
    return;
  }
  // Send request
  auto req = std::make_shared<rm_interfaces::srv::SetMode::Request>();
  req->mode = mode;

  client.on_waiting.store(true);
  auto result = client.ptr->async_send_request(
    req, [mode, &client](rclcpp::Client<rm_interfaces::srv::SetMode>::SharedFuture result) {
      client.on_waiting.store(false);
      if (result.get()->success) {
        client.mode.store(mode);
      }
    });
}

}  // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)
