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
#include <sstream>

#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"
#include "rm_serial_driver/rm_serial_driver.hpp"

namespace rm_serial_driver
{
  RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions &options)
      : Node("rm_serial_driver", options),
        owned_ctx_{new IoContext(2)},
        serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
  {
    RCLCPP_INFO(get_logger(), "Start RMSerialDriver!");

    getParams();

    // TF broadcaster
    // timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
    // tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // 创建客户端，用于向MoveIt节点发送请求
    auto client = this->create_client<rm_interfaces::srv::ArmData>("arm_data");
    rm_serial_driver::RMSerialDriver::SetArmClient set_arm_client(client);
    std::string service_name = client->get_service_name();
    RCLCPP_INFO(get_logger(), "Waiting for service: %s", service_name.c_str());
    arm_data_clients_.emplace(std::move(service_name), std::move(set_arm_client));

    // 正式代码需要
    try
    {
      serial_driver_->init_port(device_name_, *device_config_);
      if (!serial_driver_->port()->is_open())
      {
        serial_driver_->port()->open();
        receive_thread_ = std::thread(&RMSerialDriver::receiveData, this);
      }
    }
    catch (const std::exception &ex)
    {
      RCLCPP_ERROR(
          get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
      throw ex;
    }
  }

  RMSerialDriver::~RMSerialDriver()
  {
    if (receive_thread_.joinable())
    {
      receive_thread_.join();
    }

    if (serial_driver_->port()->is_open())
    {
      serial_driver_->port()->close();
    }

    if (owned_ctx_)
    {
      owned_ctx_->waitForExit();
    }
  }

  void RMSerialDriver::receiveData()
  {
    std::vector<uint8_t> header(1);
    std::vector<uint8_t> data;
    data.reserve(sizeof(ReceivePacket));

    while (rclcpp::ok())
    {
      try
      {
        serial_driver_->port()->receive(header);

        if (header[0] == 0x5A)
        {
          data.resize(sizeof(ReceivePacket) - 1);
          serial_driver_->port()->receive(data);

          data.insert(data.begin(), header[0]);
          ReceivePacket packet = fromVector(data);

          bool crc_ok =
              crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
          if (crc_ok)
          {
            latest_packet_ = packet;
            RCLCPP_INFO(get_logger(), "[Receive] reserved %d!", packet.reserved);

            // 打印接收到的数据用于调试
            RCLCPP_INFO(get_logger(), "Received packet: header=0x%02X, x=%f, y=%f, z=%f",
                        packet.header, packet.x, packet.y, packet.z);

            // 当packet.reserved为1时表示需要进行逆运动学计算
            if (packet.reserved == 1)
            {
              // 发送请求到MoveIt节点
              if (!arm_data_clients_.empty())
              {
                auto &client = arm_data_clients_.begin()->second;

                if (!client.on_waiting.load())
                {
                  // 创建请求
                  auto request = std::make_shared<rm_interfaces::srv::ArmData::Request>();
                  request->x = packet.x;
                  request->y = packet.y;
                  request->z = packet.z;
                  request->roll = packet.roll;
                  request->pitch = packet.pitch;
                  request->yaw = packet.yaw;

                  // 发送异步请求
                  client.on_waiting.store(true);
                  auto result = client.ptr->async_send_request(
                      request,
                      [this, &client](rclcpp::Client<rm_interfaces::srv::ArmData>::SharedFuture future)
                      {
                        client.on_waiting.store(false);
                        try
                        {
                          auto arm_response = future.get();
                          if (arm_response->success)
                          {
                            RCLCPP_INFO(get_logger(), "Received joint angles successfully!");
                            // 发送关节角度到串口
                            sendData(arm_response);
                          }
                          else
                          {
                            RCLCPP_ERROR(get_logger(), "Failed to get joint angles from MoveIt node!");
                          }
                        }
                        catch (const std::exception &e)
                        {
                          RCLCPP_ERROR(get_logger(), "Exception while handling service response: %s", e.what());
                        }
                      });

                  if (!result.valid())
                  {
                    client.on_waiting.store(false);
                    RCLCPP_ERROR(get_logger(), "Service request not valid");
                  }
                }
                else
                {
                  RCLCPP_WARN(get_logger(), "Previous request still pending, skipping");
                }
              }
              else
              {
                RCLCPP_ERROR(get_logger(), "No MoveIt service client available");
              }
            }
          }
          else
          {
            RCLCPP_ERROR(get_logger(), "CRC error!");
          }
        }
        else
        {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "Invalid header: %02X", header[0]);
        }
      }
      catch (const std::exception &ex)
      {
        RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
        reopenPort();
      }
    }
  }

  void RMSerialDriver::sendData(const std::shared_ptr<rm_interfaces::srv::ArmData::Response> response)
  {
    try
    {
      SendPacket packet;
      packet.start = 0xA5;

      std::stringstream ss;
      for (auto &joint : response->joint_angles)
      {
        ss << joint << " ";
      }
      RCLCPP_INFO(get_logger(), "ArmData: %s", ss.str().c_str());

      // 确保关节角度数量足够
      if (response->joint_angles.size() >= 6)
      {
        packet.joint1 = response->joint_angles[0];
        packet.joint2 = response->joint_angles[1];
        packet.joint3 = response->joint_angles[2];
        packet.joint4 = response->joint_angles[3];
        packet.joint5 = response->joint_angles[4];
        packet.joint6 = response->joint_angles[5];
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Insufficient joint angles (expected 6, got %zu)",
                     response->joint_angles.size());
        packet.joint1 = 0.0;
        packet.joint2 = 0.0;
        packet.joint3 = 0.0;
        packet.joint4 = 0.0;
        packet.joint5 = 0.0;
        packet.joint6 = 0.0;
      }

      // 计算CRC校验
      crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));

      // 发送数据
      std::vector<uint8_t> data = toVector(packet);

      serial_driver_->port()->send(data);
    }
    catch (const std::exception &ex)
    {
      RCLCPP_ERROR(this->get_logger(), "Error while sending data: %s", ex.what());
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

    try
    {
      device_name_ = declare_parameter<std::string>("device_name", "");
    }
    catch (rclcpp::ParameterTypeException &ex)
    {
      RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
      throw ex;
    }

    try
    {
      baud_rate = declare_parameter<int>("baud_rate", 0);
    }
    catch (rclcpp::ParameterTypeException &ex)
    {
      RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
      throw ex;
    }

    try
    {
      const auto fc_string = declare_parameter<std::string>("flow_control", "");

      if (fc_string == "none")
      {
        fc = FlowControl::NONE;
      }
      else if (fc_string == "hardware")
      {
        fc = FlowControl::HARDWARE;
      }
      else if (fc_string == "software")
      {
        fc = FlowControl::SOFTWARE;
      }
      else
      {
        throw std::invalid_argument{
            "The flow_control parameter must be one of: none, software, or hardware."};
      }
    }
    catch (rclcpp::ParameterTypeException &ex)
    {
      RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
      throw ex;
    }

    try
    {
      const auto pt_string = declare_parameter<std::string>("parity", "");

      if (pt_string == "none")
      {
        pt = Parity::NONE;
      }
      else if (pt_string == "odd")
      {
        pt = Parity::ODD;
      }
      else if (pt_string == "even")
      {
        pt = Parity::EVEN;
      }
      else
      {
        throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
      }
    }
    catch (rclcpp::ParameterTypeException &ex)
    {
      RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
      throw ex;
    }

    try
    {
      const auto sb_string = declare_parameter<std::string>("stop_bits", "");

      if (sb_string == "1" || sb_string == "1.0")
      {
        sb = StopBits::ONE;
      }
      else if (sb_string == "1.5")
      {
        sb = StopBits::ONE_POINT_FIVE;
      }
      else if (sb_string == "2" || sb_string == "2.0")
      {
        sb = StopBits::TWO;
      }
      else
      {
        throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
      }
    }
    catch (rclcpp::ParameterTypeException &ex)
    {
      RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
      throw ex;
    }

    device_config_ =
        std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
  }

  void RMSerialDriver::reopenPort()
  {
    RCLCPP_WARN(get_logger(), "Attempting to reopen port");
    try
    {
      if (serial_driver_->port()->is_open())
      {
        serial_driver_->port()->close();
      }
      serial_driver_->port()->open();
      RCLCPP_INFO(get_logger(), "Successfully reopened port");
    }
    catch (const std::exception &ex)
    {
      RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
      if (rclcpp::ok())
      {
        rclcpp::sleep_for(std::chrono::seconds(1));
        reopenPort();
      }
    }
  }

} // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)
