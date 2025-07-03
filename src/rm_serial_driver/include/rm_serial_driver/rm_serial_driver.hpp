// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#define RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>

// C++ system
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <mutex>
#include <atomic>

// #include "auto_aim_interfaces/msg/target.hpp"
#include "rm_interfaces/srv/set_mode.hpp"
#include "rm_interfaces/srv/arm_data.hpp"
// #include "rm_interfaces/msg/low_computer.hpp"
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_serial_driver/packet.hpp"

// moveit
// #include <moveit/move_group_interface/move_group_interface.h>

namespace rm_serial_driver
{
  class RMSerialDriver : public rclcpp::Node
  {
  public:
    explicit RMSerialDriver(const rclcpp::NodeOptions &options);

    ~RMSerialDriver() override;

    // ArmData
    struct SetArmClient
    {
      rclcpp::Client<rm_interfaces::srv::ArmData>::SharedPtr ptr;
      std::atomic<bool> on_waiting{false};

      // 移动构造函数
      SetArmClient(SetArmClient &&other) noexcept
          : ptr(std::move(other.ptr)),
            on_waiting(other.on_waiting.load()) {}

      // 移动赋值运算符
      SetArmClient &operator=(SetArmClient &&other) noexcept
      {
        if (this != &other)
        {
          ptr = std::move(other.ptr);
          on_waiting = other.on_waiting.load();
        }
        return *this;
      }

      // 拷贝构造函数（如果需要）
      SetArmClient(const SetArmClient &other) = delete;            // 可以禁用拷贝构造
      SetArmClient &operator=(const SetArmClient &other) = delete; // 可以禁用拷贝赋值

      // 构造函数
      explicit SetArmClient(std::shared_ptr<rclcpp::Client<rm_interfaces::srv::ArmData>> client)
          : ptr(std::move(client)) {}
    };
    // 新增：DataExchange服务客户端
    std::map<std::string, SetArmClient> arm_data_clients_;

    ReceivePacket latest_packet_;
    // 新增：服务服务器
    rclcpp::Service<rm_interfaces::srv::ArmData>::SharedPtr arm_data_server_;

    // void sendDataExchange(rm_interfaces::msg::GimbalCmd::SharedPtr msg);

    // 服务回调函数声明
    void handleArmData(
        const std::shared_ptr<rm_interfaces::srv::ArmData::Request> request,
        std::shared_ptr<rm_interfaces::srv::ArmData::Response> response);

    int vision_mode_ = 2;

  private:
    // Moveit接口
    // std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    void getParams();

    void receiveData();
    // 发送关节信息
    void sendData(const std::shared_ptr<rm_interfaces::srv::ArmData::Response> response);

    // void sendData(rm_interfaces::msg::GimbalCmd::SharedPtr msg);

    void reopenPort();

    // void processReceivedData(const ReceivePacket& packet, rm_interfaces::msg::GimbalCmd::SharedPtr msg);

    std::vector<double> planTrajectory(const geometry_msgs::msg::Pose &target_pose);

    // 串口
    std::unique_ptr<IoContext> owned_ctx_;
    std::string device_name_;
    std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
    std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

    // Param client to set detect_color
    // using ResultFuturePtr = std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;
    bool initial_set_param_ = false;
    uint8_t previous_receive_color_ = 0;
    rclcpp::AsyncParametersClient::SharedPtr detector_param_client_;
    // ResultFuturePtr set_param_future_;

    // 将 tf 从 odom 广播到 gimbal_link
    double timestamp_offset_ = 0;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Subscription<rm_interfaces::msg::GimbalCmd>::SharedPtr target_sub_;

    // 对于调试使用
    // rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr latency_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    std::thread receive_thread_;

    // 机械臂姿态
    geometry_msgs::msg::Pose start_pose_;   // 起始位姿
    geometry_msgs::msg::Pose current_pose_; // 当前位姿
    bool moveit_enabled_ = false;           // MoveIt启用标志
    // void setStartPose();  //设置起始位姿
    // void initMoveGroup(); //MoveGroup初始化
    std::mutex init_mutex_;                  // 新增：线程同步所需的互斥锁
    bool is_move_group_initialized_ = false; // 初始化标志位
    bool is_fully_constructed_ = false;
    std::once_flag init_flag_; // 确保只初始化一次
  };
} // namespace rm_serial_driver

#endif // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
