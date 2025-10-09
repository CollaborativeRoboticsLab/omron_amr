#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "std_msgs/msg/string.hpp"

/**
 * @brief Publishes joint states and transforms for the robot.
 */
class JointsPublisher
{
public:
  /**
   * @brief Constructor. Initializes node, publishers, and subscriptions.
   */
  JointsPublisher(rclcpp::Node::SharedPtr node)
  {
    node_ = node;
  }

  /**
   * @brief Initializes the publisher, subscriber, and transform broadcaster.
   * This should be called after the node is fully constructed.
   */
  void initialize()
  {
		topic_name_ = node_->declare_parameter<std::string>("joint_states.topic", "joint_states");
    frequency_ = node_->declare_parameter<int>("joint_states.frequency", 50);
    interval_ms_ = 1000 / frequency_;

    publisher = node_->create_publisher<sensor_msgs::msg::JointState>(topic_name_, 10);
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
    timer_ = node_->create_wall_timer(std::chrono::milliseconds(interval_ms_), std::bind(&JointsPublisher::run, this));

    // Joint state names
    joint_state.name = { MAIN_BODY_JOINT,     RIGHT_WHEEL_JOINT,    LEFT_WHEEL_JOINT,   RIGHT_FR_WHEEL_JOINT,
                         LEFT_FR_WHEEL_JOINT, RIGHT_BK_WHEEL_JOINT, LEFT_BK_WHEEL_JOINT };
    joint_state.position.resize(JOINTS_SIZE);

    pose_trans.header.frame_id = HEAD_FRAME;
    pose_trans.child_frame_id = CHILD_FRAME;
  }

  /**
   * @brief Callback for pose subscriber. Updates position and orientation.
   * @param pose_msg Status message containing location.
   */
  void update(const amr_msgs::msg::Status& pose_msg)
  {
    // Convert mm to meters
    pos_x = pose_msg.location.x / 1000.0;
    pos_y = pose_msg.location.y / 1000.0;
    pos_z = 0;

    // Convert degrees to radians
    double rad = pose_msg.location.theta;
    rad = rad * M_PI / 180;
    if (rad < 0)
      rad += (2 * M_PI);
    theta = createQuaternionMsgFromYaw(rad);
  }

  /**
   * @brief Main loop for publishing joint states and transforms.
   */
  void run()
  {
    // Update joints
    joint_state.header.stamp = node_->now();
    joint_state.position[0] = main_body_to_base_pos;
    joint_state.position[1] = right_wheel_joint_pos;
    joint_state.position[2] = left_wheel_joint_pos;
    joint_state.position[3] = right_front_small_wheel_joint_pos;
    joint_state.position[4] = left_front_small_wheel_joint_pos;
    joint_state.position[5] = left_back_small_wheel_joint_pos;
    joint_state.position[6] = right_back_small_wheel_joint_pos;

    // Update transform
    pose_trans.header.stamp = node_->now();
    pose_trans.transform.translation.x = pos_x;
    pose_trans.transform.translation.y = pos_y;
    pose_trans.transform.translation.z = pos_z;
    pose_trans.transform.rotation = theta;

    publisher->publish(joint_state);
    tf_broadcaster->sendTransform(pose_trans);
  }

private:
  /**
   * @brief Converts yaw angle to quaternion message.
   * @param yaw Yaw angle in radians.
   * @return Quaternion message.
   */
  geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw)
  {
    tf2::Quaternion tfq;
    tfq.setRPY(0, 0, yaw);
    geometry_msgs::msg::Quaternion gmq;

    double roll = 0;
    double pitch = 0;

    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    gmq.w = cy * cp * cr + sy * sp * sr;
    gmq.x = cy * cp * sr - sy * sp * cr;
    gmq.y = sy * cp * sr + cy * sp * cr;
    gmq.z = sy * cp * cr - cy * sp * sr;

    return gmq;
  }

  // ROS node and interfaces
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  int frequency_;
  int interval_ms_;

	std::string topic_name_;

  // Timer for periodic updates
  rclcpp::TimerBase::SharedPtr timer_;

  // Joint state and transform
  sensor_msgs::msg::JointState joint_state;
  geometry_msgs::msg::TransformStamped pose_trans;

  // Joint positions
  double main_body_to_base_pos = 0;
  double right_wheel_joint_pos = 0;
  double left_wheel_joint_pos = 0;
  double right_front_small_wheel_joint_pos = 0;
  double left_front_small_wheel_joint_pos = 0;
  double left_back_small_wheel_joint_pos = 0;
  double right_back_small_wheel_joint_pos = 0;

  // Robot pose
  double pos_x = 0;
  double pos_y = 0;
  double pos_z = 0;
  geometry_msgs::msg::Quaternion theta;

  // Constants
  static constexpr int JOINTS_SIZE = 7;
  static constexpr const char* HEAD_FRAME = "pose";
  static constexpr const char* CHILD_FRAME = "base_link";
  static constexpr const char* MAIN_BODY_JOINT = "main_body_to_base";
  static constexpr const char* RIGHT_WHEEL_JOINT = "right_wheel_joint";
  static constexpr const char* LEFT_WHEEL_JOINT = "left_wheel_joint";
  static constexpr const char* RIGHT_FR_WHEEL_JOINT = "right_front_small_wheel_joint";
  static constexpr const char* LEFT_FR_WHEEL_JOINT = "left_front_small_wheel_joint";
  static constexpr const char* RIGHT_BK_WHEEL_JOINT = "left_back_small_wheel_joint";
  static constexpr const char* LEFT_BK_WHEEL_JOINT = "right_back_small_wheel_joint";
};