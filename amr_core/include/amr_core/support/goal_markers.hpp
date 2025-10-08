#pragma once

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include <string>
#include <vector>
#include <memory>
#include <sstream>
#include "amr_core/socket/socket_driver.hpp"
/**
 * @brief Header-only node for visualizing goals and goal texts in RViz.
 */
class GoalMarkers
{
public:
  /**
   * @brief Constructor.
   */
  GoalMarkers(rclcpp::Node::SharedPtr node, std::shared_ptr<SocketDriver> driver)
  {
    node_ = node;
    driver_ = driver;
  }

  /**
   * @brief Initializes the GoalMarkers node by declaring parameters, setting up publishers and subscriptions, and
   * starting the main loop timer. This should be called after the node is fully constructed.
   */

  void initialize()
  {
    using namespace std::chrono_literals;

    // Declare and get parameters
    head_frame = node_->declare_parameter<std::string>("goal_markers.head_frame", "pose");
    goals_vis_topic = node_->declare_parameter<std::string>("goal_markers.topic", "goal_markers");
    goals_a_clr = node_->declare_parameter<double>("goal_markers.goals_a_colour", 1.0);
    goals_r_clr = node_->declare_parameter<double>("goal_markers.goals_r_colour", 0.0);
    goals_g_clr = node_->declare_parameter<double>("goal_markers.goals_g_colour", 1.0);
    goals_b_clr = node_->declare_parameter<double>("goal_markers.goals_b_colour", 0.0);
    goal_texts_a_clr = node_->declare_parameter<double>("goal_markers.goal_texts_a_colour", 1.0);
    goal_texts_r_clr = node_->declare_parameter<double>("goal_markers.goal_texts_r_colour", 0.0);
    goal_texts_g_clr = node_->declare_parameter<double>("goal_markers.goal_texts_g_colour", 1.0);
    goal_texts_b_clr = node_->declare_parameter<double>("goal_markers.goal_texts_b_colour", 0.0);

    goals_pub = node_->create_publisher<visualization_msgs::msg::MarkerArray>(goals_vis_topic, 10);

    // Initialize marker templates
    init_goal_marker();
    init_goal_text_marker();
  }

  void update(const std_msgs::msg::String& msg, int single_goal_timeout_ms)
  {
    goals_list.clear();
    std::istringstream iss(msg.data);
    std::string name;
    while (iss >> name)
    {
      goals_list.push_back(name);
    }

    visualization_msgs::msg::MarkerArray goals;
    visualization_msgs::msg::MarkerArray goals_text;
    goals_id = 0;
    goal_texts_id = 0;

    for (const auto& goal_name : goals_list)
    {
      std::string response;
      bool got_response;
      int req_id = driver_->queue_command(GOAL_CMD + goal_name, "End of MapObjectInfo");
      got_response = driver_->wait_for_response(req_id, response, single_goal_timeout_ms);

      std::string::size_type pos = response.find(COORD_H);

      if (pos != std::string::npos)
      {
        std::string::size_type end = response.find("\r\n", pos);
        std::string val_str = response.substr(pos + COORD_H.size(), end - (pos + COORD_H.size()));
        std::istringstream val_iss(val_str);
        std::string dummy;
        double x, y, theta;
        if (!(val_iss >> dummy >> x >> y >> theta))
        {
          RCLCPP_ERROR(node_->get_logger(), "Error reading goal coordinates");
          continue;
        }
        x /= 1000.0;
        y /= 1000.0;
        one_goal.pose.position.x = x;
        one_goal.pose.position.y = y;
        one_goal.pose.position.z = 0;
        if (theta < 0)
          theta += 360.0;
        theta = theta * 0.01745329252;
        one_goal.pose.orientation = createQuaternionMsgFromYaw(theta);
        one_goal.id = goals_id++;
        goals.markers.push_back(one_goal);

        one_goal_text.pose.position.x = x;
        one_goal_text.pose.position.y = y;
        one_goal_text.text = goal_name;
        one_goal_text.id = goal_texts_id++;
        goals_text.markers.push_back(one_goal_text);
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), "Goal coordinates not found in response");
      }
    }

    for (auto& marker : goals.markers)
    {
      marker.header.stamp = node_->now();
    }
    goals_pub->publish(goals);
    goals_pub->publish(goals_text);
  }

private:
  /**
   * @brief Initializes the goal marker template.
   */
  void init_goal_marker()
  {
    one_goal.header.frame_id = head_frame;
    one_goal.ns = GOALS_NS;
    one_goal.action = visualization_msgs::msg::Marker::MODIFY;
    one_goal.id = goals_id;
    one_goal.type = visualization_msgs::msg::Marker::ARROW;
    one_goal.scale.x = GOALS_X_SCALE;
    one_goal.scale.y = GOALS_Y_SCALE;
    one_goal.scale.z = GOALS_Z_SCALE;
    one_goal.color.a = goals_a_clr;
    one_goal.color.r = goals_r_clr;
    one_goal.color.g = goals_g_clr;
    one_goal.color.b = goals_b_clr;
  }

  /**
   * @brief Initializes the goal text marker template.
   */
  void init_goal_text_marker()
  {
    one_goal_text.header.frame_id = head_frame;
    one_goal_text.ns = GOALS_TEXT_NS;
    one_goal_text.action = visualization_msgs::msg::Marker::MODIFY;
    one_goal_text.pose.orientation.w = 1.0;
    one_goal_text.id = goal_texts_id;
    one_goal_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    one_goal_text.pose.position.z = GOAL_TEXT_HEIGHT_Z;
    one_goal_text.scale.z = GOAL_TEXT_Z_SCALE;
    one_goal_text.color.a = goal_texts_a_clr;
    one_goal_text.color.r = goal_texts_r_clr;
    one_goal_text.color.g = goal_texts_g_clr;
    one_goal_text.color.b = goal_texts_b_clr;
  }

  /**
   * @brief Utility to create a quaternion from yaw.
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

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<SocketDriver> driver_;

  // Configurable values
  std::string head_frame;
  std::string goals_vis_topic;
  float goals_a_clr;
  float goals_r_clr;
  float goals_g_clr;
  float goals_b_clr;
  float goal_texts_a_clr;
  float goal_texts_r_clr;
  float goal_texts_g_clr;
  float goal_texts_b_clr;

  // Non configurable values
  const std::string GOALS_NS = "goals";
  const std::string GOALS_TEXT_NS = "goal_texts";
  const std::string GOAL_RESP_H = "Goal";
  const std::string COORD_H = "MapObjectInfoCoord: ";
  const std::string GOAL_CMD = "MapObjectInfo ";
  const std::string GOALS_LIST_END = "End of MapObjectInfo";
  const double GOALS_X_SCALE = 0.5;
  const double GOALS_Y_SCALE = 0.1;
  const double GOALS_Z_SCALE = 0.07;
  const double GOAL_TEXT_Z_SCALE = 0.5;
  const double GOAL_TEXT_HEIGHT_Z = 0.5;

  // ROS interfaces
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr goals_pub;

  // Marker templates
  visualization_msgs::msg::Marker one_goal;
  visualization_msgs::msg::Marker one_goal_text;

  // State
  std::vector<std::string> goals_list;
  int32_t goals_id = 0;
  int32_t goal_texts_id = 0;
};