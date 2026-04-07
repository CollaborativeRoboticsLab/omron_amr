#pragma once

#include <ArNetworking/ArNetworking.h>

#include <amr_msgs/msg/location.hpp>
#include <amr_msgs/msg/status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include "amr_core/utils/libaria_runtime.hpp"

class StatusInterface
{
public:
	explicit StatusInterface(rclcpp::Node::SharedPtr node)
	: node_(std::move(node))
	, update_numbers_callback_(this, &StatusInterface::handleUpdateNumbers)
	, update_strings_callback_(this, &StatusInterface::handleUpdateStrings)
	{
	}

	~StatusInterface()
	{
		client_.disconnect();
	}

	void initialize()
	{
		host_ = getOrDeclareParameter<std::string>("robot.ip", "127.0.0.1");
		port_ = getOrDeclareParameter<int>("robot.port", 7272);
		user_ = getOrDeclareParameter<std::string>("robot.user", "admin");
		password_ = getOrDeclareParameter<std::string>("robot.password", "");
		protocol_ = getOrDeclareParameter<std::string>("robot.protocol", "6MTX");
		topic_name_ = getOrDeclareParameter<std::string>("status.topic", "amr/source/status");
		battery_topic_name_ = getOrDeclareParameter<std::string>("status.battery_topic", "battery_state");
		publish_period_ms_ = getOrDeclareParameter<int>("status.publish_period_ms", 200);

		status_pub_ = node_->create_publisher<amr_msgs::msg::Status>(topic_name_, 10);
		battery_pub_ = node_->create_publisher<sensor_msgs::msg::BatteryState>(battery_topic_name_, 10);

		connectClient();
		configureHandlers();
		client_.runAsync();

		publish_timer_ = node_->create_wall_timer(
			std::chrono::milliseconds(publish_period_ms_), std::bind(&StatusInterface::publishStatus, this));
	}

private:
	template<typename T>
	T getOrDeclareParameter(const std::string &name, const T &default_value)
	{
		if (!node_->has_parameter(name))
		{
			return node_->declare_parameter<T>(name, default_value);
		}

		T value = default_value;
		node_->get_parameter(name, value);
		return value;
	}

	void connectClient()
	{
		if (!protocol_.empty())
		{
			client_.enforceProtocolVersion(protocol_.c_str());
		}

		const char *password = password_.empty() ? nullptr : password_.c_str();
		if (!client_.blockingConnect(host_.c_str(), port_, true, user_.c_str(), password))
		{
			if (client_.wasRejected())
			{
				throw std::runtime_error("Robot rejected the status interface connection.");
			}
			throw std::runtime_error("Could not connect status interface to Omron robot server.");
		}

		client_.setRobotName(host_.c_str());
		RCLCPP_INFO(node_->get_logger(), "StatusInterface connected to %s:%d using libaria", host_.c_str(), port_);
	}

	void configureHandlers()
	{
		if (client_.dataExists("updateNumbers"))
		{
			client_.addHandler("updateNumbers", &update_numbers_callback_);
			client_.request("updateNumbers", 50);
		}
		else
		{
			RCLCPP_WARN(node_->get_logger(), "Server does not advertise updateNumbers; status pose fields will stay default");
		}

		if (client_.dataExists("updateStrings"))
		{
			client_.addHandler("updateStrings", &update_strings_callback_);
			client_.request("updateStrings", -1);
		}
		else
		{
			RCLCPP_WARN(node_->get_logger(), "Server does not advertise updateStrings; status text fields will stay default");
		}
	}

	void handleUpdateNumbers(ArNetPacket *packet)
	{
		const double state_of_charge = static_cast<double>(packet->bufToByte2()) / 10.0;
		const double x = static_cast<double>(packet->bufToByte4());
		const double y = static_cast<double>(packet->bufToByte4());
		const double theta = static_cast<double>(packet->bufToByte2());
		packet->bufToByte2();
		packet->bufToByte2();
		packet->bufToByte2();
		packet->bufToByte();

		std::lock_guard<std::mutex> lock(status_mutex_);
		latest_status_.state_of_charge = static_cast<float>(state_of_charge);
		latest_status_.location.x = x;
		latest_status_.location.y = y;
		latest_status_.location.theta = theta;
	}

	void handleUpdateStrings(ArNetPacket *packet)
	{
		char status_buffer[256] = { 0 };
		char extended_buffer[256] = { 0 };
		packet->bufToStr(status_buffer, sizeof(status_buffer));
		packet->bufToStr(extended_buffer, sizeof(extended_buffer));

		std::lock_guard<std::mutex> lock(status_mutex_);
		latest_status_.status = status_buffer;
		latest_status_.extended_status = extended_buffer;
	}

	void publishStatus()
	{
		if (!status_pub_)
		{
			return;
		}

		amr_msgs::msg::Status msg;
		{
			std::lock_guard<std::mutex> lock(status_mutex_);
			msg = latest_status_;
		}
		status_pub_->publish(msg);

		if (battery_pub_)
		{
			sensor_msgs::msg::BatteryState battery_msg;
			battery_msg.header.stamp = node_->now();
			battery_msg.present = true;
			battery_msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
			battery_msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
			battery_msg.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
			battery_msg.voltage = std::numeric_limits<float>::quiet_NaN();
			battery_msg.temperature = msg.temperature;
			battery_msg.current = std::numeric_limits<float>::quiet_NaN();
			battery_msg.charge = std::numeric_limits<float>::quiet_NaN();
			battery_msg.capacity = std::numeric_limits<float>::quiet_NaN();
			battery_msg.design_capacity = std::numeric_limits<float>::quiet_NaN();
			battery_msg.percentage = std::numeric_limits<float>::quiet_NaN();

			if (std::isfinite(msg.state_of_charge) && msg.state_of_charge >= 0.0f)
			{
				battery_msg.percentage = std::clamp(msg.state_of_charge / 100.0f, 0.0f, 1.0f);
			}

			battery_pub_->publish(battery_msg);
		}
	}

	rclcpp::Node::SharedPtr node_;
	amr_core::LibAriaRuntime aria_runtime_;
	ArClientBase client_;
	ArFunctor1C<StatusInterface, ArNetPacket *> update_numbers_callback_;
	ArFunctor1C<StatusInterface, ArNetPacket *> update_strings_callback_;

	std::mutex status_mutex_;
	amr_msgs::msg::Status latest_status_;

	std::string host_;
	int port_{ 7272 };
	std::string user_;
	std::string password_;
	std::string protocol_;
	std::string topic_name_{ "amr/source/status" };
	std::string battery_topic_name_{ "battery_state" };
	int publish_period_ms_{ 200 };

	rclcpp::Publisher<amr_msgs::msg::Status>::SharedPtr status_pub_;
	rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
	rclcpp::TimerBase::SharedPtr publish_timer_;
};
