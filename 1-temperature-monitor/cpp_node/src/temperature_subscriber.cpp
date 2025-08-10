// Copyright 2025 Ahmed Osama
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "temperature_monitor/temperature_subscriber.hpp"
#include <numeric>
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <limits>

using namespace std::chrono_literals;

TemperatureSubscriber::TemperatureSubscriber()
: Node("temperature_subscriber"),
  message_count_(0),
  min_temp_(std::numeric_limits<double>::max()),
  max_temp_(std::numeric_limits<double>::lowest())
{
  // Set up QoS profile for reliable communication
  auto qos_profile = rclcpp::QoS(10)
    .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
    .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST)
    .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

  // Create subscription
  subscription_ = this->create_subscription<TemperatureReading>(
    "temperature_data",
    qos_profile,
    std::bind(&TemperatureSubscriber::temperature_callback, this, std::placeholders::_1)
  );

  // Create timer for periodic statistics display
  stats_timer_ = this->create_wall_timer(
    5s, std::bind(&TemperatureSubscriber::display_statistics, this)
  );

  // Initialize temperature thresholds for warning system
  temp_warning_low_ = 10.0;   // Warning below 10°C
  temp_warning_high_ = 35.0;  // Warning above 35°C

  RCLCPP_INFO(this->get_logger(), "Temperature Subscriber initialized");
  RCLCPP_INFO(
    this->get_logger(), "Warning thresholds: %.1f°C - %.1f°C",
    temp_warning_low_, temp_warning_high_);
  RCLCPP_INFO(this->get_logger(), "Waiting for temperature data...");
}

void TemperatureSubscriber::temperature_callback(const TemperatureReading::SharedPtr msg)
{
  try {
    if (std::isnan(msg->temperature)) {
      RCLCPP_WARN(
        this->get_logger(), "⚠️  Invalid temperature reading from sensor %s: NaN detected",
        msg->sensor_id.c_str());
      return;
    }
    if (std::isinf(msg->temperature)) {
      RCLCPP_WARN(
        this->get_logger(), "⚠️  Invalid temperature reading from sensor %s: Infinite value (%s)",
        msg->sensor_id.c_str(), msg->temperature > 0 ? "inf" : "-inf");
      return;
    }

    message_count_++;

    // Store temperature reading
    temperature_history_.push_back(msg->temperature);

    // Keep only last 10 readings for moving average
    if (temperature_history_.size() > 10) {
      temperature_history_.pop_front();
    }

    // Update min/max temperatures
    min_temp_ = std::min(min_temp_, msg->temperature);
    max_temp_ = std::max(max_temp_, msg->temperature);

    // Calculate moving average
    double moving_avg = calculate_moving_average();

    // Determine temperature trend
    std::string trend = calculate_trend();

    // Check for temperature warnings
    check_temperature_warnings(msg->temperature, msg->sensor_id);

    // Log every 5th message to avoid spam
    if (message_count_ % 5 == 0) {
      RCLCPP_INFO(
        this->get_logger(),
        "Reading #%d from %s: %.2f°C (%.1f%% humidity) | "
        "Avg: %.2f°C | Min: %.2f°C | Max: %.2f°C | Trend: %s",
        message_count_, msg->sensor_id.c_str(), msg->temperature,
        msg->humidity, moving_avg, min_temp_, max_temp_, trend.c_str());
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error processing temperature data: %s", e.what());
  }
}

double TemperatureSubscriber::calculate_moving_average()
{
  if (temperature_history_.empty()) {
    return 0.0;
  }

  double sum = std::accumulate(temperature_history_.begin(), temperature_history_.end(), 0.0);
  return sum / temperature_history_.size();
}

std::string TemperatureSubscriber::calculate_trend()
{
  if (temperature_history_.size() < 3) {
    return "Insufficient data";
  }

  // Compare recent readings to determine trend
  size_t size = temperature_history_.size();
  double recent_avg = 0.0, older_avg = 0.0;
  int recent_count = 0, older_count = 0;

  // Calculate average of last 3 readings
  for (size_t i = size - 1; i >= size - 3 && i < size; --i) {
    recent_avg += temperature_history_[i];
    recent_count++;
  }
  recent_avg /= recent_count;

  // Calculate average of previous readings (up to 3)
  size_t start = (size >= 6) ? size - 6 : 0;
  size_t end = size - 3;
  for (size_t i = start; i < end; ++i) {
    older_avg += temperature_history_[i];
    older_count++;
  }
  if (older_count > 0) {
    older_avg /= older_count;
  }

  const double threshold = 0.5;   // Temperature change threshold in Celsius

  if (recent_avg > older_avg + threshold) {
    return "Rising";
  } else if (recent_avg < older_avg - threshold) {
    return "Falling";
  } else {
    return "Stable";
  }
}

void
TemperatureSubscriber::check_temperature_warnings(double temperature, const std::string & sensor_id)
{
  if (temperature < temp_warning_low_) {
    RCLCPP_WARN(
      this->get_logger(),
      "⚠️  LOW TEMPERATURE WARNING: Sensor %s reports %.2f°C (below %.1f°C)",
      sensor_id.c_str(), temperature, temp_warning_low_);
  } else if (temperature > temp_warning_high_) {
    RCLCPP_WARN(
      this->get_logger(),
      "⚠️  HIGH TEMPERATURE WARNING: Sensor %s reports %.2f°C (above %.1f°C)",
      sensor_id.c_str(), temperature, temp_warning_high_);
  }
}

void TemperatureSubscriber::display_statistics()
{
  if (message_count_ == 0) {
    RCLCPP_INFO(this->get_logger(), "No temperature data received yet...");
    return;
  }

  double moving_avg = calculate_moving_average();
  std::string trend = calculate_trend();

  RCLCPP_INFO(this->get_logger(), "=== TEMPERATURE STATISTICS ===");
  RCLCPP_INFO(this->get_logger(), "Messages received: %d", message_count_);
  RCLCPP_INFO(
    this->get_logger(), "Current moving average (last %zu readings): %.2f°C",
    temperature_history_.size(), moving_avg);
  RCLCPP_INFO(this->get_logger(), "Temperature range: %.2f°C - %.2f°C", min_temp_, max_temp_);
  RCLCPP_INFO(this->get_logger(), "Current trend: %s", trend.c_str());
  RCLCPP_INFO(this->get_logger(), "==============================");
}
