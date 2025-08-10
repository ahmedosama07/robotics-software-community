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

#ifndef TEMPERATURE_MONITOR__TEMPERATURE_SUBSCRIBER_HPP_
#define TEMPERATURE_MONITOR__TEMPERATURE_SUBSCRIBER_HPP_

#include <deque>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "temperature_interfaces/msg/temperature_reading.hpp"

using TemperatureReading = temperature_interfaces::msg::TemperatureReading;

class TemperatureSubscriber : public rclcpp::Node
{
public:
  TemperatureSubscriber();

  double calculate_moving_average();
  std::string calculate_trend();
  void check_temperature_warnings(double temperature, const std::string & sensor_id);
  void temperature_callback(const TemperatureReading::SharedPtr msg);
  void display_statistics();

  // Public getters
  const std::deque<double> & get_temperature_history() const {return temperature_history_;}
  double get_min_temp() const {return min_temp_;}
  double get_max_temp() const {return max_temp_;}

private:
  // Member variables
  rclcpp::Subscription<TemperatureReading>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr stats_timer_;

  std::deque<double> temperature_history_;
  int message_count_;
  double min_temp_;
  double max_temp_;
  double temp_warning_low_;
  double temp_warning_high_;
};

#endif  // TEMPERATURE_MONITOR__TEMPERATURE_SUBSCRIBER_HPP_
