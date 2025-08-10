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
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<TemperatureSubscriber>();

    RCLCPP_INFO(node->get_logger(), "Starting temperature subscriber...");

    // Handle graceful shutdown
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("temperature_subscriber"),
      "Exception in temperature subscriber: %s", e.what());
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
