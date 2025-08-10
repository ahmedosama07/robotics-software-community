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

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "temperature_monitor/temperature_subscriber.hpp"

class TestTemperatureSubscriber : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node = std::make_shared<TemperatureSubscriber>();
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

  std::shared_ptr<TemperatureSubscriber> node;
};

TEST_F(TestTemperatureSubscriber, TestInitialization)
{
  EXPECT_STREQ(node->get_name(), "temperature_subscriber");
}

TEST_F(TestTemperatureSubscriber, TestTemperatureWarnings)
{
  // Test low temperature warning
  node->check_temperature_warnings(5.0, "test_sensor");
  // Should trigger low temperature warning

  // Test high temperature warning
  node->check_temperature_warnings(40.0, "test_sensor");
  // Should trigger high temperature warning

  // Test normal temperature (no warning)
  node->check_temperature_warnings(25.0, "test_sensor");
  // Should not trigger any warnings
}

TEST_F(TestTemperatureSubscriber, TestMovingAverageCalculation)
{
  // Test with empty history
  EXPECT_DOUBLE_EQ(node->calculate_moving_average(), 0.0);

  // Add test temperatures
  std::vector<double> test_temps = {20.0, 22.0, 24.0, 26.0, 28.0};
  for (double temp : test_temps) {
    auto msg = std::make_shared<TemperatureReading>();
    msg->temperature = temp;
    node->temperature_callback(msg);
  }

  // Calculate expected average
  double expected_avg = std::accumulate(
    test_temps.begin(),
    test_temps.end(), 0.0) / test_temps.size();
  EXPECT_NEAR(node->calculate_moving_average(), expected_avg, 0.01);
}

TEST_F(TestTemperatureSubscriber, TestTemperatureTrend)
{
  // Test with insufficient data
  EXPECT_EQ(node->calculate_trend(), "Insufficient data");

  // Add increasing temperatures
  std::vector<double> increasing_temps = {20.0, 22.0, 24.0, 26.0, 28.0};
  for (double temp : increasing_temps) {
    auto msg = std::make_shared<TemperatureReading>();
    msg->temperature = temp;
    node->temperature_callback(msg);
  }

  EXPECT_EQ(node->calculate_trend(), "Rising");

  // Add decreasing temperatures
  std::vector<double> decreasing_temps = {28.0, 26.0, 24.0, 22.0, 20.0};
  for (double temp : decreasing_temps) {
    auto msg = std::make_shared<TemperatureReading>();
    msg->temperature = temp;
    node->temperature_callback(msg);
  }

  EXPECT_EQ(node->calculate_trend(), "Falling");
}

TEST_F(TestTemperatureSubscriber, TestEmptyHistoryEdgeCases) {
  EXPECT_DOUBLE_EQ(node->calculate_moving_average(), 0.0);
  EXPECT_EQ(node->calculate_trend(), "Insufficient data");
}

TEST_F(TestTemperatureSubscriber, TestHighFrequencyUpdates) {
  // Simulate 50 messages
  for (int i = 0; i < 50; ++i) {
    auto msg = std::make_shared<TemperatureReading>();
    msg->temperature = 20.0 + (i % 5);
    node->temperature_callback(msg);
  }

  // Verify history capped at 10
  EXPECT_GE(node->get_temperature_history().size(), 10);
  EXPECT_LE(node->get_temperature_history().size(), 10);    // Should not grow beyond 10
}

TEST_F(TestTemperatureSubscriber, TestInvalidMessageValues) {
  auto msg = std::make_shared<TemperatureReading>();
  msg->temperature = std::numeric_limits<double>::quiet_NaN();
  msg->sensor_id = "sensor_nan";

  EXPECT_NO_THROW(node->temperature_callback(msg));
}
