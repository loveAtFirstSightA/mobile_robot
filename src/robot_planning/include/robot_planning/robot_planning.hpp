/*
 Copyright 2025 Author lio

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

      https://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
 */

#ifndef ROBOT_PLANNING__ROBOT_PLANNING_HPP_
#define ROBOT_PLANNING__ROBOT_PLANNING_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "spdlog/spdlog.h"

namespace robot_planning
{
class RobotPlanning : public rclcpp::Node
{
public:
    RobotPlanning();
    ~RobotPlanning();

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber_;

    void map_subscriber_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    nav_msgs::msg::OccupancyGrid map_;



};
}  // namespace robot_planning
#endif  // ROBOT_PLANNING__ROBOT_PLANNING_HPP_
