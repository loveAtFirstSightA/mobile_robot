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

#include "robot_planning/robot_planning.hpp"
#include "robot_planning/a_star.hpp"

namespace robot_planning
{
RobotPlanning::RobotPlanning() : Node("robot_planning")
{
    map_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10, std::bind(&RobotPlanning::map_subscriber_callback, this, std::placeholders::_1));
}

RobotPlanning::~RobotPlanning() 
{
    // 
}


void RobotPlanning::map_subscriber_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    spdlog::info("Get new map");
    map_ = *msg;
}



}  // namespace robot_planning
