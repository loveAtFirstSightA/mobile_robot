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


#ifndef ROBOT_NODE__DIFF_DRIVE_CONTROLLER_HPP_
#define ROBOT_NODE__DIFF_DRIVE_CONTROLLER_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "robot_node/odometry.hpp"

namespace robot
{
class DiffDriveController : public rclcpp::Node
{
public:
    explicit DiffDriveController(const float wheel_seperation, const float wheel_radius);
    virtual ~DiffDriveController() {}

private:
    std::shared_ptr<rclcpp::Node> nh_;
    std::unique_ptr<Odometry> odometry_;
};
}  // namespace robot
#endif  // ROBOT_NODE__DIFF_DRIVE_CONTROLLER_HPP_
