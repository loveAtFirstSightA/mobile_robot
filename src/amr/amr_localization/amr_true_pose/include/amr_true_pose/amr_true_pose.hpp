/*
 Copyright 2025 Author lio

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

      https://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing,
 software distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
 */

#ifndef AMR_TRUE_POSE__AMR_TRUE_POSE_HPP_
#define AMR_TRUE_POSE__AMR_TRUE_POSE_HPP_

#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "spdlog/spdlog.h"
#include "nav_msgs/msg/odometry.hpp"
#include "amr_msgs/msg/pose.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace amr_true_pose
{
class AmrTruePose : public rclcpp::Node
{
public:
    AmrTruePose();
    ~AmrTruePose();

private:
    static constexpr int ROBOT_COUNT = 20;

    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> subs_;
    std::vector<rclcpp::Publisher<amr_msgs::msg::Pose>::SharedPtr> pubs_;
};
}  // namespace amr_true_pose

#endif  // AMR_TRUE_POSE__AMR_TRUE_POSE_HPP_
