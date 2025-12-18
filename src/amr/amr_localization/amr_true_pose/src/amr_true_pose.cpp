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

#include "amr_true_pose/amr_true_pose.hpp"

namespace amr_true_pose
{

AmrTruePose::AmrTruePose() : Node("amr_true_pose")
{
    subs_.resize(ROBOT_COUNT);
    pubs_.resize(ROBOT_COUNT);

    for (int i = 0; i < ROBOT_COUNT; i++) {
        std::string topic_in = "/amr_" + std::to_string(i) + "/p3d_ground_truth";
        std::string topic_out = "/amr_" + std::to_string(i) + "_pose";

        pubs_[i] = this->create_publisher<amr_msgs::msg::Pose>(topic_out, 10);

        subs_[i] = this->create_subscription<nav_msgs::msg::Odometry>(
            topic_in, 10,
            [this, i](const nav_msgs::msg::Odometry::SharedPtr msg) {
                amr_msgs::msg::Pose pose;
                pose.x = msg->pose.pose.position.x;
                pose.y = msg->pose.pose.position.y;
                pose.yaw = tf2::getYaw(msg->pose.pose.orientation);

                pubs_[i]->publish(pose);

                // 如果需要调试日志，可以打开
                // spdlog::info("amr_{}_pose: [{:.3f}, {:.3f}, {:.3f}]",
                //     i, pose.x, pose.y, pose.yaw);
            }
        );
    }
}

AmrTruePose::~AmrTruePose() {}

}  // namespace amr_true_pose
