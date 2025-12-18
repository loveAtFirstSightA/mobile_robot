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

#ifndef AMR_MOTION_CONTROL__AMR_MOTION_CONTROL_HPP_
#define AMR_MOTION_CONTROL__AMR_MOTION_CONTROL_HPP_

#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "spdlog/spdlog.h"
#include "amr_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "amr_msgs/srv/pure_pursuit.hpp"
#include "amr_msgs/msg/points.hpp"
#include "amr_msgs/msg/paths.hpp"
#include "amr_msgs/msg/amr_state.hpp"

namespace amr_motion_control
{
class AmrMotionControl : public rclcpp::Node
{
public:
    AmrMotionControl();
    ~AmrMotionControl();

private:
    static constexpr int NUM_AMR = 20;
    double v_max_ = 0.75f;
    double w_max_ = 0.8f;
    double a_max_ = 0.4f;

    // robot pose
    std::vector<amr_msgs::msg::Pose> poses_;
    std::vector<rclcpp::Subscription<amr_msgs::msg::Pose>::SharedPtr> pose_subs_;

    // Laser
    std::vector<sensor_msgs::msg::LaserScan> lasers_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> laser_subs_;

    // 接受路径点
    std::vector<amr_msgs::msg::Points> points_;
    std::vector<rclcpp::Subscription<amr_msgs::msg::Points>::SharedPtr> points_subs_;

    // 路径点转换成路径信息
    std::vector<amr_msgs::msg::Paths> paths_;

    // Velocity publisher
    std::vector<rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> vel_pubs_;

    // states
    std::vector<amr_msgs::msg::AmrState> amr_states_;
    // Publisher amr state
    std::vector<rclcpp::Publisher<amr_msgs::msg::AmrState>::SharedPtr> amr_state_pubs_;

    // Timer of motion control
    std::vector<rclcpp::TimerBase::SharedPtr> motion_control_timers_;
    // Pure Pursuit client
    std::vector<rclcpp::Client<amr_msgs::srv::PurePursuit>::SharedPtr> pure_pursuit_cli_;

    // timer feedback state
    std::vector<rclcpp::TimerBase::SharedPtr> feedback_state_timers_;
    
    // 障碍检测阈值
    double obstacle_threshold_ = 0.95f;

    // 通用函数
    void pose_callback(const amr_msgs::msg::Pose::SharedPtr msg, int amr_id);
    void points_callback(const amr_msgs::msg::Points::SharedPtr msg, int amr_id);
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg, int amr_id);

    void motion_control_timers_callback(int amr_id);
    void feedback_state_timers_callback(int amr_id);

    using ServiceResponseFuture = rclcpp::Client<amr_msgs::srv::PurePursuit>::SharedFuture;
    void pure_pursuit_cli_response_callback(ServiceResponseFuture future, int amr_id);

    void send_velocity(int amr_id, double v, double w);
    bool is_obstacle_ahead(int amr_id);

    std::vector<int> path_index_;
};
}  // namespace amr_motion_control
#endif  // AMR_MOTION_CONTROL__AMR_MOTION_CONTROL_HPP_

