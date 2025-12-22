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

#include "robot_polygon/robot_polygon.hpp"

namespace robot_polygon
{
RobotPolygon::RobotPolygon() : Node("robot_polygon")
{
     // 发布机器人近似多边形（用于 RViz 可视化）
    robot_poly_pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("robot_polygon", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&RobotPolygon::timer_callback, this));
}

RobotPolygon::~RobotPolygon() {}


void RobotPolygon::timer_callback()
{
     publish_robot_circle();
}

/*
  publish_robot_circle()
  - 功能：发布近似机器人边界的多边形，用于可视化与算法调试。
  - 说明：该多边形基于 robot_radius_ 构造 N 点圆近似，frame_id 设为 base_footprint。
*/
void RobotPolygon::publish_robot_circle()
{
    geometry_msgs::msg::PolygonStamped msg;
    msg.header.stamp = rclcpp::Time(0);
    msg.header.frame_id = "base_footprint";
    const int N = 36;
    for (int i = 0; i < N; ++i) {
        double theta = 2.0 * M_PI * i / N;
        geometry_msgs::msg::Point32 p;
        p.x = robot_radius_ * std::cos(theta);
        p.y = robot_radius_ * std::sin(theta);
        p.z = 0.0;
        msg.polygon.points.push_back(p);
    }
    robot_poly_pub_->publish(msg);
}




}  // namespace robot_polygon
