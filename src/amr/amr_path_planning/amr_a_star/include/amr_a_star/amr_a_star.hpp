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

#ifndef AMR_A_STAR__AMR_A_STAR_HPP_
#define AMR_A_STAR__AMR_A_STAR_HPP_

#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "spdlog/spdlog.h"
#include "amr_msgs/srv/a_star.hpp"
#include "amr_msgs/msg/point.hpp"

namespace amr_a_star
{

class AmrAStar : public rclcpp::Node
{
public:
    AmrAStar();
    ~AmrAStar();

private:
    void a_star_server_callback(
        const std::shared_ptr<amr_msgs::srv::AStar::Request> request,
        std::shared_ptr<amr_msgs::srv::AStar::Response> response);

    rclcpp::Service<amr_msgs::srv::AStar>::SharedPtr a_star_server_;

private:
    // 点数据结构
    struct Point {
        double x;
        double y;
    };

    // 计算曼哈顿距离
    double manhattan(const Point &a, const Point &b) {
        return std::fabs(a.x - b.x) + std::fabs(a.y - b.y);
    }

    std::vector<Point> free_points_;

    struct AStarNode {
        Point p;
        double g;
        double h;
        double f;
        int parent;
    };

    bool samePoint(const Point &a, const Point &b) {
        return std::fabs(a.x - b.x) < 1e-6 && std::fabs(a.y - b.y) < 1e-6;
    }

};
}  // namespace amr_a_star
#endif  // AMR_A_STAR__AMR_A_STAR_HPP_
