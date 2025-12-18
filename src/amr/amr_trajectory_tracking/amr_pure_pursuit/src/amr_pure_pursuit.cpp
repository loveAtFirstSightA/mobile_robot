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

#include "amr_pure_pursuit/amr_pure_pursuit.hpp"

namespace amr_pure_pursuit
{

AmrPurePursuit::AmrPurePursuit() : Node("amr_pure_pursuit")
{
     pure_pursuit_ser_ = this->create_service<amr_msgs::srv::PurePursuit>(
          "pure_pursuit_server",
          std::bind(&AmrPurePursuit::pure_pursuit_ser_callback, this,
                    std::placeholders::_1, std::placeholders::_2));
}

AmrPurePursuit::~AmrPurePursuit() {}

void AmrPurePursuit::pure_pursuit_ser_callback(
     const std::shared_ptr<amr_msgs::srv::PurePursuit::Request> request,
     std::shared_ptr<amr_msgs::srv::PurePursuit::Response> response)
{
     double current_v = request->v;
     
     double current_x = request->pose.x;
     double current_y = request->pose.y;
     double current_yaw = request->pose.yaw;
     
     double line_s_x = request->line.sx;
     double line_s_y = request->line.sy;
     double line_e_x = request->line.ex;
     double line_e_y = request->line.ey;

     // 添加纯跟踪算法
     // 计算前视距离
     double k = 0.3;
     double min_ld = 0.3;   // 最小前视距离，避免低速时角度抖动
     double max_ld = 2.0;   // 最大前视距离，避免高速时前视点过远
     double lookaheaddist = std::clamp(k * current_v, min_ld, max_ld);
     // spdlog::info("pose: [{:.3f}, {:.3f}, {:.3f}], line: [{:.3f}, {:.3f}], [{:.3f}, {:.3f}], lookaheaddist: {:.3f}",
     //      current_x, current_y, current_yaw, line_s_x, line_s_y, line_e_x, line_e_y, lookaheaddist);

     // Step 1 基于前视距离和当前位置计算不同类型曲线的目标点，当前是直线
     // Step 1-1 计算最近点
     double closest_x, closest_y;
     double vector_x, vector_y; // 直线起点至当前位置的向量
     double vector_line_x, vector_line_y; // 直线起点至直线终点的向量
     
     vector_x = current_x - line_s_x;
     vector_y = current_y - line_s_y;
     vector_line_x = line_e_x - line_s_x;
     vector_line_y = line_e_y - line_s_y;
     double line_length_square = std::pow(vector_line_x, 2) + std::pow(vector_line_y, 2);
     double t = (vector_x * vector_line_x + vector_y * vector_line_y) / line_length_square;
     if (t > 1.0f) {
          t = 1.0f;
     } else if (t < 0.0f) {
          t = 0.0f;
     }
     // spdlog::info("t: {:.3f}", t);
     closest_x = line_s_x + t * vector_line_x;
     closest_y = line_s_y + t * vector_line_y;
     // spdlog::info("closest: [{:.3f}, {:.3f}]", closest_x, closest_y);

     // Step 1-2 计算单位向量
     double line_length = std::sqrt(line_length_square);
     double unit_vector_line_x = vector_line_x / line_length;
     double unit_vector_line_y = vector_line_y / line_length;

     // Step 1-3 当前点到直线的最近的点 + 前视距离 * 单位向量
     double target_x = closest_x + lookaheaddist * unit_vector_line_x;
     double target_y = closest_y + lookaheaddist * unit_vector_line_y;
     // spdlog::info("target: [{:.3f}, {:.3f}]", target_x, target_y);

     // Step 2 根据当前点 目标点 当前角度 确定角度alpha
     double ld_vx = target_x - current_x;
     double ld_vy = target_y - current_y;
     double ld_theta = std::atan2(ld_vy, ld_vx);
     double alpha  = ld_theta - current_yaw;
     alpha = std::atan2(std::sin(alpha), std::cos(alpha));
     // spdlog::info("ld_theta: {:.3f}, current_yaw: {:.3f}, alpha: {:.3f}", ld_theta, current_yaw, alpha);

     // Step 3 差速类型的模型计算旋转半径R
     double r = lookaheaddist / (2.0f * std::sin(alpha));
     // spdlog::info("r: {:.3f}", r);
     // Step 4 v = w * r
     double w;
     if (std::abs(std::sin(alpha)) < 1e-6) {
          w = 0;
     }
     else {
          w = current_v / r; 
     }
     // w = current_v / r;

     response->velocity.v = current_v;
     response->velocity.w = w;
     // spdlog::info("velocity: [{:.3f}, {:.3f}]", current_v, w);
     // spdlog::info("");
}

}  // namespace amr_pure_pursuit
