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

#include "amr_path_planning/amr_path_planning.hpp"

namespace amr_path_planning
{

AmrPathPlanning::AmrPathPlanning() : Node("amr_path_planning")
{
    init_free_points();

    points_.resize(NUM_AMR);
    points_pubs_.resize(NUM_AMR);

    amr_states_.resize(NUM_AMR);
    amr_states_subs_.resize(NUM_AMR);

    a_star_cli_.resize(NUM_AMR);

    for (int i = 0; i < NUM_AMR; ++i)
    {
        // Path points 发布
        points_pubs_[i] = this->create_publisher<amr_msgs::msg::Points>(
            "/amr_" + std::to_string(i) + "_points", 10);

        // 机器人状态 订阅
        amr_states_subs_[i] = this->create_subscription<amr_msgs::msg::AmrState>(
            "/amr_" + std::to_string(i) + "_states", 10,
            [this, i](const amr_msgs::msg::AmrState::SharedPtr msg){ amr_states_subs_callback(msg, i); });
        
        // A Star 客户端（共用同一个服务名）
        a_star_cli_[i] = this->create_client<amr_msgs::srv::AStar>("amr_a_star");
    }

    task_sub_ = this->create_subscription<amr_msgs::msg::TaskPoint>(
        "task", 10, std::bind(&AmrPathPlanning::task_sub_callback, this, std::placeholders::_1));
    
    unit_test_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&AmrPathPlanning::unit_test_timer_callback, this));
    
    unit_test_timer_->cancel();
}

AmrPathPlanning::~AmrPathPlanning() {}

void AmrPathPlanning::unit_test_timer_callback()
{
    for (size_t i = 0; i < 1; i++)
    {
        if (amr_states_[i].state == "idle") {
            auto request = std::make_shared<amr_msgs::srv::AStar::Request>();
            request->start.x = amr_states_[i].pose.x;
            request->start.y = amr_states_[i].pose.y;
            // 随机选择一个任务点
            size_t rand_index = std::rand() % task_points_.size();
            auto target = task_points_[rand_index];
            request->end.x = target.x;
            request->end.y = target.y;
            // 填充地图数据
            for (size_t i = 0; i < free_points_.size(); i++)
            {
                amr_msgs::msg::Point p;
                p.x = free_points_[i].x;
                p.y = free_points_[i].y;
                request->map.push_back(p);
            }
            // ---------- 打印日志 ----------
            spdlog::info("amr_{} task assigned: start=({:.3f}, {:.3f}) end=({:.3f}, {:.3f})",
                        i,
                        request->start.x, request->start.y,
                        request->end.x, request->end.y);
            a_star_cli_[i]->async_send_request(
                request,
                [this, i](ServiceResponseFuture future){ a_star_cli_callback(future, i); });
        }
    }
    
}

void AmrPathPlanning::task_sub_callback(const amr_msgs::msg::TaskPoint::SharedPtr msg)
{
    int amr_id = 0;
    auto request = std::make_shared<amr_msgs::srv::AStar::Request>();
    request->start.x = msg->start.x;
    request->start.y = msg->start.y;
    request->end.x = msg->end.x;
    request->end.y = msg->end.y;
    a_star_cli_[amr_id]->async_send_request(
        request,
        [this, amr_id](ServiceResponseFuture future){ a_star_cli_callback(future, amr_id); });
}

void AmrPathPlanning::a_star_cli_callback(ServiceResponseFuture future, int amr_id)
{
    auto response = future.get();
    spdlog::info("Received response size {}", response->points.size());
    // TODO 无路径
    if (response->points.size() == 0) return;
    // 清空旧路径
    points_[amr_id].points.clear();
    points_[amr_id].points = response->points;
    // 发送路径到运动控制模块
    send_points_to_motion(amr_id, points_[amr_id]);
}

void AmrPathPlanning::send_points_to_motion(int amr_id, amr_msgs::msg::Points points)
{
    auto msg = amr_msgs::msg::Points();
    msg = points;
    points_pubs_[amr_id]->publish(msg);
    // log info
    spdlog::info("path planning send to mc points {}", points.points.size());
}

void AmrPathPlanning::amr_states_subs_callback(const amr_msgs::msg::AmrState::SharedPtr msg, int amr_id)
{
    amr_states_[amr_id] = *msg;
}



}  // namespace amr_path_planning
