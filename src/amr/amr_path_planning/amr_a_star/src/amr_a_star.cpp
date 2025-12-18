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

#include "amr_a_star/amr_a_star.hpp"
#include "spdlog/spdlog.h"

namespace amr_a_star
{
AmrAStar::AmrAStar() : Node("amr_s_star")
{
    a_star_server_ = this->create_service<amr_msgs::srv::AStar>(
        "amr_a_star", std::bind(&AmrAStar::a_star_server_callback, this, std::placeholders::_1, std::placeholders::_2));
}

AmrAStar::~AmrAStar() {}

void AmrAStar::a_star_server_callback(
    const std::shared_ptr<amr_msgs::srv::AStar::Request> request,
    std::shared_ptr<amr_msgs::srv::AStar::Response> response)
{
    spdlog::info("A_Star start: [{:.3f}, {:.3f}], end: [{:.3f}, {:.3f}]",
        request->start.x, request->start.y, request->end.x, request->end.y);
    
    Point start, goal;
    // 从请求中读取起点与终点
    start.x = request->start.x;
    start.y = request->start.y;
    goal.x = request->end.x;
    goal.y = request->end.y;
    // 填充搜索数据
    free_points_.resize(request->map.size());
    for (size_t i = 0; i < request->map.size(); i++)
    {
        Point p;
        p.x = request->map[i].x;
        p.y = request->map[i].y;
        free_points_.push_back(p);
    }
    // open list（候选节点集合），close list（已访问节点集合）
    std::vector<AStarNode> open_list;
    std::vector<AStarNode> close_list;

    // 构建起点节点：g=0，h为起点到终点的曼哈顿距离，f=g+h
    AStarNode start_node{start, 0.0f, manhattan(start, goal), manhattan(start, goal), -1};
    open_list.push_back(start_node);

    spdlog::info("A* 搜索开始: 起点 [{:.3f}, {:.3f}], 终点 [{:.3f}, {:.3f}]", start.x, start.y, goal.x, goal.y);

    while (!open_list.empty())
    {
        // 1. 在 open list 中找到 f 值最小的节点
        int current_index = 0;
        for (int i = 1; i < (int)open_list.size(); i++) 
        {
            if (open_list[i].f < open_list[current_index].f)
                current_index = i;
        }

        AStarNode current = open_list[current_index];
        // spdlog::info("当前节点: [{:.3f}, {:.3f}], g={:.3f}, h={:.3f}, f={:.3f}", 
        //             current.p.x, current.p.y, current.g, current.h, current.f);

        // 2. 判断是否到达目标点
        if (samePoint(current.p, goal)) 
        {
            std::vector<Point> path;
            while (current.parent != -1)
            {
                path.push_back(current.p);
                current = close_list[current.parent];
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());

            spdlog::info("A* 搜索成功，路径长度 = {}", path.size());
            for (size_t i = 0; i < path.size(); i++) {
                spdlog::info("路径点 {}: [{:.3f}, {:.3f}]", i, path[i].x, path[i].y);
            }
            // Response
            for (size_t i = 0; i < path.size(); i++) {
                amr_msgs::msg::Point point;
                point.x = path[i].x;
                point.y = path[i].y;
                response->points.push_back(point);
            }
            return;
        }

        // 3. 将当前节点移动到 close list
        close_list.push_back(current);
        open_list.erase(open_list.begin() + current_index);

        // 4. 遍历所有候选邻居点（即 free_points_ 中的点）
        for (size_t i = 0; i < free_points_.size(); i++)
        {
            Point neighbor = free_points_[i];

            // 跳过当前点自身
            if (samePoint(neighbor, current.p)) continue;

            // 限制邻居范围：曼哈顿距离 < 1.5
            double dist = manhattan(neighbor, current.p);
            if (dist > 1.5f) continue;

            // 跳过已在 close list 中的点
            bool in_close = false;
            for (auto &n : close_list) {
                if (samePoint(n.p, neighbor)) {
                    in_close = true;
                    break;
                }
            }
            if (in_close) continue;

            // 计算从当前点到邻居的 g 值
            double tentative_g = current.g + dist;

            bool in_open = false;
            for (auto &n : open_list) {
                if (samePoint(n.p, neighbor)) {
                    in_open = true;
                    // 如果找到更优路径，则更新 g、f、父节点
                    if (tentative_g < n.g) {
                        // spdlog::info("更新邻居: [{:.3f}, {:.3f}] 新g={:.3f} 原g={:.3f}", neighbor.x, neighbor.y, tentative_g, n.g);
                        n.g = tentative_g;
                        n.f = tentative_g + n.h;
                        n.parent = close_list.size() - 1;
                    }
                }
            }

            // 如果邻居不在 open list，则加入
            if (!in_open) {
                AStarNode new_astar_node{neighbor, tentative_g, manhattan(neighbor, goal), 0.0f, (int)close_list.size() - 1};
                new_astar_node.f = new_astar_node.g + new_astar_node.h;
                open_list.push_back(new_astar_node);

                // spdlog::info("加入邻居: [{:.3f}, {:.3f}], g={:.3f}, h={:.3f}, f={:.3f}", 
                //             neighbor.x, neighbor.y, new_astar_node.g, new_astar_node.h, new_astar_node.f);
            }
        }
    }

    // 如果 while 结束仍未找到目标点
    spdlog::warn("A* 搜索失败，未找到路径!");
    return;
}

}  // namespace amr_a_star
