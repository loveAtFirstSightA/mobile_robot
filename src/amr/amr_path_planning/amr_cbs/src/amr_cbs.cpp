#include "amr_cbs/amr_cbs.hpp"
#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <map>

namespace amr_cbs
{
AmrCBS::AmrCBS() : Node("amr_cbs")
{
    std::srand(std::time(nullptr));  // 初始化随机种子
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

        // 机器人状态订阅
        amr_states_subs_[i] = this->create_subscription<amr_msgs::msg::AmrState>(
            "/amr_" + std::to_string(i) + "_states", 10,
            [this, i](const amr_msgs::msg::AmrState::SharedPtr msg){ amr_states_subs_callback(msg, i); });

        // A* 客户端
        a_star_cli_[i] = this->create_client<amr_msgs::srv::AStar>("amr_a_star");
    }

    // timer init
    timer_ = this->create_wall_timer(
        std::chrono::seconds(3),
        std::bind(&AmrCBS::timer_callback, this));
    
    cache_points_ = free_points_;
}

AmrCBS::~AmrCBS() {}

void AmrCBS::amr_states_subs_callback(const amr_msgs::msg::AmrState::SharedPtr msg, int amr_id)
{
    amr_states_[amr_id] = *msg;
}

void AmrCBS::timer_callback()
{
    // 随机选择一个任务点
    size_t rand_index = std::rand() % task_points_.size();
    auto target = task_points_[rand_index];
    
    for (size_t i = 0; i < NUM_AMR; i++)
    {
        if (amr_states_[i].state == "idle") {
            for (size_t j = 0; j < points_[i].points.size(); j++)
            {
                Point p;
                p.x = points_[i].points[j].x;
                p.y = points_[i].points[j].y;
                cache_points_.push_back(p);
            }
        }
    }
    // TODO 找到距离目标点欧式最近的空闲机器人 发送请求
    // 找到距离目标点欧式最近的空闲机器人
    double min_dist = std::numeric_limits<double>::max();
    int best_amr = -1;

    for (size_t i = 0; i < NUM_AMR; i++)
    {
        if (amr_states_[i].state == "idle") {
            double dx = amr_states_[i].pose.x - target.x;
            double dy = amr_states_[i].pose.y - target.y;
            double dist = std::sqrt(dx * dx + dy * dy);

            if (dist < min_dist) {
                min_dist = dist;
                best_amr = static_cast<int>(i);
            }
        }
    }

    if (best_amr >= 0) {
        send_request(best_amr, target);
        spdlog::info("AMR_{} assigned task: target=({}, {})", 
                     best_amr, target.x, target.y);
    } else {
        spdlog::warn("No idle AMR available to assign task");
    }
}

void AmrCBS::send_request(int amr_id, Point target)
{
    points_[amr_id].points.clear();
    auto request = std::make_shared<amr_msgs::srv::AStar::Request>();
    request->start.x = amr_states_[amr_id].pose.x;
    request->start.y = amr_states_[amr_id].pose.y;
    request->end.x = target.x;
    request->end.y = target.y;
    // 填充地图数据
    for (size_t i = 0; i < cache_points_.size(); i++)
    {
        amr_msgs::msg::Point p;
        p.x = cache_points_[i].x;
        p.y = cache_points_[i].y;
        request->map.push_back(p);
    }
    // ---------- 打印日志 ----------
    spdlog::info("amr_{} task assigned: start=({:.3f}, {:.3f}) end=({:.3f}, {:.3f})",
                amr_id,
                request->start.x, request->start.y,
                request->end.x, request->end.y);
    a_star_cli_[amr_id]->async_send_request(
        request,
        [this, amr_id](ServiceResponseFuture future){ a_star_cli_callback(future, amr_id); });
}

void AmrCBS::a_star_cli_callback(ServiceResponseFuture future, int amr_id)
{
    auto response = future.get();
    spdlog::info("amr_{} Received response size {}", amr_id, response->points.size());
    if (response->points.empty()) return;
    points_[amr_id].points = response->points;

    // 删除路径点对应的 cache_points_
    constexpr double EPS = 1e-3;
    for (auto &p : response->points) {
        cache_points_.erase(
            std::remove_if(cache_points_.begin(), cache_points_.end(),
                        [&](const Point &fp){ 
                            return std::abs(fp.x - p.x) < EPS && std::abs(fp.y - p.y) < EPS;
                        }),
            cache_points_.end());
    }
    spdlog::info("amr_{} path processed, {} points removed from cache_points", amr_id, response->points.size());

    // 发送路径到运动控制模块
    send_points_to_motion(amr_id, points_[amr_id]);
}

void AmrCBS::send_points_to_motion(int amr_id, amr_msgs::msg::Points points)
{
    auto msg = amr_msgs::msg::Points();
    msg = points;
    points_pubs_[amr_id]->publish(msg);
    // log info
    spdlog::info("path planning send to mc points {}", points.points.size());
}

}  // namespace amr_cbs
