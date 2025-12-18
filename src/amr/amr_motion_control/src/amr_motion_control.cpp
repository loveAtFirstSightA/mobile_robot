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

#include "amr_motion_control/amr_motion_control.hpp"

namespace amr_motion_control
{

AmrMotionControl::AmrMotionControl() : Node("amr_motion_control")
{
    poses_.resize(NUM_AMR);
    pose_subs_.resize(NUM_AMR);

    lasers_.resize(NUM_AMR);
    laser_subs_.resize(NUM_AMR);

    points_.resize(NUM_AMR);
    points_subs_.resize(NUM_AMR);

    paths_.resize(NUM_AMR);
    
    vel_pubs_.resize(NUM_AMR);
    amr_states_.resize(NUM_AMR);
    amr_state_pubs_.resize(NUM_AMR);

    motion_control_timers_.resize(NUM_AMR);
    feedback_state_timers_.resize(NUM_AMR);
    pure_pursuit_cli_.resize(NUM_AMR);

    path_index_.resize(NUM_AMR);

    for (int i = 0; i < NUM_AMR; ++i)
    {
        // Pose 订阅
        pose_subs_[i] = this->create_subscription<amr_msgs::msg::Pose>(
            "/amr_" + std::to_string(i) + "_pose", 10,
            [this, i](const amr_msgs::msg::Pose::SharedPtr msg){ pose_callback(msg, i); });
        
        // Laser 订阅
        laser_subs_[i] = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/amr_" + std::to_string(i) + "/scan", 10,
            [this, i](const sensor_msgs::msg::LaserScan::SharedPtr msg){ laser_callback(msg, i); });

        // Velocity 发布
        vel_pubs_[i] = this->create_publisher<geometry_msgs::msg::Twist>(
            "/amr_" + std::to_string(i) + "/cmd_vel", 10);
        
        // Pure Pursuit 客户端（共用同一个服务名）
        pure_pursuit_cli_[i] = this->create_client<amr_msgs::srv::PurePursuit>("pure_pursuit_server");

        // Timer motion control
        motion_control_timers_[i] = this->create_wall_timer(
            std::chrono::milliseconds(20),
            [this, i]() { motion_control_timers_callback(i); });
        motion_control_timers_[i]->cancel(); // 初始化时暂停
        
        // Timer feedback state
        feedback_state_timers_[i] = this->create_wall_timer(
            std::chrono::milliseconds(20),
            [this, i]() { feedback_state_timers_callback(i); });
        feedback_state_timers_[i]->reset(); // 初始化时开始

        // Path 订阅
        points_subs_[i] = this->create_subscription<amr_msgs::msg::Points>(
            "/amr_" + std::to_string(i) + "_points", 10,
            [this, i](const amr_msgs::msg::Points::SharedPtr msg){ points_callback(msg, i); });
        
        // 上报机器人所有状态（状态机 位姿 已经完成的路径id）
        amr_state_pubs_[i] = this->create_publisher<amr_msgs::msg::AmrState>(
            "/amr_" + std::to_string(i) + "_states", 10);        
    }
    // 状态初始化
    std::vector<std::vector<double>> initial_poses = {
        {1.0, 21.0, -1.5708},
        {-1.0, 21.0, -1.5708},
        {3.0, 21.0, -1.5708},
        {-3.0, 21.0, -1.5708},
        {5.0, 21.0, -1.5708},
        {-5.0, 21.0, -1.5708},
        {7.0, 21.0, -1.5708},
        {-7.0, 21.0, -1.5708},
        {9.0, 21.0, -1.5708},
        {-9.0, 21.0, -1.5708},
        {1.0, -21.0, 1.5708},
        {-1.0, -21.0, 1.5708},
        {3.0, -21.0, 1.5708},
        {-3.0, -21.0, 1.5708},
        {5.0, -21.0, 1.5708},
        {-5.0, -21.0, 1.5708},
        {7.0, -21.0, 1.5708},
        {-7.0, -21.0, 1.5708},
        {9.0, -21.0, 1.5708},
        {-9.0, -21.0, 1.5708}
    };
    for (size_t i = 0; i < amr_states_.size(); i++) {
        amr_states_[i].state = "idle"; // idle run avoid
        amr_states_[i].pose.x = initial_poses[i][0]; // 0 索引是 x
        amr_states_[i].pose.y = initial_poses[i][1]; // 1 索引是 y
        amr_states_[i].pose.yaw = initial_poses[i][2]; // 2 索引是 yaw
    }
}

AmrMotionControl::~AmrMotionControl() {}

// 通用函数
void AmrMotionControl::pose_callback(const amr_msgs::msg::Pose::SharedPtr msg, int amr_id)
{
    poses_[amr_id].x = msg->x;
    poses_[amr_id].y = msg->y;
    poses_[amr_id].yaw = msg->yaw;

    amr_states_[amr_id].pose = poses_[amr_id];
}

void AmrMotionControl::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg, int amr_id)
{
    lasers_[amr_id] = *msg;
}

bool AmrMotionControl::is_obstacle_ahead(int amr_id)
{
    // 检查 amr_id 是否有效
    if (amr_id < 0 || amr_id >= NUM_AMR) {
        return false;
    }

    const auto& scan = lasers_[amr_id];  // 从 vector 获取对应激光数据
    if (scan.ranges.empty()) {
        return false;
    }

    // 前方 ±7.5°，总共 15°
    constexpr double front_angle_rad = 7.5 * M_PI / 180.0;
    int start_idx = std::max(0, int((0.0 - front_angle_rad - scan.angle_min) / scan.angle_increment));
    int end_idx   = std::min(int(scan.ranges.size() - 1),
                             int((0.0 + front_angle_rad - scan.angle_min) / scan.angle_increment));

    // 遍历前方激光点，判断障碍物
    for (int i = start_idx; i <= end_idx; ++i) {
        double r = scan.ranges[i];
        if (r > scan.range_min && r < obstacle_threshold_) {
            return true;
        }
    }

    return false;
}

void AmrMotionControl::points_callback(const amr_msgs::msg::Points::SharedPtr msg, int amr_id)
{
    // 这里将点转换成路径 考虑路径点动态更新的情况
    spdlog::info("amr_{} motion control target points: {}",amr_id, msg->points.size());

    // ---------- 更新点列 ----------
    if (amr_states_[amr_id].state == "idle") {
        points_[amr_id].points = msg->points;
    } else if (amr_states_[amr_id].state == "run") {
        motion_control_timers_[amr_id]->cancel();
        send_velocity(amr_id, 0.0f, 0.0f);
        points_[amr_id].points.clear();

        amr_msgs::msg::Point current;
        current.x = poses_[amr_id].x;
        current.y = poses_[amr_id].y;
        points_[amr_id].points.push_back(current);

        for (auto &p : msg->points) {
            points_[amr_id].points.push_back(p);
        }
    }
    paths_[amr_id].paths.clear();
    int n = points_[amr_id].points.size();
    if (n < 2) return;

    // 第一条 rotate
    amr_msgs::msg::Path path;
    path.type = 0;
    path.rotate.sa = poses_[amr_id].yaw;
    double dx = points_[amr_id].points[1].x - points_[amr_id].points[0].x;
    double dy = points_[amr_id].points[1].y - points_[amr_id].points[0].y;
    path.rotate.ea = std::atan2(dy, dx);
    paths_[amr_id].paths.push_back(path);

    // 合并共线直线
    int line_start_idx = 0;
    for (int i = 1; i < n - 1; ++i) {
        auto &p0 = points_[amr_id].points[i - 1];
        auto &p1 = points_[amr_id].points[i];
        auto &p2 = points_[amr_id].points[i + 1];

        double cross = (p1.x - p0.x)*(p2.y - p0.y) - (p1.y - p0.y)*(p2.x - p0.x);
        if (std::fabs(cross) > 1e-6) { // 不共线
            // 生成line
            path.type = 1;
            path.line.sx = points_[amr_id].points[line_start_idx].x;
            path.line.sy = points_[amr_id].points[line_start_idx].y;
            path.line.ex = p1.x;
            path.line.ey = p1.y;
            paths_[amr_id].paths.push_back(path);

            // 下一条rotate
            path.type = 0;
            path.rotate.sa = std::atan2(p1.y - points_[amr_id].points[line_start_idx].y,
                                        p1.x - points_[amr_id].points[line_start_idx].x);
            path.rotate.ea = std::atan2(p2.y - p1.y, p2.x - p1.x);
            paths_[amr_id].paths.push_back(path);

            line_start_idx = i;
        }
    }
    // 处理最后一段直线
    path.type = 1;
    path.line.sx = points_[amr_id].points[line_start_idx].x;
    path.line.sy = points_[amr_id].points[line_start_idx].y;
    path.line.ex = points_[amr_id].points[n-1].x;
    path.line.ey = points_[amr_id].points[n-1].y;
    paths_[amr_id].paths.push_back(path);

    // 打印最终路径
    spdlog::info("=== amr_{} final paths ===", amr_id);
    for (size_t i = 0; i < paths_[amr_id].paths.size(); ++i) {
        const auto &path = paths_[amr_id].paths[i];
        if (path.type == 0) { // rotate
            spdlog::info("Path {}: ROTATE start_angle = {:.3f}, end_angle = {:.3f}", 
                        i, path.rotate.sa, path.rotate.ea);
        } else if (path.type == 1) { // line
            spdlog::info("Path {}: LINE start = ({:.3f}, {:.3f}), end = ({:.3f}, {:.3f})",
                        i, path.line.sx, path.line.sy, path.line.ex, path.line.ey);
        } else {
            spdlog::warn("Path {}: UNKNOWN type {}", i, path.type);
        }
    }
    spdlog::info("=== end of paths ===");

    // motion control timer 已启动
    motion_control_timers_[amr_id]->reset();
    spdlog::info("motion_control_timers_[{}]->reset()", amr_id);
}

void AmrMotionControl::motion_control_timers_callback(int amr_id)
{
    amr_states_[amr_id].state = "run";

    if (is_obstacle_ahead(amr_id)) 
    {
        send_velocity(amr_id, 0.0, 0.0);
        amr_states_[amr_id].state = "pause";
        return;
    }

    if (paths_[amr_id].paths[path_index_[amr_id]].type == 0)  // rotate
    {
        double theta_now = poses_[amr_id].yaw;
        double theta_target = paths_[amr_id].paths[path_index_[amr_id]].rotate.ea;
        double dtheta = theta_target - theta_now;
        while (dtheta > M_PI)  dtheta -= 2 * M_PI;
        while (dtheta < -M_PI) dtheta += 2 * M_PI;

        if (std::fabs(dtheta) < (0.017f * 3.0f)) {
            send_velocity(amr_id, 0.0, 0.0);
            path_index_[amr_id] ++;
            if (static_cast<size_t>(path_index_[amr_id]) >= paths_[amr_id].paths.size()) {
                spdlog::info("amr_{} all paths executed.", amr_id);
                path_index_[amr_id] = 0;
                motion_control_timers_[amr_id]->cancel();
                amr_states_[amr_id].state = "idle";
            }
        } else {
            send_velocity(amr_id, 0.0, (dtheta > 0 ? w_max_ : -w_max_));
        }
    }
    if (paths_[amr_id].paths[path_index_[amr_id]].type == 1)  // line
    {
        auto request = std::make_shared<amr_msgs::srv::PurePursuit::Request>();
        request->pose = poses_[amr_id];
        request->line = paths_[amr_id].paths[path_index_[amr_id]].line;

        double dist_total = std::hypot(paths_[amr_id].paths[path_index_[amr_id]].line.ex - paths_[amr_id].paths[path_index_[amr_id]].line.sx,
                                       paths_[amr_id].paths[path_index_[amr_id]].line.ey - paths_[amr_id].paths[path_index_[amr_id]].line.sy);
        double dist_now = std::hypot(poses_[amr_id].x - paths_[amr_id].paths[path_index_[amr_id]].line.sx,
                                     poses_[amr_id].y - paths_[amr_id].paths[path_index_[amr_id]].line.sy);
        double dist_acc = (v_max_ * v_max_) / (2 * a_max_), dist_dec = dist_acc;

        double v_cmd = (dist_total < 1.0) ? 0.2 :
                       (dist_now < dist_acc) ? std::sqrt(2 * a_max_ * dist_now) :
                       (dist_now > dist_total - dist_dec) ? std::sqrt(2 * a_max_ * (dist_total - dist_now)) :
                       v_max_;

        request->v = v_cmd;
        pure_pursuit_cli_[amr_id]->async_send_request(
            request,
            [this, amr_id](ServiceResponseFuture future){ pure_pursuit_cli_response_callback(future, amr_id); });
    }
}

void AmrMotionControl::pure_pursuit_cli_response_callback(ServiceResponseFuture future, int amr_id)
{
    if (amr_states_[amr_id].state == "idle") {
        send_velocity(amr_id, 0.0, 0.0);  // 路径完成，安全清零
        return;
    }

    auto response = future.get();
    send_velocity(amr_id, response->velocity.v, response->velocity.w);

    double error_dist = std::hypot(paths_[amr_id].paths[path_index_[amr_id]].line.ex - poses_[amr_id].x,
                                   paths_[amr_id].paths[path_index_[amr_id]].line.ey - poses_[amr_id].y);

    if (error_dist < 0.015f) {
        send_velocity(amr_id, 0, 0);
        path_index_[amr_id] ++;

        if (static_cast<size_t>(path_index_[amr_id]) >= paths_[amr_id].paths.size()) {
            spdlog::info("amr_{} all paths executed.", amr_id);
            path_index_[amr_id] = 0;
            motion_control_timers_[amr_id]->cancel();
            amr_states_[amr_id].state = "idle";
        }
    }
}

// 发送机器人速度
void AmrMotionControl::send_velocity(int amr_id, double v, double w)
{
    if (std::fabs(v) < 0.01f) v = 0.0;
    if (std::fabs(w) < 0.01f) w = 0.0;
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = v;
    msg.angular.z = w;
    vel_pubs_[amr_id]->publish(msg);
}

// 定时上报所有机器人状态
void AmrMotionControl::feedback_state_timers_callback(int amr_id)
{
    auto msg = amr_msgs::msg::AmrState();
    msg = amr_states_[amr_id];
    msg.amr_id = amr_id;
    amr_state_pubs_[amr_id]->publish(msg);
}

}  // namespace amr_motion_control
