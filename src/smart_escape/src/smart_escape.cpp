#include "smart_escape/smart_escape.hpp"
#include <cmath>
#include <algorithm>
#include <random>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp> 
#include <chrono>

namespace smart_escape
{

using namespace std::chrono_literals;

/*
  SmartEscape.cpp — 实现文件（含详细注释）
  注释目标：对每个函数、关键步骤、数学转换给出“为什么这样做”和“技术效果”说明，
  以便直接用于专利说明书的实施例与有利效果描述。
*/

/*
  构造函数 SmartEscape()
  - 初始化 ROS 接口（publisher / subscriber / timer）
  - 创建用于可视化的 marker 发布器
  - 定时器负责定期触发 escape_timer_callback()，实现非阻塞控制循环
*/
SmartEscape::SmartEscape() : Node("smart_escape"), 
                             tf_buffer_(this->get_clock()), 
                             tf_listener_(tf_buffer_)
{
    // 发布机器人近似多边形（用于 RViz 可视化）
    robot_poly_pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("robot_polygon", 10);

    // 发布速度命令到 /cmd_vel，微步状态机会通过该发布接口下发速度
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    // 订阅激光（LIDAR）数据，用于构建趋势流场。
    // 注意：此回调仅缓存数据（on_scan），复杂计算由定时器线程处理，避免阻塞订阅回调。
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&SmartEscape::on_scan, this, std::placeholders::_1));
    
    // 订阅逃逸触发话题（由上层状态机或人工触发）
    escape_trigger_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "/trigger_escape", 10, std::bind(&SmartEscape::on_escape_trigger, this, std::placeholders::_1));
    
    // 可视化逃逸向量（marker），便于实验调参与论文图示
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("escape_marker", 10);

    // 定时器：周期性执行 escape_timer_callback（控制循环）
    escape_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(25), std::bind(&SmartEscape::escape_timer_callback, this));
}

SmartEscape::~SmartEscape() {}

/*
  on_scan()
  - 作用：将接收到的 LaserScan 缓存到双端队列 scans_deque_ 中（受 mutex 保护）。
  - 设计理由：
      1) 避免在订阅回调中进行昂贵计算，转而由定时器按需处理缓存数据；
      2) 使用 deque 保证插入/删除高效，队列只保留最近 trend_frame_count_ 帧。
*/
void SmartEscape::on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(scans_mutex_);
    scans_deque_.push_back(msg);
    // 保证缓存大小不超过 trend_frame_count_
    while (scans_deque_.size() > trend_frame_count_) scans_deque_.pop_front();
}

/*
  on_escape_trigger()
  - 由外部触发，开启 escape 模式。
  - 将微步状态机置为 IDLE 并重置 dr 过滤器的初始化标志，以便重新计算时间记忆。
*/
void SmartEscape::on_escape_trigger(const std_msgs::msg::Empty::SharedPtr msg)
{
    (void)msg;
    escape_active_ = true;
    micro_state_ = MicroState::IDLE;
    initialized_dr_ = false;
    RCLCPP_INFO(this->get_logger(), "Escape triggered (enhanced TFF).");
}

/*
  escape_timer_callback()
  - 定时器主循环：首先发布机器人多边形用于可视化，然后：
      1) 若 escape 未激活则返回；
      2) 调用 compute_escape_vector_from_trend() 计算 escape 向量（可能为空）；
      3) 若无向量则执行保守后退/旋转以获取更多观测；否则发布可视化并交由微步执行。
  - 设计理由：
      - 使用微步与时间低通结合，提升对动态与噪声环境的鲁棒性；
      - 保底后退行为避免因为观测不足导致永久停滞。
*/
void SmartEscape::escape_timer_callback()
{
    publish_robot_circle();
    if (!escape_active_) return;

    // 计算 escape 向量（局部坐标系）
    std::optional<Vec2> maybe_vec = compute_escape_vector_from_trend();
    if (!maybe_vec.has_value()) {
        // 数据不足或无明显趋势 -> 保守后退+小角速度尝试
        geometry_msgs::msg::Twist t;
        t.linear.x = -0.04;   // 慢速后退，拉开与障碍的距离以获取新视角
        t.angular.z = 0.2;    // 同时略微旋转以改变视角
        cmd_vel_pub_->publish(t);
        return;
    }

    Vec2 escape_vec = maybe_vec.value();

    // 发布可视化箭头（便于调参）
    publish_escape_marker(escape_vec);

    // 如果状态机不在 IDLE，则让状态机继续当前微步
    if (micro_state_ != MicroState::IDLE) {
        // 这里保留逻辑以便未来增强：可以适当调整 target angle
    }

    // 交由微步状态机执行（TURN->FORWARD->VERIFY）
    execute_micro_step(escape_vec);
}

/*
  publish_robot_circle()
  - 功能：发布近似机器人边界的多边形，用于可视化与算法调试。
  - 说明：该多边形基于 robot_radius_ 构造 N 点圆近似，frame_id 设为 base_footprint。
*/
void SmartEscape::publish_robot_circle()
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

/*
  angular_smooth()
  - 功能：对输入向量（按角度索引）做环状移动平均，处理 wrap-around（循环边界）。
  - 设计意图：减弱单束异常（噪声/反射）对趋势估计的影响，尤其是高分辨率 LIDAR。
  - 参数说明：
      in: 输入数组，长度等于激光束数（按角度索引）
      window: 平滑窗口（应为奇数以对称），例如 5
  - 返回值：平滑后的数组，长度与输入相同。
*/
std::vector<double> SmartEscape::angular_smooth(const std::vector<double>& in, int window)
{
    int n = in.size();
    std::vector<double> out(n, 0.0);
    if (n == 0) return out;
    int half = window / 2;
    for (int i = 0; i < n; ++i) {
        double s = 0.0;
        int cnt = 0;
        // 环形索引，保证边界平滑
        for (int k = -half; k <= half; ++k) {
            int idx = (i + k + n) % n;
            s += in[idx];
            ++cnt;
        }
        out[i] = s / static_cast<double>(cnt);
    }
    return out;
}

/*
  angle_diff(a, b)
  - 作用：计算 a - b 的角差，并规约到 [-pi, pi] 区间
  - 用途：用于角度控制环节（例如 PD 控制器、滞回判定）
*/
double SmartEscape::angle_diff(double a, double b)
{
    double d = a - b;
    while (d > M_PI) d -= 2*M_PI;
    while (d < -M_PI) d += 2*M_PI;
    return d;
}

/*
  check_forward_clear()
  - 功能：检测以 dir_angle 为中心的前方扇区内，是否所有测距均大于 min_dist（即前方是否安全）
  - 实现细节：
      1) 选择扇区宽度（此处固定为 20 度），也可根据 robot_radius_ 与 min_dist 动态计算；
      2) 遍历 scan 中各束，若角度在扇区内且距离小于 min_dist 则认为不安全。
  - 设计目的：在从 TURNING 转到 FORWARDING 之前做安全校验，避免直接朝狭窄缝隙高速前进。
*/
bool SmartEscape::check_forward_clear(const sensor_msgs::msg::LaserScan::SharedPtr &scan, double dir_angle, double min_dist)
{
    if (!scan) return false;
    double amin = scan->angle_min;
    double ainc = scan->angle_increment;
    int N = scan->ranges.size();
    // 扇区宽度（可依据 robot geometry 调整）
    double sector = 20.0 * M_PI / 180.0;
    for (int i = 0; i < N; ++i) {
        double a = amin + i * ainc;
        double d_angle = angle_diff(a, dir_angle);
        if (std::fabs(d_angle) <= sector / 2.0) {
            float r = scan->ranges[i];
            if (!std::isfinite(r)) continue; // 无穷大表示无返回，视为空（视野外），不计为阻碍
            if (r < min_dist) return false; // 有障碍进入安全距离
        }
    }
    return true;
}

/*
  compute_escape_vector_from_trend()
  ——— 算法核心（注释极详细）
  核心思想（简述）：
    使用多帧激光（scans_deque_）的时间维度信息，计算每个角度上的距离变化率 dr/dt，
    然后基于 dr/dt 的符号决定该角度是“变稀疏（dr>0）”还是“被挤压（dr<0）”，
    再将这些角度的“趋势”映射为逃逸方向的向量场贡献，按置信度与距离加权后累加，
    最终得到一个在机器人局部坐标系下的合成逃逸向量（escape_vec）。

  详细步骤说明：
    1) 确保 scans_deque_ 中帧数足够（>= trend_frame_count_）；若不足则返回空；
    2) 采用“中心差分”方法：选取中间帧 mid，以及 mid-1（prev）和 mid+1（next）
       以 (r_next - r_prev) / dt 估计 dr/dt，中心差分比前向差分更稳健；
    3) 对每帧先做角向平滑（angular_smooth）以抑制单束噪声；
    4) 对每个角度：
         - 计算 dr_dt 并进行时间低通（dr_filtered_）以记忆历史变化；
         - 计算置信度 conf（例如当某一侧为 max_range 时降低置信度）；
         - 计算距离权重 wdist（近障碍影响更强）；
         - 计算 mag = abs(dr)*conf*wdist 作为该角度的贡献幅值；
         - 映射方向：若 dr>=0（障碍远离）则该角度本身是“安全/疏松”方向；
                     若 dr<0（障碍靠近）则应当“远离该障碍”，即 escape_angle = angle + pi；
         - 将贡献投影为 vx, vy 累加到 acc_x, acc_y，并累计权重 acc_w。
    5) 若总权重 acc_w 非正则返回空；否则得到平均向量 vx = acc_x/acc_w, vy = acc_y/acc_w;
    6) 对结果作强度/角度稳定性判断（滞回/平滑），更新 last_escape_angle_ 并返回 Vec2。

  该实现解决的问题与工程效果：
    - 抑制单束噪声（angular smoothing）；
    - 使用中心差分 + 时间低通减少偶发测距跳变对方向判断的影响；
    - 将“被挤压方向”合理映射为“朝相反方向逃逸”，避免你之前实现中向量方向错误导致朝障碍移动的 bug；
    - 通过距离加权防止机器人朝近处微缝高速冲撞；
    - 滞回机制用于避免逃逸方向在弱信号下剧烈摆动，提高控制稳定性（在专利中可作为一项保护点）。
*/
std::optional<SmartEscape::Vec2> SmartEscape::compute_escape_vector_from_trend()
{
    std::lock_guard<std::mutex> lock(scans_mutex_);
    if (scans_deque_.size() < trend_frame_count_) return std::nullopt;

    // 选择中心帧（mid）和其前后帧用于中心差分
    int K = scans_deque_.size();
    int mid = K / 2; // integer division；当 K 为奇数时 mid 为正中
    auto &scan_prev = scans_deque_[std::max(0, mid-1)];
    auto &scan_mid  = scans_deque_[mid];
    auto &scan_next = scans_deque_[std::min(K-1, mid+1)];

    // 计算时间差 dt（使用帧的时间戳）；若异常则使用默认 dt
    double t_next = scan_next->header.stamp.sec + 1e-9 * scan_next->header.stamp.nanosec;
    double t_prev = scan_prev->header.stamp.sec + 1e-9 * scan_prev->header.stamp.nanosec;
    double dt = t_next - t_prev;
    if (dt <= 1e-5) dt = trend_dt_default_;

    // 使用中间帧的角度参数作为统一角度基准（避免不同帧角度参数略有出入）
    size_t M = scan_mid->ranges.size();
    if (M == 0) return std::nullopt;
    double angle_min = scan_mid->angle_min;
    double angle_inc = scan_mid->angle_increment;
    double max_r = scan_mid->range_max;

    // 为 prev/mid/next 构建 double 精度的 ranges 数组，并将非有限值替换为 max_r（表示测距超出）
    std::vector<double> r_prev(M), r_mid(M), r_next(M);
    for (size_t i = 0; i < M; ++i) {
        // 若索引在某帧越界或该束无返回（inf），用 max_r 填充，表示远处开阔
        float rp = (i < scan_prev->ranges.size() && std::isfinite(scan_prev->ranges[i])) ? scan_prev->ranges[i] : max_r;
        float rm = (i < scan_mid->ranges.size()  && std::isfinite(scan_mid->ranges[i]))  ? scan_mid->ranges[i]  : max_r;
        float rn = (i < scan_next->ranges.size() && std::isfinite(scan_next->ranges[i])) ? scan_next->ranges[i] : max_r;
        r_prev[i] = rp;
        r_mid[i]  = rm;
        r_next[i] = rn;
    }

    // 对每帧做角向平滑，抑制单束噪声（例如反射、跨越）
    auto r_prev_s = angular_smooth(r_prev, ang_smooth_window_);
    auto r_mid_s  = angular_smooth(r_mid,  ang_smooth_window_);
    auto r_next_s = angular_smooth(r_next, ang_smooth_window_);

    // 初始化时间滤波数组 dr_filtered_（如果尚未初始化或长度发生变化）
    if (!initialized_dr_ || dr_filtered_.size() != M) {
        dr_filtered_.assign(M, 0.0);
        initialized_dr_ = true;
    }

    // 累计逃逸向量（加权）
    double acc_x = 0.0, acc_y = 0.0, acc_w = 0.0;

    for (size_t i = 0; i < M; ++i) {
        double rl = r_next_s[i]; // t+1
        double ro = r_prev_s[i]; // t-1

        // 中心差分估计 dr/dt（单位 m/s）：(r(t+1) - r(t-1)) / dt
        // 正值表示该方向障碍在变远（可能是 gap），负值表示障碍在靠近（被挤压）
        double dr_dt = (rl - ro) / dt;

        // 对 dr/dt 做时间低通，记忆先前的变化，减小瞬时噪声影响
        dr_filtered_[i] = time_alpha_ * dr_dt + (1.0 - time_alpha_) * dr_filtered_[i];
        double dr_use = dr_filtered_[i];

        // 角度对应的方向（局部坐标）
        double angle = angle_min + angle_inc * static_cast<double>(i);

        // 置信度计算：当某一侧为 max_range 时（表示该方向可能无遮挡或测距超限），降低置信度
        double conf = 1.0;
        if (ro >= max_r * 0.999 || rl >= max_r * 0.999) {
            // 若前/后帧任一为 max_range，则该角度的 dr 判定可靠性降低（因为可能为开阔）
            conf *= 0.6;
        }

        // 距离加权：近处障碍对决策影响更大，权重 ~ (1/dist)^p
        double dist = std::max(0.001, std::min(r_mid_s[i], max_r));
        double wdist = std::pow(1.0 / dist, weight_distance_exponent_);

        // 贡献强度：绝对 dr（越大说明变化越明显）乘以置信度与距离权重
        double mag = std::abs(dr_use) * conf * wdist;

        // 方向映射（关键点）：
        // - dr >= 0：障碍在变远 -> 该角度对应的方向本身是“可行/放松”的方向，使用 angle
        // - dr <  0：障碍在靠近 -> 应该“远离障碍”，因此 escape_angle = angle + pi
        //   说明：angle 表示“障碍所在方向”，所以“远离障碍”应当沿该方向的反向
        double escape_angle;
        if (dr_use >= 0.0) escape_angle = angle;
        else escape_angle = angle + M_PI;

        // 将角度规约到 [-pi, pi] 范围，便于后续数值稳定性
        while (escape_angle > M_PI) escape_angle -= 2*M_PI;
        while (escape_angle < -M_PI) escape_angle += 2*M_PI;

        // 将该角度的贡献转到笛卡尔坐标并累加
        double vx = mag * std::cos(escape_angle);
        double vy = mag * std::sin(escape_angle);

        acc_x += vx;
        acc_y += vy;
        acc_w += mag;
    }

    // 若没有任何贡献则返回空（无可信逃逸信号）
    if (acc_w <= 0.0) return std::nullopt;

    // 平均化合成向量（按权重归一）
    double vx = acc_x / acc_w;
    double vy = acc_y / acc_w;

    // 规范化并做最小强度判断，避免数值极小造成不稳定指令
    double norm = std::hypot(vx, vy);
    if (norm < 1e-6) return std::nullopt;

    Vec2 res{vx, vy};

    // 稳定性增强（滞回/hysteresis）：
    // 若新角度与上次角度差距较大且强度较弱，则倾向保留旧方向避免振荡
    double new_ang = std::atan2(res.y, res.x);
    double last_ang = last_escape_angle_;
    double ang_err = std::fabs(angle_diff(new_ang, last_ang));
    double hyst_rad = hysteresis_angle_deg_ * M_PI / 180.0;
    if (ang_err > hyst_rad && norm < 0.5 * escape_vector_threshold_ * 50.0) {
        // 当新方向跳跃超过滞回阈并且信号又很弱时，向量向上次方向偏移（混合保留）
        double lx = std::cos(last_ang);
        double ly = std::sin(last_ang);
        res.x = 0.7 * res.x + 0.3 * lx;
        res.y = 0.7 * res.y + 0.3 * ly;
    }

    // 更新 last_escape_angle_（采用一定平滑，以避免剧烈更新）
    double smoothed_angle = 0.6 * last_escape_angle_ + 0.4 * std::atan2(res.y, res.x);
    last_escape_angle_ = smoothed_angle;

    return res;
}

/*
  execute_micro_step()
  - 这是将 escape 向量安全地转成机器人动作的核心：采用状态机分阶段执行。
  - 设计目标：避免直接把 escape 向量映射成一次性大动作（可能碰撞），而是：
      TURNING：先朝向目标角；
      FORWARDING：在确认前方一段距离安全后前进 microstep_distance_；
      VERIFY：停止并检查是否显著改善（例如前向最小距离增大），否则回退或重试。
  - 此状态机便于在专利中描述“分段微步执行策略”和“执行验证机制”作为技术方案。
*/
void SmartEscape::execute_micro_step(const Vec2 &escape_vec)
{
    // 计算目标角度（robot 局部坐标系）：atan2(y, x)
    double desired = std::atan2(escape_vec.y, escape_vec.x);
    // 将角度规约到 [-pi, pi]
    while (desired > M_PI) desired -= 2*M_PI;
    while (desired < -M_PI) desired += 2*M_PI;

    // 根据当前 micro_state_ 决定是否更新 current_target_angle_
    if (micro_state_ == MicroState::IDLE) {
        // 若当前空闲，接受新的目标角并进入 TURNING 状态
        current_target_angle_ = desired;
        micro_state_ = MicroState::TURNING;
        state_start_time_ = this->now();
    } else if (micro_state_ == MicroState::TURNING) {
        // 在转的时候允许小幅度适应（若新目标与当前目标在 adapt_thresh 内）
        double adapt_thresh = 20.0 * M_PI / 180.0;
        if (std::fabs(angle_diff(desired, current_target_angle_)) < adapt_thresh) {
            current_target_angle_ = 0.6 * current_target_angle_ + 0.4 * desired;
        }
    } else if (micro_state_ == MicroState::FORWARDING) {
        // 前进阶段不改变目标角，保证动作一致性
    }

    // 根据状态发布相应的速度命令
    geometry_msgs::msg::Twist cmd;
    if (micro_state_ == MicroState::TURNING) {
        // 计算角误差（相对于 robot 当前朝向为 0）
        double ang_err = angle_diff(current_target_angle_, 0.0);
        // PD-like 比例控制（这里只用 KP）
        double ang_kp = 1.4;
        double wz = saturate(ang_kp * ang_err, -angular_speed_max_, angular_speed_max_);
        // 若角误差很小，则可尝试前进（需要再做前方安全检查）
        if (std::fabs(ang_err) < turn_tolerance_rad_) {
            // 获取最新一帧扫描以做前方检查
            sensor_msgs::msg::LaserScan::SharedPtr lastscan;
            {
                std::lock_guard<std::mutex> lock(scans_mutex_);
                if (!scans_deque_.empty()) lastscan = scans_deque_.back();
            }
            bool clear = true;
            if (lastscan) clear = check_forward_clear(lastscan, current_target_angle_, safety_distance_);
            if (clear) {
                // 转向到位且前方安全 -> 进入前进状态
                micro_state_ = MicroState::FORWARDING;
                state_start_time_ = this->now();
                cmd.linear.x = std::min(forward_speed_, linear_speed_max_);
                cmd.angular.z = 0.0;
                cmd_vel_pub_->publish(cmd);
                return;
            } else {
                // 前方不安全：在原地小幅后退同时继续微调角度（增加通过缝隙的可能性）
                cmd.linear.x = -0.03; // 小幅后退，避免贴障
                cmd.angular.z = wz;
                cmd_vel_pub_->publish(cmd);
                return;
            }
        } else {
            // 单纯转向（角速度控制）
            cmd.linear.x = 0.0;
            cmd.angular.z = wz;
            cmd_vel_pub_->publish(cmd);
            return;
        }
    } else if (micro_state_ == MicroState::FORWARDING) {
        // 前进一段固定时间（microstep_distance_/forward_speed_）
        double duration = microstep_distance_ / std::max(0.01, forward_speed_);
        double elapsed = (this->now() - state_start_time_).seconds();
        if (elapsed < duration) {
            // 每个周期检查前方是否仍然安全，若不安全则中断并退回 IDLE
            sensor_msgs::msg::LaserScan::SharedPtr lastscan;
            {
                std::lock_guard<std::mutex> lock(scans_mutex_);
                if (!scans_deque_.empty()) lastscan = scans_deque_.back();
            }
            bool clear = true;
            if (lastscan) clear = check_forward_clear(lastscan, current_target_angle_, safety_distance_);
            if (!clear) {
                // 中断前进，执行小幅后退并回到 IDLE（等待下一次 escape 向量）
                geometry_msgs::msg::Twist t;
                t.linear.x = -0.04;
                t.angular.z = 0.25;
                cmd_vel_pub_->publish(t);
                micro_state_ = MicroState::IDLE;
                return;
            }
            // 否则持续前进
            geometry_msgs::msg::Twist t;
            t.linear.x = std::min(forward_speed_, linear_speed_max_);
            t.angular.z = 0.0;
            cmd_vel_pub_->publish(t);
            return;
        } else {
            // 完成此次微步前进，停止并进入 VERIFY 状态以检测效果
            micro_state_ = MicroState::VERIFY;
            state_start_time_ = this->now();
            geometry_msgs::msg::Twist t;
            t.linear.x = 0.0; t.angular.z = 0.0;
            cmd_vel_pub_->publish(t);
            return;
        }
    } else if (micro_state_ == MicroState::VERIFY) {
        // VERIFY：检查前方扇区内最小距离是否有显著改善
        sensor_msgs::msg::LaserScan::SharedPtr lastscan;
        {
            std::lock_guard<std::mutex> lock(scans_mutex_);
            if (!scans_deque_.empty()) lastscan = scans_deque_.back();
        }
        bool improved = false;
        if (lastscan) {
            // 计算以 current_target_angle_ 为中心扇区的最小距离
            double amin = lastscan->angle_min;
            double ainc = lastscan->angle_increment;
            int N = lastscan->ranges.size();
            double sector = 20.0 * M_PI / 180.0;
            double minr = 1e9;
            for (int i = 0; i < N; ++i) {
                double a = amin + i * ainc;
                if (std::fabs(angle_diff(a, current_target_angle_)) <= sector/2.0) {
                    float r = lastscan->ranges[i];
                    if (std::isfinite(r)) minr = std::min(minr, (double)r);
                }
            }
            // 若该扇区最小距离超过 safety_distance_ + margin，则认为情况已改善
            if (minr > safety_distance_ + 0.05) improved = true;
        }
        if (improved) {
            // 成功：回到 IDLE，等待下一次 escape 向量继续迭代或上层判断结束逃逸
            micro_state_ = MicroState::IDLE;
            return;
        } else {
            // 未改善：回到 IDLE 并执行小幅后退以改变姿态/视角，等待下一次尝试
            micro_state_ = MicroState::IDLE;
            geometry_msgs::msg::Twist t;
            t.linear.x = -0.04; t.angular.z = 0.16;
            cmd_vel_pub_->publish(t);
            return;
        }
    }

    // 如果状态仍为 IDLE（未被上面的任何分支处理）
    if (micro_state_ == MicroState::IDLE) {
        // 将 escape 向量投影为角度与强度
        double ang = std::atan2(escape_vec.y, escape_vec.x);
        double strength = std::hypot(escape_vec.x, escape_vec.y);
        // 若强度超过阈值，则以该角度为目标进入 TURNING；否则执行弱信号保底行为（后退+慢转）
        if (strength > escape_vector_threshold_) {
            current_target_angle_ = ang;
            micro_state_ = MicroState::TURNING;
            state_start_time_ = this->now();
        } else {
            // 信号弱：后退并旋转寻找新的视角
            geometry_msgs::msg::Twist t;
            t.linear.x = -0.03;
            t.angular.z = 0.15;
            cmd_vel_pub_->publish(t);
        }
    }
}

/*
  publish_escape_marker()
  - 可视化当前 escape 向量（绿色箭头），长度按向量模长缩放到 [0,1] 区间用于显示；
  - 该函数在调参和专利实验图中非常有用，可以直接截图放到论文/专利说明书的实施例中。
*/
void SmartEscape::publish_escape_marker(const Vec2 &v)
{
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "base_footprint";
    m.header.stamp = rclcpp::Time(0);
    m.ns = "escape";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::ARROW;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.04; // shaft diameter
    m.scale.y = 0.08;  // head diameter
    m.scale.z = 0.08;
    m.color.a = 1.0;
    m.color.r = 0.0;
    m.color.g = 1.0;
    m.color.b = 0.0;

    geometry_msgs::msg::Point p_start, p_end;
    p_start.x = 0.0; p_start.y = 0.0; p_start.z = 0.0;
    double len = std::hypot(v.x, v.y);
    if (len < 1e-6) {
        p_end.x = 0.0; p_end.y = 0.0;
    } else {
        // 将箭头长度缩放到 [0,1] 以便可视化（防止过长）
        double scale = std::min(len, 1.0);
        p_end.x = v.x / len * scale;
        p_end.y = v.y / len * scale;
    }
    m.points.clear();
    m.points.push_back(p_start);
    m.points.push_back(p_end);

    marker_pub_->publish(m);
}

/*
  saturate()
  - 简单的限幅函数，将 v 限定到 [vmin, vmax]。用于速度限幅等。
*/
double SmartEscape::saturate(double v, double vmin, double vmax)
{
    if (v > vmax) return vmax;
    if (v < vmin) return vmin;
    return v;
}

}  // namespace smart_escape
