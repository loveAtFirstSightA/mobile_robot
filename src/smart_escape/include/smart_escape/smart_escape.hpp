#ifndef SMART_ESCAPE__SMART_ESCAPE_HPP_
#define SMART_ESCAPE__SMART_ESCAPE_HPP_

/*
  smart_escape.hpp — 头文件

  说明（概要）：
  本头文件声明了 SmartEscape 类，用于在机器人被障碍物包围或局部卡死时，
  基于“环境趋势流场（Trend Flow Field）”的自演化脱困策略进行微步脱困。
  注释面向专利/论文写作：每个参数、函数与状态机均给出工程目的、技术效果与调参建议。
*/

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/empty.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <deque>
#include <vector>
#include <cmath>
#include <mutex>
#include <optional>
#include <visualization_msgs/msg/marker.hpp>
#include <chrono>

namespace smart_escape
{

/*
  SmartEscape 类（声明概要）
  - 功能：在局部路径规划失败或机器人被包围时，依据历时激光数据构建局部“趋势场”，
          以微步（turn->forward->verify）方式逐步脱困。
  - 设计原则：
      1) 尽量少依赖全局地图（降低对 SLAM 的依赖），使用多帧激光的时间维度信息判断“可通行方向”；
      2) 使用角向滤波与时间低通抑制噪声，采用中心差分（更稳健的 dr/dt）估计；
      3) 用状态机（TURNING/FORWARDING/VERIFY）保证运动安全性与鲁棒性；
      4) 保留丰富可调参数以便工程调试与论文/专利中写明实施例的参数集合。
*/
class SmartEscape : public rclcpp::Node
{
public:
    SmartEscape();
    ~SmartEscape();

private:
    // ----------------------------
    // ROS2 核心通信组件（发布/订阅/定时器）
    // ----------------------------
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr robot_poly_pub_;
    /*
      robot_poly_pub_：
      - 发布机器人形状（圆形近似）的 PolygonStamped，通常用于 RViz 可视化机器人体积/安全边界。
      - 专利/论文写法中可说明：用于可视化与碰撞预估（发布 robot radius 的多边形近似）。
    */

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    /*
      cmd_vel_pub_：
      - 向底层运动控制器下发速度命令（/cmd_vel）。
      - 在实现中采用微步状态机控制下发短时速度，保障执行可回退。
    */

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    /*
      scan_sub_：
      - 订阅激光雷达数据，用于构建多帧趋势（Trend Flow Field）。
      - 注意：假设激光以角度索引对齐（若不同设备角度分辨率不同，请在实现里做插值或重采样）。
    */

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr escape_trigger_sub_;
    /*
      escape_trigger_sub_：
      - 触发逃逸模式的 topic（/trigger_escape），当上层检测到 stuck 或被包围时发布触发消息。
      - 在工程上建议上层或状态监控器触发该主题，而不是持续开启，以避免误触发。
    */

    rclcpp::TimerBase::SharedPtr escape_timer_;
    /*
      escape_timer_：
      - 定时器回调用于周期性计算趋势向量并下发速度命令（本实现为 25ms）。
      - 设计为非阻塞：每次回调只发短时/小幅度速度，不做阻塞等待。
    */

    // 可视化逃逸向量（marker）
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    /*
      marker_pub_：
      - 发布可视化箭头（escape_marker），便于在 RViz 中观察趋势向量与调参。
      - 在专利图示/实验部分可截图展示该箭头与激光点云之间的关系。
    */

    // TF 组件（备用，当前实现未直接调用 tf，但保留以便扩展）
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    /*
      tf_buffer_, tf_listener_：
      - 保留以便未来若需将 escape 向量映射到其他坐标系（例如 odom 或 map）时使用。
    */

    // 机器人的近似半径（m）
    double robot_radius_ = 0.15;
    /*
      robot_radius_：
      - 用于绘制 robot polygon 以及用于安全距离估计参考。
      - 在专利实施例中应写明如何根据真实机器人尺寸调整该参数（通常略大于实际半径以留安全余量）。
    */

    // 逃逸模式激活开关
    bool escape_active_ = false;


    // =========================
    // ===== 参数（可调） =====
    // =========================
    // 这些参数均为工程可调参；在专利说明中建议给出推荐数值范围和调参理由。
    size_t trend_frame_count_ = 7;            // 用多少帧构建趋势（N，应为奇数以便中心差分）
    /*
      trend_frame_count_：
      - 趋势计算所需帧数，建议设置为奇数（例如 5/7/9），以便采用中心差分（使用 mid-1 和 mid+1）。
      - 越大：噪声抑制能力增强，但对动态变化的响应变慢；越小：响应快但噪声更多。
      - 专利写法：可声明一个实现例（如7帧、激光频率10Hz，对应时间窗约0.6s）。
    */

    double trend_dt_default_ = 0.1;           // 采样帧间预估 dt（s）
    /*
      trend_dt_default_：
      - 当激光时间戳不可靠或间隔太小/为0时使用的默认 dt。
      - 在专利中说明：可结合实际激光帧率（例如10Hz -> 0.1s）设定。
    */

    int ang_smooth_window_ = 5;               // 角向平滑窗口（奇数）
    /*
      ang_smooth_window_：
      - 对单帧的 range 做角向移动平均（窗口大小），用于抑制单束噪声或反射异常。
      - 较大窗口会平滑更多，但可能模糊较细缝隙；工程建议：对 360/1080 beam lidar 分别使用 5~9 不等。
    */

    double time_alpha_ = 0.3;                 // dr/dt 时间低通系数 (0..1)
    /*
      time_alpha_：
      - 对每个角度的 dr/dt 做指数平滑： dr_filtered = alpha*dr_dt + (1-alpha)*dr_filtered.
      - alpha 越大：对瞬时变化敏感，越小：更平滑。
      - 用于平衡：响应速度 vs 抗噪能力。
    */

    double forward_speed_ = 0.05;             // 前进速度 (m/s)
    double angular_speed_max_ = 0.8;          // 最大角速度 (rad/s)
    double linear_speed_max_ = 0.08;          // 最大线速度
    /*
      forward_speed_, angular_speed_max_, linear_speed_max_：
      - 控制机器人微步移动（TURN/FORWARD）时的速度上限。为安全起见默认值较小。
      - 专利中应明确：这些值需根据机器人动力学和环境（地面摩擦、载重）调节。
    */

    double weight_distance_exponent_ = 1.2;   // 距离权重指数
    /*
      weight_distance_exponent_：
      - 对距离的加权使用 1/r^p 形式，p 为该指数。p 越大，近障碍影响越大。
      - 在权重累加时，使近障碍对逃逸向量贡献更显著，避免机器人朝近处小缝冲撞。
    */

    double escape_vector_threshold_ = 0.04;   // 逃逸向量模长阈值
    /*
      escape_vector_threshold_：
      - 当计算出的逃逸向量强度小于该阈值时，认为信号弱，进入后退/摆动等保底策略。
      - 专利写法里可以把该阈值与激光精度、滤波参数关联说明。
    */

    double safety_distance_ = 0.22;           // 前方安全最小距离（m）
    /*
      safety_distance_：
      - 前向检查所需的最小安全距离（microstep 前需保证该区间内无障碍）。
      - 典型值略大于 robot_radius_，以留出冗余缓冲（例如 robot_radius 0.15 -> safety 0.22）。
    */

    double microstep_distance_ = 0.18;        // 每次前进微步长度（m）
    /*
      microstep_distance_：
      - 每次 FORWARD 动作前进距离（通过持续发送指定时长的速度命令实现）。
      - 太大可能造成碰撞，太小会导致脱困过慢。专利实施例给出区间（0.1~0.25m）。
    */

    double turn_tolerance_rad_ = 0.12;        // 角度容忍度（rad）用于微步状态机
    /*
      turn_tolerance_rad_：
      - 认为机器人已朝向目标角度的角度误差阈值（rad），用于从 TURNING 转到 FORWARDING。
      - 典型值 0.1~0.2 rad（约 6~12 度）。
    */

    double hysteresis_angle_deg_ = 20.0;      // 保持旧方向的角度滞回阈（度）
    /*
      hysteresis_angle_deg_：
      - 当新计算出的逃逸角度与上一次相差很大，但新信号强度又很弱时，使用滞回策略保持旧方向以抑制振荡。
      - 在专利中属于“稳定性增强”措施的技术特征。
    */


    // ----------------------------
    // Scan 缓冲区与线程锁（保护 scans_deque_）
    // ----------------------------
    std::deque<sensor_msgs::msg::LaserScan::SharedPtr> scans_deque_;
    std::mutex scans_mutex_;
    /*
      scans_deque_：
      - 保存最近 trend_frame_count_ 帧的 LaserScan，用于中心差分计算（使用 mid-1 和 mid+1）。
      - 采用 deque 保证插入/弹出高效，使用 mutex 保护多线程并发访问。
    */


    // ----------------------------
    // internal state & 算法函数声明
    // ----------------------------
    struct Vec2 { double x; double y; };
    /*
      Vec2：
      - 简单二维向量结构（局部坐标系 base_footprint）。
      - x 对应机器人前方方向，y 对应左侧方向（遵循 ROS 右手坐标：x 前，y 左）。
    */

    std::optional<Vec2> compute_escape_vector_from_trend();
    /*
      compute_escape_vector_from_trend():
      - 主要算法函数：基于 scans_deque_ 计算每个角度的 dr/dt（中心差分 + 时间滤波），
        将 dr 的符号与角度映射为“逃逸/避障方向”，按权重融合得到单一 escape 向量。
      - 返回 std::nullopt 表示数据不足或没有可信逃逸向量。
      - 此函数的实现细节在 cpp 文件中，注释会解释每一步的原因与物理意义（专利必需）。
    */

    void execute_micro_step(const Vec2 &escape_vec);
    /*
      execute_micro_step():
      - 将 escape_vec 转换为具体动作（由微步状态机 TURNING -> FORWARDING -> VERIFY 执行）。
      - 状态机保证：先对齐再前进，再验证环境是否改善；若未改善则退回 IDLE 并尝试其他策略。
      - 这样做是为了解决直接并行调速（同时转向与前进）在稠密障碍环境中可能导致的碰撞风险。
    */

    void publish_escape_marker(const Vec2 &v);
    /*
      publish_escape_marker():
      - 发布用于调试的可视化 arrow（RViz），显示当前 escape 向量方向与幅值（通过箭头长度表示）。
    */


    // ----------------------------
    // 微步状态机（用于把 escape 向量安全地转成动作）
    // ----------------------------
    enum class MicroState { IDLE, TURNING, FORWARDING, VERIFY };
    MicroState micro_state_ = MicroState::IDLE;
    /*
      micro_state_：
      - IDLE：未开始微步或正在等待新的 escape 向量；
      - TURNING：以角速度修正朝向至 target angle；
      - FORWARDING：在确认前方安全的情况下以线速度前进 microstep_distance_；
      - VERIFY：停止并检查前方是否有明显改善（例如 min range 增大），若改善则继续下一轮，否则回退或重新规划。
      - 该设计有助于分阶段验证并避免一次性大动作导致的失败。
    */

    double current_target_angle_ = 0.0;   // rad in base_footprint
    double last_escape_angle_ = 0.0;      // keep last for hysteresis
    rclcpp::Time state_start_time_;
    /*
      current_target_angle_ / last_escape_angle_ / state_start_time_：
      - 用于状态机控制、滞回判断与微步时序。
      - 在专利中可声称这三者共同构成“微步自适应执行模块”的关键实现要素。
    */


    // ----------------------------
    // 回调函数声明（订阅/定时器）
    // ----------------------------
    void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    /*
      on_scan():
      - 将收到的 LaserScan 推入 scans_deque_。
      - 仅做缓冲与必要的边界处理（不在回调内进行复杂计算，以避免阻塞订阅线程）。
    */

    void on_escape_trigger(const std_msgs::msg::Empty::SharedPtr msg);
    /*
      on_escape_trigger():
      - 外部触发逃逸流程（上层检测器或操作者发布 /trigger_escape）。
      - 将 escape_active_ 置为 true，触发定时器中的逃逸逻辑启动。
    */

    void escape_timer_callback();
    /*
      escape_timer_callback():
      - 定时器回调核心入口，周期性调用 compute_escape_vector_from_trend() 并
        将得到的逃逸向量送入微步执行器 execute_micro_step()。
      - 若 compute 返回空值，执行保底行为（如慢速后退或旋转）以尝试获得更多可观测数据。
    */

    void publish_robot_circle();
    /*
      publish_robot_circle():
      - 发布一个近似机器人外形的 PolygonStamped（N 边近似圆）。
      - 主要用于 RViz 可视化与碰撞分析（便于调参与专利附图）。
    */


    // ----------------------------
    // 工具函数声明
    // ----------------------------
    double saturate(double v, double vmin, double vmax);
    /*
      saturate：
      - 将变量限制在 [vmin, vmax] 范围内，常用于速度限幅。
    */

    std::vector<double> angular_smooth(const std::vector<double>& in, int window);
    /*
      angular_smooth：
      - 对单帧 range 数组沿角度维度做移动平均（考虑 wrap-around），用于抑制单点噪声。
      - 在专利中可以写成“角向卷积滤波模块”并给出窗口大小对性能的影响。
    */

    double angle_diff(double a, double b); // normalized difference a-b in [-pi,pi]
    /*
      angle_diff：
      - 计算角差并归到 [-pi, pi]，用于角度控制和滞回判断。
    */

    bool check_forward_clear(const sensor_msgs::msg::LaserScan::SharedPtr &scan, double dir_angle, double min_dist);
    /*
      check_forward_clear：
      - 检查以 dir_angle 为中心的扇区内，所有测距是否大于 min_dist（前方是否安全）。
      - 实现细节在 cpp 中：选择扇区宽度（例如 20°）并根据机器人尺寸计算对应索引区间。
    */


    // ----------------------------
    // dr/dt 时间滤波存储（按角度）
    // ----------------------------
    std::vector<double> dr_filtered_;  // same length as last scan ranges
    bool initialized_dr_ = false;
    /*
      dr_filtered_：
      - 每个角度分量的时间低通滤波结果（dr/dt）。
      - 初始时未初始化，compute 函数会根据 scan 长度分配并置为 0。
      - 专利中可将该数组作为“时间记忆模块”的实现细节描述。
    */
};

}  // namespace smart_escape

#endif  // SMART_ESCAPE__SMART_ESCAPE_HPP_
