#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "simple_follow_edge/pid_controller.hpp"
#include <limits>

class SimpleFollowEdgeNode : public rclcpp::Node
{
public:
    SimpleFollowEdgeNode()
        : Node("simple_follow_edge_node"),
          pid_controller_(1.0, 0.0, 0.1, 1.0)  // PID参数需要根据实际情况调整
    {
        // 初始化订阅者
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&SimpleFollowEdgeNode::laser_callback, this, std::placeholders::_1));

        // 初始化发布者
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

private:
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // 获取前方和右侧的距离
        double front_distance = get_front_distance(msg);
        double right_distance = get_right_distance(msg);

        // 如果前方有障碍物，则停止或减速
        if (front_distance < 0.5)  // 假设0.5米内认为前方有障碍物
        {
            RCLCPP_WARN(this->get_logger(), "Obstacle detected in front! Stopping.");
            stop_robot();
        }
        else
        {
            // 使用 PID 控制器根据右侧的距离调整机器人角速度
            double control_signal = pid_controller_.compute(right_distance);

            // 发布速度命令
            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.linear.x = 0.2;  // 固定线速度，可以根据实际情况调整
            cmd_vel.angular.z = control_signal;

            cmd_vel_pub_->publish(cmd_vel);
        }
    }

    // 获取激光雷达数据中与前方相关的距离（0到30度）
    double get_front_distance(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        size_t start_index = static_cast<size_t>((msg->angle_min - 15) / msg->angle_increment);
        size_t end_index = static_cast<size_t>((msg->angle_min + 15) / msg->angle_increment);

        double min_distance = std::numeric_limits<double>::infinity();

        // 从前方区域计算最小距离
        for (size_t i = start_index; i <= end_index; ++i)
        {
            if (msg->ranges[i] < min_distance)
            {
                min_distance = msg->ranges[i];
            }
        }

        return min_distance;
    }

    // 获取激光雷达数据中与右侧相关的距离（90到270度）
    double get_right_distance(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        size_t start_index = static_cast<size_t>((msg->angle_min - 80) / msg->angle_increment);
        size_t end_index = static_cast<size_t>((msg->angle_min - 100) / msg->angle_increment);

        double min_distance = std::numeric_limits<double>::infinity();

        // 从右侧区域计算最小距离
        for (size_t i = start_index; i <= end_index; ++i)
        {
            if (msg->ranges[i] < min_distance)
            {
                min_distance = msg->ranges[i];
            }
        }

        return min_distance;
    }

    // 停止机器人
    void stop_robot()
    {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd_vel);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    PIDController pid_controller_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleFollowEdgeNode>());
    rclcpp::shutdown();
    return 0;
}
