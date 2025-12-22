#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "smart_escape/smart_escape.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<smart_escape::SmartEscape>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
