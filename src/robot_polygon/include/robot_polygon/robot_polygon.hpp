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

#ifndef ROBOT_POLYGON__ROBOT_POLYGON_HPP_
#define ROBOT_POLYGON__ROBOT_POLYGON_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"

namespace robot_polygon
{
class RobotPolygon : public rclcpp::Node
{
public:
   RobotPolygon();
   ~RobotPolygon();

private:
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr robot_poly_pub_;
    
    void timer_callback();

    void publish_robot_circle();

    double robot_radius_{0.12};

};
}  // namespace robot_polygon
#endif  // ROBOT_POLYGON__ROBOT_POLYGON_HPP_
 