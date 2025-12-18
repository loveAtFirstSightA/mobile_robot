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

#ifndef AMR_PURE_PURSUIT__AMR_PURE_PURSUIT_HPP_
#define AMR_PURE_PURSUIT__AMR_PURE_PURSUIT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "spdlog/spdlog.h"
#include "amr_msgs/srv/pure_pursuit.hpp"

namespace amr_pure_pursuit
{
class AmrPurePursuit : public rclcpp::Node
{
public:
    AmrPurePursuit();
    ~AmrPurePursuit();

private:
    void pure_pursuit_ser_callback(
     const std::shared_ptr<amr_msgs::srv::PurePursuit::Request> request,
     std::shared_ptr<amr_msgs::srv::PurePursuit::Response> response);

    rclcpp::Service<amr_msgs::srv::PurePursuit>::SharedPtr pure_pursuit_ser_;


};
}  // namespace amr_pure_pursuit
#endif  // AMR_PURE_PURSUIT__AMR_PURE_PURSUIT_HPP_
