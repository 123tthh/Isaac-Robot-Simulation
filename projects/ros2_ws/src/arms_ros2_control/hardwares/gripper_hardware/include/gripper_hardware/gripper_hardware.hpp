// Copyright 2024
//
// Licensed under the MIT License.
// You may obtain a copy of the License at
//
//     https://opensource.org/licenses/MIT
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

namespace gripper_hardware
{
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    /**
     * @brief 夹爪硬件接口类
     *
     * 该类实现了基于ROS2话题的夹爪硬件接口，可以:
     * - 通过话题订阅夹爪状态信息
     * - 通过话题发布夹爪控制指令
     *
     * 支持的接口类型:
     * - position (位置)
     */
    class GripperHardware : public hardware_interface::SystemInterface
    {
        public:
        GripperHardware() = default;
        virtual ~GripperHardware() = default;

        /**
         * @brief 初始化硬件接口
         *
         * 从硬件信息中读取配置参数:
         * - state_topic: 状态订阅话题 (默认: /gripper_state)
         * - command_topic: 命令发布话题 (默认: /gripper_command)
         */
        CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

        /**
         * @brief 导出状态接口
         */
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        /**
         * @brief 导出命令接口
         */
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        /**
         * @brief 读取硬件状态
         *
         * 从订阅的话题更新关节状态
         */
        hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

        /**
         * @brief 写入硬件命令
         *
         * 将命令发布到控制话题
         */
        hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

        private:
        // ROS2 节点
        rclcpp::Node::SharedPtr node_;

        // 订阅者和发布者
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_subscriber_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_publisher_;

        // 最新接收到的状态
        sensor_msgs::msg::JointState latest_state_;
        bool state_received_ = false;

        // 关节位置状态和命令存储
        std::vector<double> joint_positions_;
        std::vector<double> joint_position_commands_;

        // 配置参数
        bool initialize_commands_from_state_ = true;  // 是否从第一次状态初始化命令
        bool commands_initialized_ = false;

        // 话题名称
        std::string state_topic_;
        std::string command_topic_;
    };

}  // namespace gripper_hardware
