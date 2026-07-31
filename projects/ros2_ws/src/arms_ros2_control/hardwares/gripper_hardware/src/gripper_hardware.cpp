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

#include "gripper_hardware/gripper_hardware.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <rclcpp/executors.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace gripper_hardware
{

    CallbackReturn GripperHardware::on_init(const hardware_interface::HardwareInfo& info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
        {
            return CallbackReturn::ERROR;
        }

        // 初始化存储空间 - 只保留位置
        joint_positions_.resize(info_.joints.size(), 0.0);
        joint_position_commands_.resize(info_.joints.size(), 0.0);

        // 读取初始值
        for (auto i = 0u; i < info_.joints.size(); i++)
        {
            const auto& joint = info_.joints[i];

            // 设置位置初始值
            for (const auto& interface : joint.state_interfaces)
            {
                if (interface.name == hardware_interface::HW_IF_POSITION && !interface.initial_value.empty())
                {
                    joint_positions_[i] = std::stod(interface.initial_value);
                    if (!initialize_commands_from_state_)
                    {
                        joint_position_commands_[i] = std::stod(interface.initial_value);
                    }
                }
            }
        }

        // 读取硬件参数
        auto get_param = [this](const std::string& param_name, const std::string& default_value) -> std::string {
            if (auto it = info_.hardware_parameters.find(param_name); it != info_.hardware_parameters.end())
            {
                return it->second;
            }
            return default_value;
            };

        // 读取配置参数
        state_topic_ = get_param("state_topic", "/gripper_state");
        command_topic_ = get_param("command_topic", "/gripper_command");

        if (auto it = info_.hardware_parameters.find("initialize_commands_from_state"); it != info_.hardware_parameters.end())
        {
            initialize_commands_from_state_ = (it->second == "true");
        }

        // 创建ROS2节点
        rclcpp::NodeOptions options;
        options.arguments({ "--ros-args", "-r", "__node:=gripper_hardware_" + info_.name });
        node_ = rclcpp::Node::make_shared("_", options);

        RCLCPP_INFO(node_->get_logger(), "Initializing gripper hardware interface: %s", info_.name.c_str());
        RCLCPP_INFO(node_->get_logger(), "  State topic: %s", state_topic_.c_str());
        RCLCPP_INFO(node_->get_logger(), "  Command topic: %s", command_topic_.c_str());
        RCLCPP_INFO(node_->get_logger(), "  Initialize commands from state: %s",
            initialize_commands_from_state_ ? "true" : "false");

        // 创建发布者和订阅者
        command_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>(
            command_topic_, rclcpp::QoS(1));

        state_subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>(
            state_topic_, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                latest_state_ = *msg;
                state_received_ = true;
            });

        RCLCPP_INFO(node_->get_logger(), "Gripper hardware interface initialized successfully");
        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> GripperHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        for (auto i = 0u; i < info_.joints.size(); i++)
        {
            const auto& joint = info_.joints[i];

            for (const auto& interface : joint.state_interfaces)
            {
                if (interface.name == hardware_interface::HW_IF_POSITION)
                {
                    state_interfaces.emplace_back(
                        hardware_interface::StateInterface(joint.name, hardware_interface::HW_IF_POSITION, &joint_positions_[i]));
                }
            }
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> GripperHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        for (auto i = 0u; i < info_.joints.size(); i++)
        {
            const auto& joint = info_.joints[i];

            for (const auto& interface : joint.command_interfaces)
            {
                if (interface.name == hardware_interface::HW_IF_POSITION)
                {
                    command_interfaces.emplace_back(
                        hardware_interface::CommandInterface(joint.name, hardware_interface::HW_IF_POSITION, &joint_position_commands_[i]));
                }
            }
        }

        return command_interfaces;
    }

    hardware_interface::return_type GripperHardware::read(
        const rclcpp::Time& /*time*/,
        const rclcpp::Duration& /*period*/)
    {
        // 处理订阅的消息
        rclcpp::spin_some(node_);

        if (!state_received_)
        {
            // 如果还没有收到状态消息，返回OK但不更新状态
            return hardware_interface::return_type::OK;
        }

        // 从最新状态消息更新关节状态
        for (auto i = 0u; i < info_.joints.size(); i++)
        {
            const auto& joint_name = info_.joints[i].name;

            // 在接收到的消息中查找对应的关节
            auto it = std::find(latest_state_.name.begin(), latest_state_.name.end(), joint_name);
            if (it != latest_state_.name.end())
            {
                auto index = std::distance(latest_state_.name.begin(), it);

                // 更新位置
                if (index < static_cast<int>(latest_state_.position.size()))
                {
                    joint_positions_[i] = latest_state_.position[index];
                }
            }
        }

        // 如果启用了从状态初始化命令，并且还未初始化
        if (initialize_commands_from_state_ && !commands_initialized_)
        {
            for (auto i = 0u; i < info_.joints.size(); i++)
            {
                joint_position_commands_[i] = joint_positions_[i];
            }
            commands_initialized_ = true;
            RCLCPP_INFO(node_->get_logger(), "Commands initialized from first received state");
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type GripperHardware::write(
        const rclcpp::Time& /*time*/,
        const rclcpp::Duration& /*period*/)
    {
        // 直接发布命令，不进行阈值判断
        sensor_msgs::msg::JointState command_msg;
        command_msg.header.stamp = node_->now();

        command_msg.name.resize(info_.joints.size());
        command_msg.position.resize(info_.joints.size());

        // 填充命令消息 - 只包含位置
        for (auto i = 0u; i < info_.joints.size(); i++)
        {
            command_msg.name[i] = info_.joints[i].name;
            command_msg.position[i] = joint_position_commands_[i];
        }

        // 发布命令
        command_publisher_->publish(command_msg);

        return hardware_interface::return_type::OK;
    }

}  // namespace gripper_hardware

// 导出插件
PLUGINLIB_EXPORT_CLASS(gripper_hardware::GripperHardware, hardware_interface::SystemInterface)
