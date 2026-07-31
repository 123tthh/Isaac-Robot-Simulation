#ifndef RM_HARDWARE__RM_ROS2_CONTROL_HARDWARE_HPP_
#define RM_HARDWARE__RM_ROS2_CONTROL_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "rm_ros_interfaces/msg/jointpos.hpp"
#include "rm_ros_interfaces/msg/armstate.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"

namespace rm_hardware
{

    class RMRos2ControlHardware : public hardware_interface::SystemInterface
    {
        public:
        RCLCPP_SHARED_PTR_DEFINITIONS(RMRos2ControlHardware)

            hardware_interface::CallbackReturn on_init(
                const hardware_interface::HardwareInfo& info) override;

        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State& previous_state) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State& previous_state) override;

        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State& previous_state) override;

        hardware_interface::return_type read(
            const rclcpp::Time& time, const rclcpp::Duration& period) override;

        hardware_interface::return_type write(
            const rclcpp::Time& time, const rclcpp::Duration& period) override;

        private:
        // Parameters
        int arm_dof_;
        bool follow_;
        std::string robot_ip_;
        int robot_port_;
        double command_timeout_;
        double publish_rate_;
        std::string joint_command_topic_;
        std::string joint_state_topic_;

        // Joint data
        std::vector<double> hw_commands_;
        std::vector<double> hw_positions_;
        std::vector<double> hw_velocities_;
        std::vector<double> hw_efforts_;
        std::vector<double> prev_commands_;
        std::vector<double> initial_positions_;  // Store initial positions from URDF
        std::vector<double> prev_positions_;      // 上一次读取的关节位置（用于速度计算）
        std::vector<std::string> joint_names_;        // ROS 2 node and publishers/subscribers
        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<rm_ros_interfaces::msg::Jointpos>::SharedPtr joint_pos_publisher_;
        rclcpp::Subscription<rm_ros_interfaces::msg::Armstate>::SharedPtr arm_state_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;

        // Realtime buffers
        realtime_tools::RealtimeBuffer<std::vector<double>> position_command_buffer_;
        realtime_tools::RealtimeBuffer<std::shared_ptr<sensor_msgs::msg::JointState>> joint_state_buffer_;

        // Callback functions
        void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

        // Helper functions
        bool initialize_robot_communication();
        void send_joint_commands();
        void update_joint_states();

        // Communication flags and timing
        bool robot_connected_;
        std::chrono::steady_clock::time_point last_command_time_;
        std::chrono::steady_clock::time_point last_state_time_;
        // 用于速度计算的上一次状态时间戳（与 last_state_time_ 分离，避免未更新情况下重复计算）
        std::chrono::steady_clock::time_point prev_state_time_;

        // Joint message for publishing
        rm_ros_interfaces::msg::Jointpos joint_msg_;
    };

}  // namespace rm_hardware

#endif  // RM_HARDWARE__RM_ROS2_CONTROL_HARDWARE_HPP_
