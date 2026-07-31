#ifndef RM_HARDWARE__RM_SIXFORCE_HARDWARE_HPP_
#define RM_HARDWARE__RM_SIXFORCE_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "rm_ros_interfaces/msg/sixforce.hpp"
#include "realtime_tools/realtime_buffer.hpp"
// 一欧元滤波器
#include "rm_hardware/one_euro_filter.hpp"

namespace rm_hardware
{

    class RMSixForceHardware : public hardware_interface::SensorInterface
    {
        public:
        RCLCPP_SHARED_PTR_DEFINITIONS(RMSixForceHardware)

            hardware_interface::CallbackReturn on_init(
                const hardware_interface::HardwareInfo& info) override;

        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State& previous_state) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State& previous_state) override;

        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State& previous_state) override;

        hardware_interface::return_type read(
            const rclcpp::Time& time, const rclcpp::Duration& period) override;

        private:
        // Parameters
        std::string sensor_topic_;
        double sensor_timeout_;
        double update_rate_;

        // One Euro Filter
        std::vector<std::unique_ptr<OneEuroFilter>> filters_;
        double filter_freq_;
        double filter_min_cutoff_;
        double filter_beta_;
        double filter_d_cutoff_;

        // Force/Torque sensor data
        std::vector<std::string> sensor_names_;
        std::vector<double> hw_sensor_values_;

        // ROS 2 node and subscriber
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<rm_ros_interfaces::msg::Sixforce>::SharedPtr sixforce_subscriber_;

        // Realtime buffer
        realtime_tools::RealtimeBuffer<std::shared_ptr<rm_ros_interfaces::msg::Sixforce>> sixforce_buffer_;

        // Callback function
        void sixforce_callback(const rm_ros_interfaces::msg::Sixforce::SharedPtr msg);

        // Helper functions
        bool initialize_sensor_communication();
        void update_sensor_states();

        // Communication flags and timing
        bool sensor_connected_;
        std::chrono::steady_clock::time_point last_sensor_time_;
        std::chrono::steady_clock::time_point last_update_time_;

        // Data validation
        bool is_data_valid();

        // Deadzone and bias parameters
        double deadzone[6];
        double bias[6];
    };

}  // namespace rm_hardware

#endif  // RM_HARDWARE__RM_SIXFORCE_HARDWARE_HPP_
