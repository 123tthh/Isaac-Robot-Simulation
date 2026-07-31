#include "rm_hardware/rm_sixforce_hardware.hpp"
#include "rm_hardware/one_euro_filter.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rm_hardware
{

    hardware_interface::CallbackReturn RMSixForceHardware::on_init(
        const hardware_interface::HardwareInfo& info)
    {
        if (
            hardware_interface::SensorInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Initialize parameters with default values if not provided
        sensor_topic_ = info_.hardware_parameters.count("sensor_topic") ?
            info_.hardware_parameters["sensor_topic"] : "/rm_driver/get_tool_force_data_result";
        sensor_timeout_ = info_.hardware_parameters.count("sensor_timeout") ?
            std::stod(info_.hardware_parameters["sensor_timeout"]) : 1.0;
        update_rate_ = info_.hardware_parameters.count("update_rate") ?
            std::stod(info_.hardware_parameters["update_rate"]) : 100.0;

        // One Euro Filter parameters
        filter_freq_ = info_.hardware_parameters.count("filter_freq") ?
            std::stod(info_.hardware_parameters["filter_freq"]) : 150.0;
        filter_min_cutoff_ = info_.hardware_parameters.count("filter_min_cutoff") ?
            std::stod(info_.hardware_parameters["filter_min_cutoff"]) : 1.0;
        filter_beta_ = info_.hardware_parameters.count("filter_beta") ?
            std::stod(info_.hardware_parameters["filter_beta"]) : 0.1;
        filter_d_cutoff_ = info_.hardware_parameters.count("filter_d_cutoff") ?
            std::stod(info_.hardware_parameters["filter_d_cutoff"]) : 1.0;

        for (int i = 0; i < 6; i++)
        {
            deadzone[i] = info_.hardware_parameters.count("deadzone_" + std::to_string(i)) ?
                std::stod(info_.hardware_parameters["deadzone_" + std::to_string(i)]) : 0.0; // Default deadzone is 0.0
            bias[i] = info_.hardware_parameters.count("bias_" + std::to_string(i)) ?
                std::stod(info_.hardware_parameters["bias_" + std::to_string(i)]) : 0.0; // Default bias is 0.0
        }
        RCLCPP_INFO(
            rclcpp::get_logger("RMSixForceHardware"),
            "Initializing RM Six-axis Force/Torque Sensor with topic: %s", sensor_topic_.c_str());

        RCLCPP_INFO(
            rclcpp::get_logger("RMSixForceHardware"),
            "Sensor parameters - Timeout: %.2fs, Update rate: %.1fHz", sensor_timeout_, update_rate_);

        RCLCPP_INFO(
            rclcpp::get_logger("RMSixForceHardware"),
            "One Euro Filter parameters - Freq: %.1f, MinCutoff: %.1f, Beta: %.1f, DCutoff: %.1f",
            filter_freq_, filter_min_cutoff_, filter_beta_, filter_d_cutoff_);

        // Log deadzone and bias parameters
        RCLCPP_INFO(
            rclcpp::get_logger("RMSixForceHardware"),
            "Deadzone: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
            deadzone[0], deadzone[1], deadzone[2], deadzone[3], deadzone[4], deadzone[5]);

        RCLCPP_INFO(
            rclcpp::get_logger("RMSixForceHardware"),
            "Bias: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
            bias[0], bias[1], bias[2], bias[3], bias[4], bias[5]);

        // Initialize sensor data arrays
        sensor_names_ = { "force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z" };
        hw_sensor_values_.resize(6);

        // Validate sensor configuration
        if (info_.sensors.size() != 1)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("RMSixForceHardware"),
                "Expected exactly 1 sensor, but got %zu sensors in URDF", info_.sensors.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        const auto& sensor = info_.sensors[0];
        RCLCPP_INFO(
            rclcpp::get_logger("RMSixForceHardware"),
            "Configuring sensor '%s' of type '%s'", sensor.name.c_str(), sensor.type.c_str());

        if (sensor.state_interfaces.size() != 6)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("RMSixForceHardware"),
                "Sensor '%s' has %zu state interfaces. 6 expected (fx, fy, fz, mx, my, mz).",
                sensor.name.c_str(), sensor.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Validate interface names
        std::vector<std::string> expected_interfaces = { "force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z" };
        for (size_t i = 0; i < sensor.state_interfaces.size(); ++i)
        {
            if (sensor.state_interfaces[i].name != expected_interfaces[i])
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("RMSixForceHardware"),
                    "Sensor interface %zu has name '%s'. Expected '%s'.",
                    i, sensor.state_interfaces[i].name.c_str(), expected_interfaces[i].c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RMSixForceHardware::on_configure(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        // Create ROS 2 node for communication
        node_ = rclcpp::Node::make_shared("rm_sixforce_hardware_node");

        RCLCPP_INFO(rclcpp::get_logger("RMSixForceHardware"), "Configuring RM Six-axis Force/Torque Sensor...");

        // Create subscriber for six-axis force/torque data
        rclcpp::QoS qos(10);
        sixforce_subscriber_ = node_->create_subscription<rm_ros_interfaces::msg::Sixforce>(
            sensor_topic_, qos,
            std::bind(&RMSixForceHardware::sixforce_callback, this, std::placeholders::_1));

        // Initialize sensor values to zero
        for (size_t i = 0; i < 6; ++i) {
            hw_sensor_values_[i] = 0.0;
            RCLCPP_INFO(rclcpp::get_logger("RMSixForceHardware"), "Sensor value %zu (%s): initialized to 0.0",
                i, sensor_names_[i].c_str());
        }

        // Initialize One Euro Filters
        filters_.reserve(6);
        for (int i = 0; i < 6; ++i) {
            filters_.emplace_back(std::make_unique<OneEuroFilter>(
                filter_freq_, filter_min_cutoff_, filter_beta_, filter_d_cutoff_));
        }

        sensor_connected_ = initialize_sensor_communication();

        RCLCPP_INFO(rclcpp::get_logger("RMSixForceHardware"), "Successfully configured RM Six-axis Force/Torque Sensor!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> RMSixForceHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        const auto& sensor = info_.sensors[0];
        for (size_t i = 0; i < sensor.state_interfaces.size(); ++i)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                sensor.name, sensor.state_interfaces[i].name, &hw_sensor_values_[i]));
        }

        return state_interfaces;
    }

    hardware_interface::CallbackReturn RMSixForceHardware::on_activate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("RMSixForceHardware"), "Activating RM Six-axis Force/Torque Sensor...");

        // Reset sensor values
        for (auto& value : hw_sensor_values_) {
            value = 0.0;
        }

        last_sensor_time_ = std::chrono::steady_clock::now();
        last_update_time_ = std::chrono::steady_clock::now();

        RCLCPP_INFO(rclcpp::get_logger("RMSixForceHardware"), "Successfully activated RM Six-axis Force/Torque Sensor!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RMSixForceHardware::on_deactivate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("RMSixForceHardware"), "Deactivating RM Six-axis Force/Torque Sensor...");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type RMSixForceHardware::read(
        const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
    {
        // Update sensor states from subscriber data
        update_sensor_states();

        // Spin the node to process callbacks
        rclcpp::spin_some(node_);

        // Check for sensor timeout
        auto current_time = std::chrono::steady_clock::now();
        auto time_since_last_sensor = std::chrono::duration_cast<std::chrono::milliseconds>(
            current_time - last_sensor_time_).count();

        if (time_since_last_sensor > sensor_timeout_ * 1000.0) {
            RCLCPP_WARN_THROTTLE(
                rclcpp::get_logger("RMSixForceHardware"),
                *node_->get_clock(),
                5000,  // 5 seconds throttle
                "No sensor data received for %.2f seconds. Check if %s topic is publishing.",
                time_since_last_sensor / 1000.0, sensor_topic_.c_str());
        }

        return hardware_interface::return_type::OK;
    }

    void RMSixForceHardware::sixforce_callback(const rm_ros_interfaces::msg::Sixforce::SharedPtr msg)
    {
        sixforce_buffer_.writeFromNonRT(msg);
        last_sensor_time_ = std::chrono::steady_clock::now();
    }

    bool RMSixForceHardware::initialize_sensor_communication()
    {
        // Simulate successful initialization
        RCLCPP_INFO(rclcpp::get_logger("RMSixForceHardware"), "Simulating sensor communication initialization...");
        return true;
    }

    void RMSixForceHardware::update_sensor_states()
    {
        auto sixforce_msg = sixforce_buffer_.readFromRT();
        if (!sixforce_msg || !(*sixforce_msg)) {
            return;
        }

        std::vector<double> raw_values = {
            (*sixforce_msg)->force_fx,
            (*sixforce_msg)->force_fy,
            (*sixforce_msg)->force_fz,
            (*sixforce_msg)->force_mx,
            (*sixforce_msg)->force_my,
            (*sixforce_msg)->force_mz
        };


        for (int i = 0; i < 6; ++i) {
            double processed_value = raw_values[i] - bias[i];
            if (std::abs(processed_value) < deadzone[i]) {
                processed_value = 0.0;
            }
            hw_sensor_values_[i] = filters_[i]->filter(processed_value);
        }
    }

    bool RMSixForceHardware::is_data_valid()
    {
        // Check if all sensor values are within reasonable ranges
        for (size_t i = 0; i < hw_sensor_values_.size(); ++i) {
            if (std::isnan(hw_sensor_values_[i]) || std::isinf(hw_sensor_values_[i])) {
                RCLCPP_WARN(
                    rclcpp::get_logger("RMSixForceHardware"),
                    "Invalid sensor value detected at index %zu (%s): %f",
                    i, sensor_names_[i].c_str(), hw_sensor_values_[i]);
                return false;
            }
        }

        return true;
    }

}  // namespace rm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    rm_hardware::RMSixForceHardware, hardware_interface::SensorInterface)
