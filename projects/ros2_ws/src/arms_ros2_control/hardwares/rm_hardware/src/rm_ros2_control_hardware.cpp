#include "rm_hardware/rm_ros2_control_hardware.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rm_hardware
{

    hardware_interface::CallbackReturn RMRos2ControlHardware::on_init(
        const hardware_interface::HardwareInfo& info)
    {
        if (
            hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Initialize parameters with default values if not provided
        arm_dof_ = info_.hardware_parameters.count("arm_dof") ?
            std::stoi(info_.hardware_parameters["arm_dof"]) : 7;
        follow_ = info.hardware_parameters.count("follow") ?
            (info_.hardware_parameters["follow"] == "true") : false;
        robot_ip_ = info_.hardware_parameters.count("robot_ip") ?
            info_.hardware_parameters["robot_ip"] : "192.168.1.18";
        robot_port_ = info_.hardware_parameters.count("robot_port") ?
            std::stoi(info_.hardware_parameters["robot_port"]) : 8080;
        command_timeout_ = info_.hardware_parameters.count("command_timeout") ?
            std::stod(info_.hardware_parameters["command_timeout"]) : 0.5;
        publish_rate_ = info_.hardware_parameters.count("publish_rate") ?
            std::stod(info_.hardware_parameters["publish_rate"]) : 150.0; // 150Hz default ,高跟随模式，低于10ms
        joint_command_topic_ = info_.hardware_parameters.count("joint_command_topic") ?
            info_.hardware_parameters["joint_command_topic"] : "/rm_driver/movej_canfd_cmd";
        joint_state_topic_ = info_.hardware_parameters.count("joint_state_topic") ?
            info_.hardware_parameters["joint_state_topic"] : "/rm_driver/joint_states";

        RCLCPP_INFO(
            rclcpp::get_logger("RMRos2ControlHardware"),
            "Initializing RM Robot with %d DOF, follow=%s", arm_dof_, follow_ ? "true" : "false");

        RCLCPP_INFO(
            rclcpp::get_logger("RMRos2ControlHardware"),
            "Joint command topic: %s", joint_command_topic_.c_str());

        RCLCPP_INFO(
            rclcpp::get_logger("RMRos2ControlHardware"),
            "Joint state topic: %s", joint_state_topic_.c_str());

        // Initialize joint related variables
        joint_names_.resize(arm_dof_);
        hw_positions_.resize(arm_dof_);
        hw_velocities_.resize(arm_dof_);
        hw_efforts_.resize(arm_dof_);
        hw_commands_.resize(arm_dof_);
        prev_commands_.resize(arm_dof_);
        initial_positions_.resize(arm_dof_);
        prev_positions_.resize(arm_dof_);

        // First, populate joint names
        for (size_t i = 0; i < info_.joints.size(); ++i)
        {
            joint_names_[i] = info_.joints[i].name;
            RCLCPP_INFO(
                rclcpp::get_logger("RMRos2ControlHardware"),
                "Joint %zu: %s", i, joint_names_[i].c_str());
        }

        // Read initial positions from joint configurations
        for (size_t i = 0; i < info_.joints.size(); ++i)
        {
            const auto& joint = info_.joints[i];
            // Find the position command interface to get initial value
            for (const hardware_interface::InterfaceInfo& command_interface : joint.command_interfaces)
            {
                if (command_interface.name == hardware_interface::HW_IF_POSITION)
                {
                    double initial_value = 0.0;
                    if (!command_interface.initial_value.empty())
                    {
                        initial_value = std::stod(command_interface.initial_value);
                    }
                    initial_positions_[i] = initial_value;
                    RCLCPP_INFO(
                        rclcpp::get_logger("RMRos2ControlHardware"),
                        "Joint '%s' initial position: %.4f", joint.name.c_str(), initial_value);
                    break;
                }
            }
        }

        // Validate joint configuration
        if (info_.joints.size() != static_cast<size_t>(arm_dof_))
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("RMRos2ControlHardware"),
                "Expected %d joints, but got %zu joints in URDF", arm_dof_, info_.joints.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        for (const hardware_interface::ComponentInfo& joint : info_.joints)
        {
            if (joint.command_interfaces.size() != 1)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("RMRos2ControlHardware"),
                    "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                    joint.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("RMRos2ControlHardware"),
                    "Joint '%s' has '%s' command interface. '%s' expected.", joint.name.c_str(),
                    joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::CallbackReturn::ERROR;
            }


            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("RMRos2ControlHardware"),
                    "Joint '%s' has '%s' as first state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("RMRos2ControlHardware"),
                    "Joint '%s' has '%s' as second state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RMRos2ControlHardware::on_configure(
        const rclcpp_lifecycle::State&)
    {
        // 创建ros2 node节点
        node_ = rclcpp::Node::make_shared("rm_ros2_control_hardware_node");

        RCLCPP_INFO(rclcpp::get_logger("RMRos2ControlHardware"), "Configuring RM Hardware...");
        rclcpp::QoS qos(10);
        joint_pos_publisher_ = node_->create_publisher<rm_ros_interfaces::msg::Jointpos>(
            joint_command_topic_, qos);

        joint_state_subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>(
            joint_state_topic_, qos,
            std::bind(&RMRos2ControlHardware::joint_state_callback, this, std::placeholders::_1));

        // Initialize joint names and states; commands 用 NaN 避免启动时发送有效指令导致机械臂跳动
        const double kNaN = std::numeric_limits<double>::quiet_NaN();
        for (size_t i = 0; i < static_cast<size_t>(arm_dof_); ++i) {
            joint_names_[i] = info_.joints[i].name;

            // 初始状态设为 0（将由反馈更新），命令设为 NaN 表示“尚未收到控制器指令”
            hw_positions_[i] = 0.0;
            hw_velocities_[i] = 0.0;
            hw_efforts_[i] = 0.0;
            hw_commands_[i] = kNaN;
            prev_commands_[i] = kNaN;  // 保持与当前命令一致，使首轮 write 不误判变化
            prev_positions_[i] = 0.0;

            RCLCPP_INFO(rclcpp::get_logger("RMRos2ControlHardware"), "Joint %zu: %s (init command=NaN)", i, joint_names_[i].c_str());
        }

        // Initialize realtime buffers with NaN commands
        std::vector<double> initial_commands(arm_dof_, kNaN);
        position_command_buffer_.writeFromNonRT(initial_commands);

        // Initialize joint message structure (based on rm_control)
        joint_msg_.joint.resize(arm_dof_);
        joint_msg_.dof = arm_dof_;
        joint_msg_.follow = follow_;
        joint_msg_.expand = 0.0;

        RCLCPP_INFO(rclcpp::get_logger("RMRos2ControlHardware"), "Successfully configured RM Hardware!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> RMRos2ControlHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (auto i = 0u; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> RMRos2ControlHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (auto i = 0u; i < info_.joints.size(); i++)
        {
            // Only export position command interface
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
            RCLCPP_INFO(
                rclcpp::get_logger("RMRos2ControlHardware"),
                "Joint %u (%s) - Exported position command interface.",
                i, joint_names_[i].c_str());

        }

        return command_interfaces;
    }

    hardware_interface::CallbackReturn RMRos2ControlHardware::on_activate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("RMRos2ControlHardware"), "Activating RM Hardware...");
        // 保持 on_configure 中的 NaN 命令，直到控制器第一次写入有效命令；避免激活瞬间跳到 initial_positions_
        // 如果需要使用当前位置作为初始状态，可在 read() 回调更新后再由控制器发送命令。
        for (auto i = 0u; i < hw_commands_.size(); i++) {
            RCLCPP_INFO(
                rclcpp::get_logger("RMRos2ControlHardware"),
                "Joint %u (%s) - Command: NaN (waiting first valid command)",
                i, joint_names_[i].c_str());
        }

        last_command_time_ = std::chrono::steady_clock::now();
        last_state_time_ = std::chrono::steady_clock::now();
        prev_state_time_ = last_state_time_;

        RCLCPP_INFO(rclcpp::get_logger("RMRos2ControlHardware"), "Successfully activated RM Hardware!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RMRos2ControlHardware::on_deactivate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("RMRos2ControlHardware"), "Deactivating RM Hardware...");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type RMRos2ControlHardware::read(
        const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
    {
        // Update joint states from robot feedback
        update_joint_states();

        // Spin the node to process callbacks
        rclcpp::spin_some(node_);

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type RMRos2ControlHardware::write(
        const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
    {
        // Check if command has changed significantly
        bool command_changed = false;
        for (size_t i = 0; i < hw_commands_.size(); ++i) {
            // 如果当前或之前命令是 NaN，跳过变化判定，直到第一次收到正常数字
            if (std::isnan(hw_commands_[i]) || std::isnan(prev_commands_[i])) {
                continue;
            }
            if (std::abs(hw_commands_[i] - prev_commands_[i]) > 1e-6) {
                command_changed = true;
                prev_commands_[i] = hw_commands_[i];
            }
        }

        // Check timing constraints
        auto current_time = std::chrono::steady_clock::now();
        auto time_since_last_command = std::chrono::duration_cast<std::chrono::milliseconds>(
            current_time - last_command_time_).count();

        // Send command if changed or if timeout reached (keep-alive)
        double timeout_ms = (1.0 / publish_rate_) * 1000.0; // Convert rate to timeout
        if (command_changed || time_since_last_command > timeout_ms) {
            // 仅当所有命令有效（非 NaN）才发送，避免启动阶段错误指令
            bool all_valid = true;
            for (const auto& cmd : hw_commands_) {
                if (std::isnan(cmd)) { all_valid = false; break; }
            }
            if (all_valid) {
                send_joint_commands();
                last_command_time_ = current_time;
            }
        }

        return hardware_interface::return_type::OK;
    }

    void RMRos2ControlHardware::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        joint_state_buffer_.writeFromNonRT(msg);
    }



    void RMRos2ControlHardware::send_joint_commands()
    {
        // Prepare joint command message (following rm_control pattern)
        joint_msg_.follow = follow_;

        for (size_t i = 0; i < static_cast<size_t>(arm_dof_); ++i) {
            joint_msg_.joint[i] = static_cast<float>(hw_commands_[i]);
        }

        // Publish the joint command
        joint_pos_publisher_->publish(joint_msg_);
    }

    void RMRos2ControlHardware::update_joint_states()
    {
        auto joint_state_ptr = joint_state_buffer_.readFromRT();
        if (joint_state_ptr && *joint_state_ptr) {
            const auto& joint_state = **joint_state_ptr;

            if (joint_state.name.size() >= static_cast<size_t>(arm_dof_)) {
                // 新增: 根据 joint_state.name 重建索引映射，确保按照 joint1..jointN 顺序写入内部数组
                // 说明: sensor_msgs::JointState 的 name/position/velocity/effort 是并行数组，可能顺序不同于内部假设
                // 做法: 为每个内部关节 (joint_names_[i] 或 "joint"+i) 找到其在 joint_state.name 中的索引，若未找到则跳过更新该关节
                std::vector<size_t> name_remap(arm_dof_, static_cast<size_t>(-1));
                if (joint_state.name.size() == joint_state.position.size()) {
                    for (size_t external_idx = 0; external_idx < joint_state.name.size(); ++external_idx) {
                        const std::string& ext_name = joint_state.name[external_idx];
                        // 依次匹配内部 joint_names_[i] 或 规范名称 "joint"+编号
                        for (size_t internal_i = 0; internal_i < static_cast<size_t>(arm_dof_); ++internal_i) {
                            const std::string expected_alias = std::string("joint") + std::to_string(internal_i + 1);
                            if (ext_name == joint_names_[internal_i] || ext_name == expected_alias) {
                                name_remap[internal_i] = external_idx;
                                break;
                            }
                        }
                    }
                }
                else {
                    // 名称与位置长度不匹配时，不能安全重排，保持原逻辑 (仍按索引 0..dof-1 使用)
                    return;
                }
                // 时间差计算
                auto now_tp = std::chrono::steady_clock::now();
                double dt = std::chrono::duration<double>(now_tp - prev_state_time_).count();
                bool valid_dt = dt > 1e-6 && dt < 1.0; // 上限防止长时间无更新导致巨大速度

                for (size_t internal_i = 0; internal_i < static_cast<size_t>(arm_dof_); ++internal_i) {
                    size_t external_idx = name_remap[internal_i];
                    if (external_idx == static_cast<size_t>(-1) || external_idx >= joint_state.position.size()) {
                        // 未找到匹配，跳过该关节的更新（保持旧值）
                        continue;
                    }
                    double new_pos = joint_state.position[external_idx];
                    double raw_vel = 0.0;
                    hw_positions_[internal_i] = new_pos;
                    if (joint_state.velocity.size() > external_idx) {
                        // 如果消息内直接提供速度，优先使用
                        raw_vel = joint_state.velocity[external_idx];
                        hw_velocities_[internal_i] = raw_vel;
                    }
                    if (joint_state.effort.size() > external_idx) {
                        hw_efforts_[internal_i] = joint_state.effort[external_idx];
                    }
                    // 若未提供速度，使用位置差分估算
                    if (joint_state.velocity.size() <= external_idx) {
                        if (valid_dt) {
                            raw_vel = (new_pos - prev_positions_[internal_i]) / dt;
                        }
                        else {
                            raw_vel = 0.0; // 第一次或无效 dt
                        }
                        hw_velocities_[internal_i] = raw_vel;
                    }
                    prev_positions_[internal_i] = new_pos;
                }
                prev_state_time_ = now_tp;
                last_state_time_ = now_tp;
                return;
            }
        }


    }

}  // namespace rm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    rm_hardware::RMRos2ControlHardware, hardware_interface::SystemInterface)
