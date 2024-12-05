#include "hexaarm_controller/hexaarm_hardware.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace hexaarm_hardware
{
    HEXAArm::HEXAArm()
    {
    }

    HEXAArm::~HEXAArm()
    {
        if (serial_port_.IsOpen())
        {
            try
            {
                serial_port_.Close();
            }
            catch (...)
            {
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("hexaarm_hardware"), "Failed to close serial port - " << serial_port_name_);
            }
        }
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn HEXAArm::on_init(const hardware_interface::HardwareInfo &hardware_info)
    {
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
        if (result != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS)
        {
            return result;
        }

        try
        {
            serial_port_name_ = info_.hardware_parameters.at("serial_port_name_");
        }
        catch (const std::out_of_range &e)
        {
            RCLCPP_FATAL(rclcpp::get_logger("hexaarm_hardware"), "Failed to get serial port name from hardware_info ");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
        }

        position_commands.reserve(info_.joints.size());
        position_states.reserve(info_.joints.size());
        prev_position_commands.reserve(info_.joints.size());

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> HEXAArm::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name,
                hardware_interface::HW_IF_POSITION,
                &position_states[i]));
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> HEXAArm::export_command_interfaces()
    {

        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name,
                hardware_interface::HW_IF_POSITION,
                &position_commands[i]));
        }
        return command_interfaces;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn HEXAArm::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("hexaarm_hardware"), "Activating HEXAArm . ...");

        // Reset command and state vectors
        position_commands = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        position_states = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        prev_position_commands = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        try
        {
            serial_port_.Open(serial_port_name_);
            serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        }
        catch (...)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("hexaarm_hardware"), "Failed to initialize serial port - " << serial_port_name_);
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
        }

        RCLCPP_INFO(rclcpp::get_logger("hexaarm_hardware"), "HEXAArm activated successfully, Ready to receive commands ...");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn HEXAArm::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("hexaarm_hardware"), "Deactivating HEXAArm ...");
        if (serial_port_.IsOpen())
        {
            try
            {
                serial_port_.Close();
            }
            catch (...)
            {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("hexaarm_hardware"), "Failed to close serial port - " << serial_port_name_);
            }
        }

        RCLCPP_INFO(rclcpp::get_logger("hexaarm_hardware"), "HEXAArm deactivated successfully.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type HEXAArm::read(
        const rclcpp::Time &time,
        const rclcpp::Duration &period)
    {
        // Read from serial port, parse data and update position_states vector
        // Example:
        // std::string data = serial_port_.ReadLine();
        // std::istringstream iss(data);
        // iss >> position_states[0] >> position_states[1] >> position_states[2] >> position_states[3] >> position_states[4];
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type HEXAArm::write(
        const rclcpp::Time &time,
        const rclcpp::Duration &period)
    {
        // Write position_commands vector to serial port, in this format joint1, joint2, joint3, joint4, joint5\n
        if (position_commands == prev_position_commands)
        {
            return hardware_interface::return_type::OK;
        }

        std::string serial_data_packet;

        int joint1 = static_cast<int>(position_commands[0] * 180 / M_PI);
        serial_data_packet.append(std::to_string(joint1) + ",");
        int joint2 = static_cast<int>(position_commands[1] * 180 / M_PI);
        serial_data_packet.append(std::to_string(joint2) + ",");
        int joint3 = static_cast<int>(position_commands[2] * 180 / M_PI);
        serial_data_packet.append(std::to_string(joint3) + ",");
        int joint4 = static_cast<int>(position_commands[3] * 180 / M_PI);
        serial_data_packet.append(std::to_string(joint4) + ",");
        int joint5 = static_cast<int>(position_commands[4] * 180 / M_PI);
        serial_data_packet.append(std::to_string(joint5) + "\n");
        int joint6 = static_cast<int>(position_commands[5] * 180 / M_PI);
        serial_data_packet.append(std::to_string(joint6) + "\n");
        int joint7 = static_cast<int>(position_commands[6] * 180 / M_PI);
        serial_data_packet.append(std::to_string(joint7) + "\n");

        try
        {
            if (i == 0)
            {
                serial_port_.Write(serial_data_packet);
                RCLCPP_INFO_STREAM(rclcpp::get_logger("hexaarm_hardware"), "Sent command to serial port: " << serial_data_packet);
                i = 1;
            }
        }
        catch (...)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("hexaarm_hardware"), "Failed to write to serial port - " << serial_port_name_);
            return hardware_interface::return_type::ERROR;
        }

        prev_position_commands = position_commands;
        return hardware_interface::return_type::OK;
    }
}
PLUGINLIB_EXPORT_CLASS(hexaarm_hardware::HEXAArm , hardware_interface::SystemInterface)