#include "hexaarm_controller/hexaarm_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace hexaarm_controller
{
    HexaarmInterface::HexaarmInterface()
    {
    }

    HexaarmInterface::~HexaarmInterface()
    {
        if (arduino_.IsOpen())
        {
            try
            {
                arduino_.Close();
            }
            catch (...)
            {
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("HexaarmInterface"), "Something went wrong while closing the connection with port " << port_);
            }
        }
    }

    CallbackReturn HexaarmInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
    {
        CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);

        if (result != CallbackReturn::SUCCESS)
        {
            return result;
        }

        try
        {
            port_ = info_.hardware_parameters.at("port");
        }
        catch (const std::out_of_range &e)
        {
            RCLCPP_FATAL(rclcpp::get_logger("HexaarmInterface"), "No Serial Port provided! Aborting");
            return CallbackReturn::FAILURE;
        }

        position_commands_.reserve(info_.joints.size());
        position_states_.reserve(info_.joints.size());
        prev_position_commands_.reserve(info_.joints.size());

        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> HexaarmInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> HexaarmInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]));
        }
        return command_interfaces;
    }

    CallbackReturn HexaarmInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("HexaarmInterface"), "Starting the robot hardware...");

        position_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        prev_position_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        position_states_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        try
        {
            arduino_.Open(port_);
            arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        }
        catch (...)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("HexaarmInterface"), "Failed to inrterface with the port " << port_);
            return CallbackReturn::FAILURE;
        }
        RCLCPP_INFO(rclcpp::get_logger("HexaarmInterface"), "Hardware started, ready to take commands ");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn HexaarmInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("HexaarmInterface"), "Stopping the robot hardware...");
        if (arduino_.IsOpen())
        {
            try
            {
                arduino_.Close();
            }
            catch (...)
            {
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("HexaarmInterface"), "Failed to close the connection with the port " << port_);
                return CallbackReturn::FAILURE;
            }
        }

        RCLCPP_INFO(rclcpp::get_logger("HexaarmInterface"), "Hardware stopped");
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type HexaarmInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        position_states_ = position_commands_;
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type HexaarmInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        if (position_commands_ == prev_position_commands_)
        {
            return hardware_interface::return_type::OK;
        }

        std::string msg;
        int base = static_cast<int>((position_commands_.at(0) + (M_PI / 2)) * 180) / M_PI;
        msg.append("b");
        msg.append(std::to_string(base));
        msg.append(",");
        int shoulder = 180 - static_cast<int>(((position_commands_.at(1) + (M_PI / 2)) * 180) / M_PI);
        msg.append("s");
        msg.append(std::to_string(shoulder));
        msg.append(",");
        int elbow = static_cast<int>(((position_commands_.at(2) + (M_PI / 2)) * 180) / M_PI);
        msg.append("e");
        msg.append(std::to_string(elbow));
        msg.append(",");
        int wrist = static_cast<int>(((position_commands_.at(3) + (M_PI / 2)) * 180) / M_PI);
        msg.append("w");
        msg.append(std::to_string(wrist));
        msg.append(",");
        int ulna = static_cast<int>(((position_commands_.at(4) + (M_PI / 2)) * 180) / M_PI);
        msg.append("u");
        msg.append(std::to_string(ulna));
        msg.append(",");
        int femur = static_cast<int>(((position_commands_.at(4) + (M_PI / 2)) * 180) / M_PI);
        msg.append("f");
        msg.append(std::to_string(femur));
        msg.append(",");
        int gripper = static_cast<int>((-position_commands_.at(5) * 180) / (M_PI / 2));
        msg.append("g");
        msg.append(std::to_string(gripper));
        msg.append(",");
        try
        {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("HexaarmInterface"), "Sending new command " << msg);
            arduino_.Write(msg);
        }
        catch (...)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("HexaarmInterface"), "Something wen wrong while sending the message " << msg);
            return hardware_interface::return_type::ERROR;
        }

        prev_position_commands_ = position_commands_;
        return hardware_interface::return_type::OK;
    }
}

PLUGINLIB_EXPORT_CLASS(hexaarm_controller::HexaarmInterface, hardware_interface::SystemInterface);