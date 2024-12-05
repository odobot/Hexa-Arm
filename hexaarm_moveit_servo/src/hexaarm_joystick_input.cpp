#include "hexaarm_moveit_servo/hexaarm_joystick_input.h"

namespace hexaarm_moveit_servo
{

    JoyToServoPub::JoyToServoPub(const rclcpp::NodeOptions &options)
        : Node("joy_to_twist_publisher", options), mode_("cartesian_control")
    {

        // Create publishers
        twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/servo_server/delta_twist_cmds", 0.1);
        joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>("/servo_server/delta_joint_cmds", 0.1);

        // Create a client to start the ServoServer
        servo_start_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_server/start_servo");
        servo_start_client_->wait_for_service(std::chrono::milliseconds(100));
        servo_start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());

        // Create a subscription to the joy topic
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 0.1, std::bind(&JoyToServoPub::joyCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Joy to Servo Publisher initialized.");
    }

    JoyToServoPub::~JoyToServoPub()
    {
        // Destructor logic (if needed)
    }

    void JoyToServoPub::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        auto twist_msg = geometry_msgs::msg::TwistStamped();
        auto joint_msg = control_msgs::msg::JointJog();

        twist_msg.header.stamp = this->now();
        twist_msg.header.frame_id = "base_link";
        joint_msg.header.stamp = this->now();
        joint_msg.header.frame_id = "Revolute";

        // Example joint control logic
        if (msg->buttons[0] == 1)
        {
            joint_msg.joint_names.push_back("base_shoulder");
            joint_msg.velocities.push_back(5);
        }

        if (msg->buttons[1] == 1)
        {
            joint_msg.joint_names.push_back("base_shoulder");
            joint_msg.velocities.push_back(-5);
        }
        if (msg->buttons[3] == 1)
        {
            joint_msg.joint_names.push_back("shoulder_elbow");
            joint_msg.velocities.push_back(5);
        }

        if (msg->buttons[4] == 1)
        {
            joint_msg.joint_names.push_back("shoulder_elbow");
            joint_msg.velocities.push_back(-5);
        }
        if (msg->buttons[6] == 1)
        {
            joint_msg.joint_names.push_back("elbow_wrist");
            joint_msg.velocities.push_back(5);
        }

        if (msg->buttons[7] == 1)
        {
            joint_msg.joint_names.push_back("elbow_wrist");
            joint_msg.velocities.push_back(-5);
        }
        if (msg->buttons[8] == 1)
        {
            joint_msg.joint_names.push_back("wrist_ulna");
            joint_msg.velocities.push_back(5);
        }

        if (msg->buttons[9] == 1)
        {
            joint_msg.joint_names.push_back("wrist_ulna");
            joint_msg.velocities.push_back(-5);
        }
        if (msg->buttons[14] == 1)
        {
            joint_msg.joint_names.push_back("ulna_febur");
            joint_msg.velocities.push_back(-5);
        }

        if (msg->buttons[13] == 1)
        {
            joint_msg.joint_names.push_back("ulna_febur");
            joint_msg.velocities.push_back(5);
        }

        if (msg->axes[7] == 1)
        {
            joint_msg.joint_names.push_back("febur_gripper");
            joint_msg.velocities.push_back(5);
        }

        if (msg->axes[7] == -1)
        {
            joint_msg.joint_names.push_back("febur_gripper");
            joint_msg.velocities.push_back(-5);
        }

        if (msg->axes[8] == 1)
        {
            joint_msg.joint_names.push_back("right_finger");
            joint_msg.velocities.push_back(5);
        }

        if (msg->axes[8] == -1)
        {
            joint_msg.joint_names.push_back("right_finger");
            joint_msg.velocities.push_back(-5);
        }

        // Implement joint control using buttons and axes as needed
        joint_pub_->publish(joint_msg);

        // // Mode switching logic (A for joint control, B for Cartesian control)
        // if (msg->buttons[10] == 1)
        // { // SELECT button
        //     mode_ = "joint_control";
        //     RCLCPP_INFO(this->get_logger(), "Switched to Joint Control Mode");
        // }
        // else if (msg->buttons[11] == 1)
        // { // START button
        //     mode_ = "cartesian_control";
        //     RCLCPP_INFO(this->get_logger(), "Switched to Cartesian Control Mode");
        // }

        // if (mode_ == "cartesian_control")
        // {
        //     // Use axes for Cartesian control
        //     twist_msg.twist.linear.x = msg->axes[0] * 0.01;                                                  // Left stick X-axis
        //     twist_msg.twist.linear.y = msg->axes[1] * 0.01;                                                  // Left stick Y-axis
        //     twist_msg.twist.linear.z = (msg->buttons[3] == 1) ? 0.01 : (msg->buttons[4] == 1 ? -0.01 : 0.0); // X/Y buttons for z-axis
        //     twist_msg.twist.angular.z = msg->axes[2] * 0.01;                                                 // Right stick X-axis for rotation

        //     filterTwistMsg(twist_msg, 0.001); // Filter to remove noise
        //     twist_pub_->publish(twist_msg);
        // }
        // else if (mode_ == "joint_control")
        // {
        //     // Example joint control logic
        //     if (msg->buttons[0] == 1)
        //     {
        //         joint_msg.joint_names.push_back("base_shoulder");
        //         joint_msg.velocities.push_back(5);
        //     }

        //     if (msg->buttons[1] == 1)
        //     {
        //         joint_msg.joint_names.push_back("base_shoulder");
        //         joint_msg.velocities.push_back(-5);
        //     }
        //     if (msg->buttons[3] == 1)
        //     {
        //         joint_msg.joint_names.push_back("shoulder_elbow");
        //         joint_msg.velocities.push_back(5);
        //     }

        //     if (msg->buttons[4] == 1)
        //     {
        //         joint_msg.joint_names.push_back("shoulder_elbow");
        //         joint_msg.velocities.push_back(-5);
        //     }
        //     if (msg->buttons[6] == 1)
        //     {
        //         joint_msg.joint_names.push_back("elbow_wrist");
        //         joint_msg.velocities.push_back(5);
        //     }

        //     if (msg->buttons[7] == 1)
        //     {
        //         joint_msg.joint_names.push_back("elbow_wrist");
        //         joint_msg.velocities.push_back(-5);
        //     }
        //     if (msg->buttons[8] == 1)
        //     {
        //         joint_msg.joint_names.push_back("wrist_ulna");
        //         joint_msg.velocities.push_back(5);
        //     }

        //     if (msg->buttons[9] == 1)
        //     {
        //         joint_msg.joint_names.push_back("wrist_ulna");
        //         joint_msg.velocities.push_back(-5);
        //     }
        //     if (msg->buttons[14] == 1)
        //     {
        //         joint_msg.joint_names.push_back("ulna_febur");
        //         joint_msg.velocities.push_back(-5);
        //     }

        //     if (msg->buttons[13] == 1)
        //     {
        //         joint_msg.joint_names.push_back("ulna_febur");
        //         joint_msg.velocities.push_back(5);
        //     }

        //     if (msg->axes[7] == 1)
        //     {
        //         joint_msg.joint_names.push_back("febur_gripper");
        //         joint_msg.velocities.push_back(5);
        //     }

        //     if (msg->axes[7] == -1)
        //     {
        //         joint_msg.joint_names.push_back("febur_gripper");
        //         joint_msg.velocities.push_back(-5);
        //     }

        //     if (msg->axes[8] == 1)
        //     {
        //         joint_msg.joint_names.push_back("right_finger");
        //         joint_msg.velocities.push_back(5);
        //     }

        //     if (msg->axes[8] == -1)
        //     {
        //         joint_msg.joint_names.push_back("right_finger");
        //         joint_msg.velocities.push_back(-5);
        //     }

        //     // Implement joint control using buttons and axes as needed
        //     joint_pub_->publish(joint_msg);
        // }
    }

    void JoyToServoPub::filterTwistMsg(geometry_msgs::msg::TwistStamped &twist, double val)
    {
        if (std::abs(twist.twist.linear.x) < val)
            twist.twist.linear.x = 0;
        if (std::abs(twist.twist.linear.y) < val)
            twist.twist.linear.y = 0;
        if (std::abs(twist.twist.linear.z) < val)
            twist.twist.linear.z = 0;
        if (std::abs(twist.twist.angular.x) < val)
            twist.twist.angular.x = 0;
        if (std::abs(twist.twist.angular.y) < val)
            twist.twist.angular.y = 0;
        if (std::abs(twist.twist.angular.z) < val)
            twist.twist.angular.z = 0;
    }

} // namespace hexaarm_moveit_servo

// Register with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(hexaarm_moveit_servo::JoyToServoPub)
