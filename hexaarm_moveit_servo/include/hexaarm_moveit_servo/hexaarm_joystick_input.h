#ifndef __HEXAARM_JOY_STICK_INPUT_H__
#define __HEXAARM_JOY_STICK_INPUT_H__

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace hexaarm_moveit_servo {

class JoyToServoPub : public rclcpp::Node {
public:
    JoyToServoPub(const rclcpp::NodeOptions& options);
    ~JoyToServoPub();

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void filterTwistMsg(geometry_msgs::msg::TwistStamped& twist, double val = 0.05);

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    std::string mode_;  // "joint_control" or "cartesian_control"
};

} // namespace hexaarm_moveit_servo

#endif 