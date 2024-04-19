#include <thread>

#include "agent/go1_agent.h"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/accel.hpp"
#include "global_log.h"

class Ros2Agent : public Go1Agent, public rclcpp::Node
{
public:
    Ros2Agent() : Go1Agent(ModelType::GO1_FLAT), Node("orbit_deploy")
    {
        twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/go1/twist", 10, std::bind(&Ros2Agent::twist_callback, this, std::placeholders::_1));
        twist_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&Ros2Agent::twist_cmd_callback, this, std::placeholders::_1));
        accel_sub_ = this->create_subscription<geometry_msgs::msg::Accel>(
            "/go1/gravity", 10, std::bind(&Ros2Agent::accel_callback, this, std::placeholders::_1));
        agent_thread_ = std::thread(&Go1Agent::Run, this);
        agent_thread_.detach();
    }
    ~Ros2Agent() {}
private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_cmd_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Accel>::SharedPtr accel_sub_;
    std::thread agent_thread_;
    void twist_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received velocity command: %f, %f, %f", msg->linear.x, msg->linear.y, msg->angular.z);
        SetVelocityCommands(msg->linear.x, msg->linear.y, msg->angular.z);
    }
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received twist: %f, %f, %f", msg->linear.x, msg->linear.y, msg->linear.z);
        SetBaseLinVel(msg->linear.x, msg->linear.y, msg->linear.z);
    }
    void accel_callback(const geometry_msgs::msg::Accel::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received projected gravity: %f, %f, %f", msg->linear.x, msg->linear.y, msg->linear.z);
        SetProjectedGravity(msg->linear.x, msg->linear.y, msg->linear.z);
    }
    void ros2_agent_run()
    {
        Run();
    }
};

int main(int argc, char *argv[])
{
    InitLogger();
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Ros2Agent>());
    rclcpp::shutdown();
    return 0;
}
