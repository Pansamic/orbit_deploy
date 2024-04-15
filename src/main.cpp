#include "agent/go1_agent.h"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/accel.hpp"


class Ros2Agent : public Go1Agent, public rclcpp::Node
{
public:
    Ros2Agent() : Go1Agent(ModelType::GO1_ROUGH), Node("orbit_deploy")
    {
        twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&Ros2Agent::twist_callback, this, std::placeholders::_1));
        accel_sub_ = this->create_subscription<geometry_msgs::msg::Accel>(
            "cmd_accel", 10, std::bind(&Ros2Agent::accel_callback, this, std::placeholders::_1));
    }
    ~Ros2Agent() {}
private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Accel>::SharedPtr accel_sub_;

    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std::cout << "Received twist command: " << msg->linear.x << ", " << msg->linear.y << ", " << msg->linear.z << std::endl;
    }

    void accel_callback(const geometry_msgs::msg::Accel::SharedPtr msg)
    {
        std::cout << "Received accel command: " << msg->linear.x << ", " << msg->linear.y << ", " << msg->linear.z << std::endl;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Ros2Agent>());
    rclcpp::shutdown();
    return 0;
}
