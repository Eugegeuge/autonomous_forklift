#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class SimpleController : public rclcpp::Node
{
public:
    SimpleController() : Node("simple_controller")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/forklift1/cmd_vel", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&SimpleController::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Simple Controller Started. Moving forklift forward and turning.");
    }

private:
    void timer_callback()
    {
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = 0.5;
        message.angular.z = 0.2;
        publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleController>());
    rclcpp::shutdown();
    return 0;
}
