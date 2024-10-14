#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>
#include <string>

class HelloWorldPubNode : public rclcpp::Node
{
    public:
        HelloWorldPubNode() : Node("hello_world_pub_node"), counter_(0)
        {
            publisher_ = this->create_publisher<std_msgs::msg::String>("hello_world", 10);
            timer_ = this->create_wall_timer(
                std::chrono::seconds(1),
                std::bind(&HelloWorldPubNode::publish_hello_world, this));
        }

    private:
        void publish_hello_world()
        {
            auto message = std_msgs::msg::String();
            message.data = "Hello world " + std::to_string(counter_);

            publisher_->publish(message);
            counter_++;
        }

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        int counter_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HelloWorldPubNode>());
    rclcpp::shutdown();

    return 0;
}
